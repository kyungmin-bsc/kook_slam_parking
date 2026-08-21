#!/usr/bin/env python3
"""
지도에 없는 장애물을 실시간으로 찾아내서 RViz에 띄우는 노드.

Nav2의 costmap obstacle_layer도 라이다를 반영해서 회피 경로를 만들어주지만,
costmap은 "비용이 높은 셀"로만 보여서 **당일 새로 놓인 장애물인지, 원래 지도에
있던 벽인지 눈으로 구분이 안 된다.** 이 노드는 그 둘을 분리해서 새 장애물만
빨간 마커로 띄운다. 대회 당일 트랙에 뭐가 추가됐는지 즉시 파악하기 위한 것.

원리
----
/scan의 각 점을 map 프레임으로 옮긴 뒤, 정적 지도(/map)의 해당 위치를 본다.
  - 그 근처에 지도상 장애물이 있다  -> 원래 있던 벽. 무시.
  - 근처가 전부 비어 있다            -> 지도에 없는 새 장애물. 표시.

'근처'로 보는 이유: 위치추정 오차 때문에 벽 스캔점이 지도 벽에서 몇 cm 어긋나
찍힌다. 정확히 같은 셀만 보면 벽 전체가 새 장애물로 오검출된다.
match_radius가 그 허용 오차다.

노이즈 억제
----------
단발 스캔점 하나로 장애물이라 판정하지 않는다. 격자로 묶어서(cluster_size)
한 칸에 min_points개 이상 모인 경우만 인정하고, 연속 프레임에서 반복 관측된
것에 가중치를 준다(persistence).
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def map_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ObstacleMonitor(Node):

    def __init__(self):
        super().__init__('obstacle_monitor')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('marker_topic', '/obstacles/markers')
        self.declare_parameter('grid_topic', '/obstacles/grid')
        self.declare_parameter('laser_frame', 'laser_frame')

        # 지도상 장애물이 이 반경 안에 있으면 '원래 있던 벽'으로 본다.
        # 위치추정 오차 + 지도 해상도(0.05)를 감안한 값.
        self.declare_parameter('match_radius', 0.18)
        # 새 장애물 격자 크기
        self.declare_parameter('cluster_size', 0.10)
        # 한 격자에 이만큼 점이 모여야 장애물로 인정
        self.declare_parameter('min_points', 3)
        # 이만큼 연속 관측되어야 마커로 띄운다 (스파이크 노이즈 제거)
        self.declare_parameter('min_hits', 2)
        # 이 시간 동안 재관측 없으면 마커에서 지운다 (치워진 장애물)
        self.declare_parameter('forget_sec', 3.0)
        # 너무 먼 점은 신뢰도가 낮아 무시
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('min_range', 0.15)

        self.match_radius = float(self.get_parameter('match_radius').value)
        self.cluster_size = float(self.get_parameter('cluster_size').value)
        self.min_points = int(self.get_parameter('min_points').value)
        self.min_hits = int(self.get_parameter('min_hits').value)
        self.forget_sec = float(self.get_parameter('forget_sec').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.laser_frame = self.get_parameter('laser_frame').value

        self._map: Optional[OccupancyGrid] = None
        # (gx, gy) -> [hits, last_seen_sec]
        self._tracks: Dict[Tuple[int, int], list] = {}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self._on_map, map_qos())
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan,
            rclpy.qos.qos_profile_sensor_data)
        self._pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 1)
        # 통행 판정용. 마커는 사람이 보는 것이고, 이 격자는 mission_manager가
        # passability 판정에 쓴다. 확정된 장애물 셀만 100으로 찍는다.
        self._grid_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('grid_topic').value, map_qos())

        self.get_logger().info(
            '장애물 모니터 시작 (match_radius=%.2fm, cluster=%.2fm, min_points=%d)'
            % (self.match_radius, self.cluster_size, self.min_points))

    # -- 지도 ---------------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg
        self.get_logger().info(
            '정적 지도 수신: %dx%d @ %.3fm/px'
            % (msg.info.width, msg.info.height, msg.info.resolution))

    def _static_occupied_near(self, x: float, y: float) -> bool:
        """map 좌표 (x,y) 주변 match_radius 안에 지도상 장애물이 있는가."""
        g = self._map
        info = g.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        span = int(math.ceil(self.match_radius / res))

        for dy in range(-span, span + 1):
            for dx in range(-span, span + 1):
                if dx * dx + dy * dy > span * span:
                    continue
                px, py = cx + dx, cy + dy
                if not (0 <= px < info.width and 0 <= py < info.height):
                    # 지도 밖은 '미지'이므로 새 장애물로 취급하지 않는다
                    return True
                v = g.data[py * info.width + px]
                if v < 0 or v >= 50:      # 미지(-1) 또는 점유
                    return True
        return False

    # -- 스캔 ---------------------------------------------------------------

    def _on_scan(self, scan: LaserScan) -> None:
        if self._map is None:
            return
        try:
            tr = self._tf_buffer.lookup_transform(
                'map', scan.header.frame_id or self.laser_frame,
                rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn('map<-라이다 TF 없음: %s' % exc,
                                   throttle_duration_sec=3.0)
            return

        t = tr.transform.translation
        yaw = yaw_from_quat(tr.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        now = self.get_clock().now().nanoseconds * 1e-9

        # 1) 지도에 없는 점만 골라 격자에 모은다
        buckets: Dict[Tuple[int, int], int] = {}
        ang = scan.angle_min
        for r in scan.ranges:
            a = ang
            ang += scan.angle_increment
            if not (self.min_range < r < self.max_range) or math.isinf(r) or math.isnan(r):
                continue
            lx, ly = r * math.cos(a), r * math.sin(a)
            mx = t.x + c * lx - s * ly
            my = t.y + s * lx + c * ly
            if self._static_occupied_near(mx, my):
                continue
            key = (int(math.floor(mx / self.cluster_size)),
                   int(math.floor(my / self.cluster_size)))
            buckets[key] = buckets.get(key, 0) + 1

        # 2) 충분히 점이 모인 격자만 추적 대상으로 승격
        for key, n in buckets.items():
            if n < self.min_points:
                continue
            rec = self._tracks.get(key)
            if rec is None:
                self._tracks[key] = [1, now]
            else:
                rec[0] += 1
                rec[1] = now

        # 3) 오래 안 보인 건 잊는다 (치워진 장애물)
        for key in [k for k, v in self._tracks.items()
                    if now - v[1] > self.forget_sec]:
            del self._tracks[key]

        self._publish(now)

    # -- 발행 ---------------------------------------------------------------

    def _publish(self, now: float) -> None:
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        confirmed = [(k, v) for k, v in self._tracks.items() if v[0] >= self.min_hits]

        stamp = self.get_clock().now().to_msg()
        for i, (key, rec) in enumerate(confirmed):
            gx, gy = key
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'unmapped_obstacle'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = Point(
                x=(gx + 0.5) * self.cluster_size,
                y=(gy + 0.5) * self.cluster_size,
                z=0.10)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = self.cluster_size
            m.scale.z = 0.20
            # 최근 관측일수록 진하게
            fresh = max(0.0, 1.0 - (now - rec[1]) / self.forget_sec)
            m.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.45 + 0.45 * fresh)
            arr.markers.append(m)

        if confirmed:
            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = stamp
            label.ns = 'unmapped_obstacle_count'
            label.id = 0
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            gx, gy = confirmed[0][0]
            label.pose.position = Point(x=(gx + 0.5) * self.cluster_size,
                                        y=(gy + 0.5) * self.cluster_size, z=0.45)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.14
            label.color = ColorRGBA(r=1.0, g=0.35, b=0.35, a=1.0)
            label.text = '지도에 없는 장애물 %d곳' % len(confirmed)
            arr.markers.append(label)
            self.get_logger().warn('지도에 없는 장애물 %d곳 감지' % len(confirmed),
                                   throttle_duration_sec=3.0)

        self._pub.publish(arr)
        self._publish_grid(confirmed)

    def _publish_grid(self, confirmed) -> None:
        """확정 장애물을 정적 지도와 같은 좌표계/해상도의 격자로 발행.

        mission_manager가 /map과 이 격자를 겹쳐서 '이 경로가 아직 뚫려 있는가'를
        판정한다. 마커(MarkerArray)로는 그 계산을 할 수 없어서 따로 낸다.
        """
        base = self._map
        if base is None:
            return
        info = base.info
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info = info
        data = [0] * (info.width * info.height)

        # 클러스터 격자(cluster_size)는 지도 해상도보다 크므로 셀 여러 개를 덮는다
        span = max(1, int(round(self.cluster_size / info.resolution)))
        for (gx, gy), _rec in confirmed:
            wx = (gx + 0.5) * self.cluster_size
            wy = (gy + 0.5) * self.cluster_size
            cx = int((wx - info.origin.position.x) / info.resolution)
            cy = int((wy - info.origin.position.y) / info.resolution)
            for dy in range(-span // 2, span // 2 + 1):
                for dx in range(-span // 2, span // 2 + 1):
                    px, py = cx + dx, cy + dy
                    if 0 <= px < info.width and 0 <= py < info.height:
                        data[py * info.width + px] = 100
        g.data = data
        self._grid_pub.publish(g)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
