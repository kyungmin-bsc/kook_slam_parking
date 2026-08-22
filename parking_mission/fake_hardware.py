#!/usr/bin/env python3
"""
라이다 / VESC / 초음파를 대신하는 가상 하드웨어 노드.

`/xycar_motor`를 구독해 자전거 모델로 차를 굴리고, 지도에서 광선을 쏴서 `/scan`을
만들어 낸다. 루프가 닫히므로 실차 없이 미션 전체를 돌려볼 수 있다.

    [mission_manager / cmd_vel_bridge]
                 |  /xycar_motor
                 v
          [fake_hardware]  <-- /map (map_server가 로드한 것 그대로)
                 |
                 +--> /scan (laser_frame)      --> AMCL, costmap
                 +--> /vesc_odom + tf(odom->base_link)
                 +--> /xycar_ultrasonic         --> 후방 감시 가드
                 +--> /sim/true_pose            --> AMCL 오차 비교용

무엇을 검증할 수 있고 없는가
--------------------------
검증 가능:
  - AMCL이 출발 위치에 수렴하는가
  - Nav2가 우리 경유점 경로를 실제로 만들어내는가 (inflation_radius가 통로를
    막는지 여부가 여기서 드러난다)
  - mission_manager 상태 전이가 끝까지 도는가
  - 주차 기동/보정/초음파 정지가 실제 노드에서 동작하는가
  - 시각화 마커가 RViz에 제대로 뜨는가

검증 불가:
  - AMCL alpha 튜닝값 (실제 odom 노이즈 특성이 아니라 여기서 지어낸 값이다)
  - radius_scale (실제 타이어 슬립이 아니라 여기서 지어낸 값이다)
  - 라이다 실제 반사 특성, 초음파 경면 반사
  - ros1_bridge 지연/누락

즉 **코드 버그를 잡는 도구이지 물리 파라미터를 정하는 도구가 아니다.**
물리값은 CALIBRATION_GUIDE.md의 실차 절차로만 나온다.

실행
----
    ros2 launch parking_mission bringup.launch.py use_fake_hardware:=true

구동 오차를 넣어 보정 동작을 보려면:

    ros2 launch parking_mission bringup.launch.py use_fake_hardware:=true \\
        sim_radius_err:=1.15 sim_steer_bias:=2.5
"""

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from tf2_ros import TransformBroadcaster

from . import mission_config as cfg
from .geometry import Pose2D, wrap_angle
from .ultrasonic import (CH_LEFT, CH_REAR_CENTER, CH_REAR_LEFT, CH_REAR_RIGHT,
                         CH_RIGHT)

try:
    import numpy as np
except ImportError:  # numpy 없으면 광선추적이 너무 느려 못 쓴다
    np = None

# 실측 제원
WHEELBASE = 0.33
MAX_STEER = 35.0
SPEED_GAIN = 9.86          # /xycar_motor speed 단위 per (m/s)
MOVE_THRESHOLD = 2.5       # 실측: speed 2는 안 움직이고 3부터 굴러간다
LIDAR_X = 0.41             # base_link -> laser_frame
FRONT, REAR, HALFW = 0.46, -0.15, 0.15
ULTRA_CLAMP = 140          # 드라이버가 이 값을 넘기면 0으로 만든다


def map_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class FakeHardware(Node):

    def __init__(self):
        super().__init__('fake_hardware')

        if np is None:
            self.get_logger().error(
                'numpy가 없어 광선추적을 할 수 없다. pip install numpy')
            raise SystemExit(1)

        # --- 시작 자세 (기본: 대회 출발 위치) ---
        self.declare_parameter('start_x', cfg.START_POSE.x)
        self.declare_parameter('start_y', cfg.START_POSE.y)
        self.declare_parameter('start_yaw', cfg.START_POSE.yaw)

        # --- 구동 오차 주입 ---
        self.declare_parameter('radius_err', 1.0)    # 1.15 = 15% 덜 꺾임
        self.declare_parameter('steer_bias', 0.0)    # 조향 영점 오차 (deg)
        self.declare_parameter('servo_lag', 0.0)     # 서보 1차지연 시간상수 (s)
        self.declare_parameter('speed_err', 1.0)

        # --- odom 드리프트 (AMCL이 할 일이 있어야 한다) ---
        self.declare_parameter('odom_yaw_drift_per_m', 0.010)   # rad per m
        self.declare_parameter('odom_scale_err', 1.00)

        # --- 라이다 ---
        self.declare_parameter('scan_hz', 10.0)
        self.declare_parameter('scan_beams', 180)
        self.declare_parameter('scan_range_max', 12.0)
        self.declare_parameter('scan_range_min', 0.12)
        self.declare_parameter('scan_noise', 0.010)   # m, 1 sigma

        # --- 추가 장애물 (지도에 없는 것. "x,y;x,y" 형식) ---
        self.declare_parameter('obstacles', '')
        self.declare_parameter('obstacle_radius', 0.09)

        self.declare_parameter('motor_topic', '/xycar_motor')
        self.declare_parameter('publish_ultrasonic', True)

        p = self.get_parameter
        self.true = Pose2D(float(p('start_x').value),
                           float(p('start_y').value),
                           float(p('start_yaw').value))
        self.odom = Pose2D(0.0, 0.0, 0.0)     # odom 원점 = 출발 지점

        self.radius_err = float(p('radius_err').value)
        self.steer_bias = float(p('steer_bias').value)
        self.servo_lag = float(p('servo_lag').value)
        self.speed_err = float(p('speed_err').value)
        self.drift = float(p('odom_yaw_drift_per_m').value)
        self.scale_err = float(p('odom_scale_err').value)
        self.noise = float(p('scan_noise').value)
        self.obst_r = float(p('obstacle_radius').value)

        self.obstacles: List[tuple] = []
        raw = str(p('obstacles').value).strip()
        if raw:
            for part in raw.split(';'):
                if not part.strip():
                    continue
                sx, sy = part.split(',')
                self.obstacles.append((float(sx), float(sy)))

        self._grid: Optional[OccupancyGrid] = None
        self._occ = None            # numpy bool 배열 (h, w)
        self._cmd = (0.0, 0.0)      # (angle_deg, speed_units)
        self._applied_steer = 0.0
        self._last_cmd_t = None
        self._amcl: Optional[Pose2D] = None

        # --- 통신 ---
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos())
        self.create_subscription(Float32MultiArray, p('motor_topic').value,
                                 self._on_motor, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_amcl, 10)

        self._scan_pub = self.create_publisher(LaserScan, '/scan',
                                               qos_profile_sensor_data)
        self._odom_pub = self.create_publisher(Odometry, '/vesc_odom', 20)
        self._true_pub = self.create_publisher(PoseStamped, '/sim/true_pose', 10)
        self._tf = TransformBroadcaster(self)
        self._ultra_pub = None
        if bool(p('publish_ultrasonic').value):
            self._ultra_pub = self.create_publisher(
                Int32MultiArray, '/xycar_ultrasonic', 10)

        # --- 타이머 ---
        self._dt = 0.02                       # 50 Hz 물리/odom
        self.create_timer(self._dt, self._step)
        self.create_timer(1.0 / float(p('scan_hz').value), self._publish_scan)
        if self._ultra_pub is not None:
            # 20Hz로 돌리면 5개 원뿔 x 5광선 = 초당 500회 광선추적이라
            # controller_server의 20Hz 제어 루프를 밀어낸다
            # ("Control loop missed its desired rate of 20.0Hz").
            # 실제 드라이버도 30Hz 루프에서 시리얼이 오는 만큼만 내므로 10Hz면 충분하다.
            self.create_timer(0.10, self._publish_ultra)
        self.create_timer(3.0, self._report)

        # --- 빔 각도 사전계산 ---
        n = int(p('scan_beams').value)
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_inc = (self.angle_max - self.angle_min) / n
        self._beam = np.arange(n) * self.angle_inc + self.angle_min
        self.range_max = float(p('scan_range_max').value)
        self.range_min = float(p('scan_range_min').value)

        self.get_logger().info(
            '가상 하드웨어 시작. 출발 %s | 오차 radius=%.2f steer=%+.1fdeg lag=%.2fs | '
            '장애물 %d개'
            % (self.true, self.radius_err, self.steer_bias, self.servo_lag,
               len(self.obstacles)))
        self.get_logger().warn(
            '이건 코드 버그를 잡는 도구다. AMCL alpha나 radius_scale 같은 물리값은 '
            '여기서 정할 수 없다 - 실차 절차(CALIBRATION_GUIDE.md)로만 나온다.')

    # -- 지도 --------------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._grid = msg
        w, h = msg.info.width, msg.info.height
        arr = np.array(msg.data, dtype=np.int16).reshape(h, w)
        # 미지(-1)도 막힌 것으로 본다. 경기장 밖이 전부 미지다.
        self._occ = (arr >= 50) | (arr < 0)

        if self.obstacles:
            res = msg.info.resolution
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y
            span = int(math.ceil(self.obst_r / res))
            for wx, wy in self.obstacles:
                cx = int((wx - ox) / res)
                cy = int((wy - oy) / res)
                for dy in range(-span, span + 1):
                    for dx in range(-span, span + 1):
                        if dx * dx + dy * dy > span * span:
                            continue
                        if 0 <= cx + dx < w and 0 <= cy + dy < h:
                            self._occ[cy + dy, cx + dx] = True

        self.get_logger().info(
            '지도 수신 %dx%d @%.3fm, 점유 %d셀 (추가 장애물 %d개 반영)'
            % (w, h, msg.info.resolution, int(self._occ.sum()),
               len(self.obstacles)))

    # -- 모터 --------------------------------------------------------------

    def _on_motor(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        self._cmd = (float(msg.data[0]), float(msg.data[1]))
        self._last_cmd_t = self._now()

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        self._amcl = Pose2D(
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- 물리 --------------------------------------------------------------

    def _step(self) -> None:
        angle, speed = self._cmd
        # 명령이 0.5초 넘게 없으면 정지 (cmd_timeout 흉내)
        if self._last_cmd_t is not None and self._now() - self._last_cmd_t > 0.5:
            speed = 0.0

        target = max(-MAX_STEER, min(MAX_STEER, angle + self.steer_bias))
        if self.servo_lag > 0.0:
            k = 1.0 - math.exp(-self._dt / self.servo_lag)
            self._applied_steer += (target - self._applied_steer) * k
        else:
            self._applied_steer = target

        # 실측 데드밴드: speed 2는 안 움직인다
        v = 0.0 if abs(speed) < MOVE_THRESHOLD else (
            speed / SPEED_GAIN) * self.speed_err

        delta = math.radians(self._applied_steer) / max(self.radius_err, 1e-3)
        yaw_rate = v * math.tan(delta) / WHEELBASE

        ds = v * self._dt
        dyaw = yaw_rate * self._dt

        # 참값
        self.true.yaw = wrap_angle(self.true.yaw + dyaw)
        self.true.x += ds * math.cos(self.true.yaw)
        self.true.y += ds * math.sin(self.true.yaw)

        # odom - 드리프트를 섞어 참값과 어긋나게 한다. 안 그러면 AMCL이
        # 할 일이 없어서 검증이 되지 않는다.
        d_odom = ds * self.scale_err
        dyaw_odom = dyaw + self.drift * abs(ds)
        self.odom.yaw = wrap_angle(self.odom.yaw + dyaw_odom)
        self.odom.x += d_odom * math.cos(self.odom.yaw)
        self.odom.y += d_odom * math.sin(self.odom.yaw)

        self._publish_odom(v, yaw_rate)
        self._publish_true()

    # -- 발행 --------------------------------------------------------------

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _publish_odom(self, v: float, yaw_rate: float) -> None:
        st = self._stamp()
        o = Odometry()
        o.header.stamp = st
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = self.odom.x
        o.pose.pose.position.y = self.odom.y
        o.pose.pose.orientation.z = math.sin(self.odom.yaw / 2.0)
        o.pose.pose.orientation.w = math.cos(self.odom.yaw / 2.0)
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = yaw_rate
        self._odom_pub.publish(o)

        t = TransformStamped()
        t.header.stamp = st
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.odom.x
        t.transform.translation.y = self.odom.y
        t.transform.rotation.z = o.pose.pose.orientation.z
        t.transform.rotation.w = o.pose.pose.orientation.w
        self._tf.sendTransform(t)

    def _publish_true(self) -> None:
        ps = PoseStamped()
        ps.header.stamp = self._stamp()
        ps.header.frame_id = 'map'
        ps.pose.position.x = self.true.x
        ps.pose.position.y = self.true.y
        ps.pose.orientation.z = math.sin(self.true.yaw / 2.0)
        ps.pose.orientation.w = math.cos(self.true.yaw / 2.0)
        self._true_pub.publish(ps)

    # -- 광선추적 ----------------------------------------------------------

    def _raycast(self, ox: float, oy: float, angles) -> 'np.ndarray':
        """(ox,oy)에서 angles 방향으로 쏜 거리 배열. 지도 좌표계."""
        g = self._grid
        res = g.info.resolution
        gx0 = g.info.origin.position.x
        gy0 = g.info.origin.position.y
        w, h = g.info.width, g.info.height

        ca = np.cos(angles)
        sa = np.sin(angles)
        r = np.full(angles.shape, self.range_min, dtype=np.float64)
        done = np.zeros(angles.shape, dtype=bool)
        step = res * 0.7

        while True:
            live = ~done & (r < self.range_max)
            if not live.any():
                break
            xs = ox + r * ca
            ys = oy + r * sa
            cx = ((xs - gx0) / res).astype(np.int32)
            cy = ((ys - gy0) / res).astype(np.int32)
            out = (cx < 0) | (cx >= w) | (cy < 0) | (cy >= h)
            cxc = np.clip(cx, 0, w - 1)
            cyc = np.clip(cy, 0, h - 1)
            blocked = out | self._occ[cyc, cxc]
            done |= live & blocked
            r = np.where(live & ~blocked, r + step, r)
        return r

    def _publish_scan(self) -> None:
        if self._occ is None:
            self.get_logger().warn('지도 대기 중 - /scan 미발행',
                                   throttle_duration_sec=5.0)
            return
        # 라이다는 base_link보다 0.41m 앞에 있다
        lx = self.true.x + LIDAR_X * math.cos(self.true.yaw)
        ly = self.true.y + LIDAR_X * math.sin(self.true.yaw)
        rng = self._raycast(lx, ly, self._beam + self.true.yaw)

        if self.noise > 0:
            rng = rng + np.random.normal(0.0, self.noise, rng.shape)
        rng = np.clip(rng, self.range_min, self.range_max)
        # 최대거리 도달 빔은 '측정 없음'으로 표시
        rng[rng >= self.range_max - 1e-6] = float('inf')

        s = LaserScan()
        s.header.stamp = self._stamp()
        s.header.frame_id = 'laser_frame'
        s.angle_min = self.angle_min
        s.angle_max = self.angle_max - self.angle_inc
        s.angle_increment = self.angle_inc
        s.range_min = self.range_min
        s.range_max = self.range_max
        s.ranges = [float(v) for v in rng]
        self._scan_pub.publish(s)

    # -- 초음파 ------------------------------------------------------------

    def _publish_ultra(self) -> None:
        if self._occ is None:
            return
        c, sn = math.cos(self.true.yaw), math.sin(self.true.yaw)
        data = [0] * 8

        def cone(px, py, heading):
            # 광선 3개면 +-15도 원뿔의 최근접을 잡기에 충분하다. 5개는 낭비다.
            a = np.array([heading - 0.26, heading, heading + 0.26])
            return float(self._raycast(px, py, a).min())

        for ch, ly in ((CH_REAR_LEFT, HALFW), (CH_REAR_CENTER, 0.0),
                       (CH_REAR_RIGHT, -HALFW)):
            px = self.true.x + c * REAR - sn * ly
            py = self.true.y + sn * REAR + c * ly
            d = cone(px, py, self.true.yaw + math.pi)
            v = int(round(d * 100))
            data[ch] = 0 if v > ULTRA_CLAMP else v

        for ch, ly, off in ((CH_LEFT, HALFW, math.pi / 2),
                            (CH_RIGHT, -HALFW, -math.pi / 2)):
            px = self.true.x + c * 0.10 - sn * ly
            py = self.true.y + sn * 0.10 + c * ly
            d = cone(px, py, self.true.yaw + off)
            v = int(round(d * 100))
            data[ch] = 0 if v > ULTRA_CLAMP else v

        m = Int32MultiArray()
        m.data = data
        self._ultra_pub.publish(m)

    # -- 리포트 ------------------------------------------------------------

    def _report(self) -> None:
        if self._amcl is None:
            self.get_logger().info(
                '참값 %s | AMCL 아직 없음 (/scan 수신·수렴 대기)' % self.true)
            return
        e = math.hypot(self._amcl.x - self.true.x, self._amcl.y - self.true.y)
        ey = math.degrees(abs(wrap_angle(self._amcl.yaw - self.true.yaw)))
        tag = 'OK' if (e < 0.15 and ey < 10.0) else '주의'
        self.get_logger().info(
            '[%s] 참값 %s | AMCL 오차 %.3fm / %.1fdeg' % (tag, self.true, e, ey))


def main(args=None):
    rclpy.init(args=args)
    node = FakeHardware()
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

