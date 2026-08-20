#!/usr/bin/env python3
"""
주차 미션 전체를 순서대로 진행시키는 상태기계 노드.

미션 순서 (대회 공문)
  출발지점 -> A구역 T자 후진주차 -> A 탈출 -> B구역 평행주차 -> B 탈출 -> 출발지점 복귀

구조
----
미션을 Step 객체의 리스트로 표현하고, 20Hz 타이머에서 현재 Step의 update()를
호출한다. Step은 RUNNING / DONE / FAILED 중 하나를 돌려주고, DONE이면 다음
Step으로 넘어간다.

Step 종류:
  NavStep   - Nav2 NavigateToPose 액션으로 목표 pose까지 이동
  ParkStep  - 슬롯 안 정밀 주차 기동 (Nav2 미사용, 직접 모터 제어)
  DwellStep - 정차 대기
  ExitStep  - 직전 ParkStep의 기동을 되짚어 슬롯 탈출

두 좌표계를 쓰는 이유
--------------------
기동을 '시작할 때'의 기준점은 map 프레임(AMCL 보정 pose)에서 잡고,
기동 '도중의 추적'은 odom 프레임(/vesc_odom)으로 한다.

  - map(AMCL): 절대 위치는 맞지만 파티클 재계산 때문에 몇 cm씩 튄다.
               몇 초짜리 짧은 기동 중에는 이 점프가 오히려 오차원이 된다.
  - odom(VESC): 절대 위치는 드리프트하지만, 짧은 구간 안에서는 훨씬 매끄럽고 정확하다.

그래서 ParkStep은 진입 시점에 map 기준으로 '슬롯까지 어떻게 갈지'를 한 번 풀고,
그 뒤 실행은 odom 델타로만 폐루프 제어한다.

'매번 다시 푼다'가 핵심
----------------------
진입 pose는 사전 계산된 시퀀스를 재생하는 게 아니라, NavStep이 끝난 그 순간의
**실제 위치**를 읽어서 거기서부터 슬롯 중심까지 가는 해를 새로 계산한다.
Nav2가 데려다준 자리가 이상적 진입점에서 십수 cm 벗어나 있어도 그 오차가
기동 자체에 흡수된다 (geometry.py self-test에서 12cm/8cm/7deg 검증).
"""

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener

from . import mission_config as cfg
from .geometry import (
    Pose2D,
    from_slot_frame,
    integrate,
    reverse_prims,
    sample_path,
    solve_parallel,
    solve_correction,
    solve_perpendicular,
    solve_straight_only,
    to_slot_frame,
    total_length,
    wrap_angle,
)
from .motion import DONE as EX_DONE
from .motion import FAILED as EX_FAILED
from .motion import ManeuverExecutor, MotionConfig
from . import viz

RUNNING, DONE, FAILED = 'RUNNING', 'DONE', 'FAILED'


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_stamped(pose: Pose2D, frame: str, stamp) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = stamp
    ps.pose.position.x = float(pose.x)
    ps.pose.position.y = float(pose.y)
    ps.pose.orientation.z = math.sin(pose.yaw / 2.0)
    ps.pose.orientation.w = math.cos(pose.yaw / 2.0)
    return ps


# ---------------------------------------------------------------------------
# Step 정의
# ---------------------------------------------------------------------------

class Step:
    name = 'step'

    def enter(self, mgr: 'MissionManager') -> None:
        pass

    def update(self, mgr: 'MissionManager') -> str:
        return DONE


class LegStep(Step):
    """한 이동 구간. 여러 경로를 우선순위대로 시도하고, 각 경로는 경유점을 순서대로 통과.

    실패 처리가 두 겹이다.
      - 경유점 하나가 실패 -> 그 경로의 max_attempts만큼 그 경유점부터 재시도
      - 재시도를 다 써도 안 되면 -> 그 경로를 포기하고 다음 우선순위 경로로

    다음 경로로 넘어갈 때 차를 되돌리는 별도 동작은 하지 않는다. Nav2가 현재
    위치에서 새 경로의 첫 경유점까지 알아서 계획하기 때문이다. 통로 중간에서
    막혀 멈췄더라도 거기서부터 다른 통로 입구로 빠져나가는 경로가 나온다.
    """

    def __init__(self, leg: cfg.Leg):
        self.leg = leg
        self.name = leg.name
        self._ri = 0          # 현재 경로 인덱스
        self._wi = 0          # 현재 경유점 인덱스
        self._tries = 0       # 현재 경로에서 쓴 재시도 횟수
        self._goal_future = None
        self._result_future = None
        self._goal_handle = None

    # -- 진입 --------------------------------------------------------------

    def enter(self, mgr: 'MissionManager') -> None:
        mgr.set_nav_enabled(True)
        self._ri = 0
        self._start_route(mgr)

    def _start_route(self, mgr: 'MissionManager') -> None:
        route = self.leg.routes[self._ri]
        self._wi = 0
        self._tries = 0
        mgr.log('  경로 %d/%d "%s" 시작 (경유 %d점, 실측 마진 %.2fm)'
                % (self._ri + 1, len(self.leg.routes), route.name,
                   len(route.waypoints), route.margin))
        if route.note:
            mgr.log('    %s' % route.note)
        mgr.publish_route(route, self._ri)
        self._send(mgr)

    # -- 목표 전송 ----------------------------------------------------------

    def _send(self, mgr: 'MissionManager') -> None:
        self._goal_future = None
        self._result_future = None
        self._goal_handle = None

        if not mgr.nav_client.wait_for_server(timeout_sec=0.1):
            mgr.log('    navigate_to_pose 액션 서버 대기 중...')
            return

        route = self.leg.routes[self._ri]
        target = route.waypoints[self._wi]
        last = self._wi == len(route.waypoints) - 1
        goal = NavigateToPose.Goal()
        goal.pose = pose_stamped(target, 'map', mgr.get_clock().now().to_msg())
        mgr.log('    경유점 %d/%d%s %s'
                % (self._wi + 1, len(route.waypoints),
                   ' (최종)' if last else '', target))
        self._goal_future = mgr.nav_client.send_goal_async(goal)

    # -- 매 tick -----------------------------------------------------------

    def update(self, mgr: 'MissionManager') -> str:
        # 액션 서버가 아직 없어 못 보낸 상태
        if self._goal_future is None and self._goal_handle is None:
            if mgr.nav_client.wait_for_server(timeout_sec=0.0):
                self._send(mgr)
            return RUNNING

        # 목표 수락 대기
        if self._goal_future is not None:
            if not self._goal_future.done():
                return RUNNING
            self._goal_handle = self._goal_future.result()
            self._goal_future = None
            if self._goal_handle is None or not self._goal_handle.accepted:
                mgr.log('    Nav2가 목표를 거부 (경로 생성 실패 가능성)')
                return self._on_waypoint_failed(mgr)
            self._result_future = self._goal_handle.get_result_async()
            return RUNNING

        # 결과 대기
        if self._result_future is not None:
            if not self._result_future.done():
                return RUNNING
            status = self._result_future.result().status
            self._result_future = None
            if status == GoalStatus.STATUS_SUCCEEDED:
                return self._on_waypoint_reached(mgr)
            mgr.log('    Nav2 실패 (status=%d)' % status)
            return self._on_waypoint_failed(mgr)

        return RUNNING

    # -- 성공/실패 처리 ------------------------------------------------------

    def _on_waypoint_reached(self, mgr: 'MissionManager') -> str:
        route = self.leg.routes[self._ri]
        self._wi += 1
        if self._wi >= len(route.waypoints):
            cur = mgr.map_pose()
            final = route.waypoints[-1]
            if cur is not None:
                err = math.hypot(cur.x - final.x, cur.y - final.y)
                yaw_err = abs(wrap_angle(cur.yaw - final.yaw))
                mgr.log('  "%s" 경로 완료. 최종 %s (목표 대비 %.3fm / %.1fdeg)'
                        % (route.name, cur, err, math.degrees(yaw_err)))
            else:
                mgr.log('  "%s" 경로 완료' % route.name)
            return DONE
        # 다음 경유점으로. 재시도 카운터는 경유점마다 새로 준다.
        self._tries = 0
        self._send(mgr)
        return RUNNING

    def _on_waypoint_failed(self, mgr: 'MissionManager') -> str:
        route = self.leg.routes[self._ri]
        self._tries += 1
        if self._tries < route.max_attempts:
            mgr.log('    재시도 %d/%d' % (self._tries, route.max_attempts))
            self._send(mgr)
            return RUNNING

        mgr.log('  "%s" 경로 포기 (경유점 %d에서 %d회 실패)'
                % (route.name, self._wi + 1, self._tries))
        self._ri += 1
        if self._ri >= len(self.leg.routes):
            mgr.log('  모든 경로 실패. 통로가 전부 막혔거나 위치추정이 어긋남')
            return FAILED
        mgr.log('  다음 우선순위 경로로 전환')
        self._start_route(mgr)
        return RUNNING


class ParkStep(Step):
    """슬롯 안 정밀 주차 기동. Nav2를 끄고 모터를 직접 잡는다."""

    def __init__(self, slot: cfg.SlotConfig):
        self.slot = slot
        self.name = '%s구역 주차' % slot.name

    def enter(self, mgr: 'MissionManager') -> None:
        self._corrections = 0
        # 1) Nav2 구동 차단 - 모터를 두고 싸우지 않도록
        mgr.set_nav_enabled(False)
        mgr.publish_motor(0.0, 0.0)

        # 2) 지금 이 순간의 실제 위치를 map 프레임에서 읽는다
        cur_map = mgr.map_pose()
        if cur_map is None:
            mgr.log('  map->base_link TF를 못 읽음. 주차 불가')
            self._failed = True
            return
        self._failed = False

        # 3) 슬롯 좌표계로 옮겨서 목표를 (0,0,0)으로 만든 뒤 해석적으로 푼다
        start_slot = to_slot_frame(cur_map, self.slot.slot_pose)
        ideal = self.slot.staging_slot_frame()
        mgr.log('  실제 진입 %s' % cur_map)
        mgr.log('  슬롯좌표 %s (이상 %s)' % (start_slot, ideal))

        if self.slot.kind == 'perpendicular':
            prims = solve_perpendicular(start_slot, self.slot.radius)
            if prims is None:
                # 이미 슬롯 축과 나란하면 원호 분해가 특이점이 된다. 직선으로 충분.
                mgr.log('  이미 슬롯 축과 나란함 -> 직선 후진으로 축약')
                prims = solve_straight_only(start_slot)
        else:
            prims = solve_parallel(start_slot, self.slot.radius)

        if prims is None:
            mgr.log('  기동 해를 찾지 못함. 진입 위치가 너무 벗어난 듯')
            self._failed = True
            return

        # 4) 푼 해가 실제로 목표에 닿는지 자체 검산 (풀고 나서 반드시 확인)
        end = integrate(start_slot, prims, self.slot.radius)
        residual = math.hypot(end.x, end.y)
        if residual > 0.02 or abs(wrap_angle(end.yaw)) > math.radians(2.0):
            mgr.log('  해 검산 실패: 잔차 %.3fm / %.1fdeg'
                    % (residual, math.degrees(wrap_angle(end.yaw))))
            self._failed = True
            return
        mgr.log('  해 검산 OK: 잔차 %.4fm, 총 %.2fm' % (residual, total_length(prims)))

        # 5) RViz로 눈으로 볼 수 있게 계획 경로를 발행 (전진=초록/후진=주황)
        mgr.publish_maneuver(cur_map, prims, self.slot.radius, self.slot.slot_pose)

        # 6) 탈출 때 되짚어 쓸 수 있게 보관
        mgr.last_park_prims = prims
        mgr.last_park_radius = self.slot.radius

        mgr.executor_.cfg.turn_radius = self.slot.radius
        mgr.executor_.start(prims, tag=self.slot.name)

    def update(self, mgr: 'MissionManager') -> str:
        if self._failed:
            return FAILED
        st = mgr.executor_.state
        if st == EX_FAILED:
            return FAILED
        if st != EX_DONE:
            return RUNNING

        # 기동이 끝났다. 실제로 얼마나 맞았는지 확인하고, 남으면 한 번 더 다듬는다.
        cur = mgr.map_pose()
        if cur is None:
            mgr.log('  주차 후 위치 확인 불가 (TF 없음). 그대로 진행')
            return DONE

        err = to_slot_frame(cur, self.slot.slot_pose)
        mgr.log('  기동 종료. 종/횡 %.3f/%.3f m, 방위 %.1fdeg'
                % (err.x, err.y, math.degrees(err.yaw)))

        if (abs(err.x) <= self.slot.pos_tol and abs(err.y) <= self.slot.pos_tol
                and abs(err.yaw) <= self.slot.yaw_tol):
            mgr.log('  주차 완료 (허용오차 이내)')
            return DONE

        if self._corrections >= self.slot.max_corrections:
            # 여기서 실패 처리하지 않는다. 조금 삐뚤어진 주차가, 보정을 반복하다
            # 벽에 닿거나 시간을 다 쓰는 것보다 낫다.
            mgr.log('  보정 %d회 소진. 현 상태로 종료' % self._corrections)
            return DONE

        prims = solve_correction(err, self.slot.radius)
        if prims is None:
            mgr.log('  보정 기동 해 없음. 현 상태로 종료')
            return DONE

        self._corrections += 1
        mgr.log('  보정 %d회차 (%.2fm)' % (self._corrections, total_length(prims)))
        mgr.publish_maneuver(cur, prims, self.slot.radius, self.slot.slot_pose)
        mgr.executor_.start(prims, tag='%s보정%d' % (self.slot.name, self._corrections))
        return RUNNING


class ExitStep(Step):
    """직전 주차 기동을 되짚어 슬롯을 빠져나온다."""

    def __init__(self, name: str):
        self.name = name

    def enter(self, mgr: 'MissionManager') -> None:
        mgr.set_nav_enabled(False)
        if not mgr.last_park_prims:
            mgr.log('  되짚을 기동 기록이 없음')
            self._failed = True
            return
        self._failed = False
        prims = reverse_prims(mgr.last_park_prims)
        mgr.executor_.cfg.turn_radius = mgr.last_park_radius
        cur = mgr.map_pose()
        if cur is not None:
            mgr.publish_maneuver(cur, prims, mgr.last_park_radius, None)
        mgr.executor_.start(prims, tag='탈출')

    def update(self, mgr: 'MissionManager') -> str:
        if self._failed:
            return FAILED
        st = mgr.executor_.state
        if st == EX_DONE:
            return DONE
        if st == EX_FAILED:
            return FAILED
        return RUNNING


class DwellStep(Step):
    """제자리 정차. 주차 판정 여유를 준다."""

    def __init__(self, name: str, seconds: float):
        self.name = name
        self.seconds = seconds

    def enter(self, mgr: 'MissionManager') -> None:
        mgr.set_nav_enabled(False)
        mgr.publish_motor(0.0, 0.0)
        self._t0 = mgr.now()

    def update(self, mgr: 'MissionManager') -> str:
        mgr.publish_motor(0.0, 0.0)
        return DONE if (mgr.now() - self._t0) >= self.seconds else RUNNING


# ---------------------------------------------------------------------------
# 노드
# ---------------------------------------------------------------------------

class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('motor_topic', '/xycar_motor')
        self.declare_parameter('odom_topic', '/vesc_odom')
        self.declare_parameter('enable_topic', '/nav_drive_enable')
        self.declare_parameter('autostart', False)
        self.declare_parameter('tick_hz', 20.0)

        # 기동 튜닝값 (motion.MotionConfig 기본값을 파라미터로 덮어쓸 수 있게)
        self.declare_parameter('park_speed', 0.30)
        self.declare_parameter('pre_steer_time', 0.5)
        self.declare_parameter('settle_time', 0.35)
        self.declare_parameter('radius_scale', 1.0)

        motor_topic = self.get_parameter('motor_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        enable_topic = self.get_parameter('enable_topic').value

        self._motor_pub = self.create_publisher(Float32MultiArray, motor_topic, 10)
        self._enable_pub = self.create_publisher(Bool, enable_topic, latched_qos())
        self._path_pub = self.create_publisher(Path, '/parking/planned_path', latched_qos())
        self._marker_pub = self.create_publisher(
            MarkerArray, '/parking/maneuver_markers', latched_qos())
        self._route_pub = self.create_publisher(
            MarkerArray, '/parking/route_markers', latched_qos())
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._odom: Optional[Pose2D] = None
        self.last_park_prims = None
        self.last_park_radius = 0.5
        self._nav_enabled: Optional[bool] = None

        mcfg = MotionConfig(
            park_speed=float(self.get_parameter('park_speed').value),
            pre_steer_time=float(self.get_parameter('pre_steer_time').value),
            settle_time=float(self.get_parameter('settle_time').value),
            radius_scale=float(self.get_parameter('radius_scale').value),
        )
        self.executor_ = ManeuverExecutor(
            mcfg, self.publish_motor, self.now, self.log)

        self._steps: List[Step] = self._build_steps()
        self._idx = -1
        self._started = False
        self._finished = False

        for line in cfg.describe_all().split('\n'):
            self.log(line)
        self.log('스텝 %d개: %s' % (len(self._steps),
                                    ' -> '.join(s.name for s in self._steps)))

        period = 1.0 / float(self.get_parameter('tick_hz').value)
        self.create_timer(period, self._tick)

        if bool(self.get_parameter('autostart').value):
            self.start()
        else:
            self.log('autostart=false. /mission/start 에 아무 Bool이나 보내면 시작합니다.')
            self.create_subscription(Bool, '/mission/start', self._on_start_cmd, 1)

    # -- 미션 구성 ---------------------------------------------------------

    def _build_steps(self) -> List[Step]:
        """미션 순서를 Step 리스트로 조립.

        이동 구간은 LegStep(경유점 고정 + 우선순위 폴백), 주차 구간은 ParkStep.
        mission_config.LEGS와 SLOTS를 짝지어 엮는다.
        """
        steps: List[Step] = []
        for slot, leg in zip(cfg.SLOTS, cfg.LEGS):
            steps.append(LegStep(leg))
            steps.append(ParkStep(slot))
            steps.append(DwellStep('%s구역 정차' % slot.name, slot.dwell_sec))
            steps.append(ExitStep('%s구역 탈출' % slot.name))
        steps.append(LegStep(cfg.LEG_B_TO_START))
        return steps

    # -- 보조 --------------------------------------------------------------

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def log(self, msg: str) -> None:
        self.get_logger().info(msg)

    def publish_motor(self, angle: float, speed: float) -> None:
        m = Float32MultiArray()
        m.data = [float(angle), float(speed)]
        self._motor_pub.publish(m)

    def set_nav_enabled(self, enabled: bool) -> None:
        if self._nav_enabled == enabled:
            return
        self._nav_enabled = enabled
        self._enable_pub.publish(Bool(data=enabled))

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = Pose2D(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quat(msg.pose.pose.orientation),
        )

    def map_pose(self) -> Optional[Pose2D]:
        """map -> base_link 를 TF에서 조회. AMCL 보정이 반영된 절대 위치."""
        try:
            tr = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except Exception as exc:  # TF 미준비/타임아웃 전부 포함
            self.get_logger().warn('map->base_link 조회 실패: %s' % exc,
                                   throttle_duration_sec=2.0)
            return None
        t = tr.transform.translation
        return Pose2D(t.x, t.y, yaw_from_quat(tr.transform.rotation))

    def publish_planned_path(self, poses: List[Pose2D]) -> None:
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [pose_stamped(p, 'map', path.header.stamp) for p in poses]
        self._path_pub.publish(path)

    def publish_route(self, route, index: int) -> None:
        """선택된 이동 경로의 경유점을 RViz에 띄운다.

        어느 경로를 쓰고 있는지 눈으로 바로 알 수 있어야 한다. 폴백이 일어나면
        색이 바뀌므로 "지금 몇 순위 경로로 가고 있는지"가 화면에 드러난다.
        """
        stamp = self.get_clock().now().to_msg()
        self._route_pub.publish(viz.route_markers(route, index, 'map', stamp))

    def publish_maneuver(self, start: Pose2D, prims, radius: float,
                         goal: Optional[Pose2D]) -> None:
        """기동 경로를 색 구분 마커 + Path 두 가지로 발행.

        Path는 Nav2 경로와 같은 형식이라 비교하기 좋고, MarkerArray는
        전진/후진을 색으로 구분해서 보여준다.
        """
        stamp = self.get_clock().now().to_msg()
        self._marker_pub.publish(viz.clear_all('map', stamp))
        arr = viz.maneuver_markers(start, prims, radius, goal, 'map', stamp)
        arr.markers.append(viz.legend_text('map', stamp))
        self._marker_pub.publish(arr)
        self.publish_planned_path(sample_path(start, prims, radius, 0.03))

    # -- 진행 --------------------------------------------------------------

    def _on_start_cmd(self, _msg: Bool) -> None:
        self.start()

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        self.log('=== 미션 시작 ===')
        self._advance()

    def _advance(self) -> None:
        self._idx += 1
        if self._idx >= len(self._steps):
            self._finish(True)
            return
        step = self._steps[self._idx]
        self.log('[%d/%d] %s' % (self._idx + 1, len(self._steps), step.name))
        step.enter(self)

    def _finish(self, ok: bool) -> None:
        self._finished = True
        self.set_nav_enabled(False)
        self.publish_motor(0.0, 0.0)
        self.log('=== 미션 %s ===' % ('완료' if ok else '실패'))

    def _tick(self) -> None:
        if not self._started or self._finished:
            return

        # 기동 실행기는 자기 상태를 스스로 굴린다 (odom 델타 기반 폐루프)
        if self.executor_.state not in ('IDLE', EX_DONE, EX_FAILED):
            if self._odom is None:
                self.executor_.abort('/vesc_odom 미수신 - ros1_bridge 확인 필요')
            else:
                self.executor_.tick(self._odom)

        result = self._steps[self._idx].update(self)
        if result == DONE:
            self._advance()
        elif result == FAILED:
            self.log('스텝 실패: %s' % self._steps[self._idx].name)
            self._finish(False)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_motor(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
