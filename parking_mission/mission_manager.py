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
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener

from . import mission_config as cfg
from .geometry import (
    Pose2D,
    Prim,
    from_slot_frame,
    integrate,
    partial_exit,
    reverse_prims,
    sample_path,
    solve_correction,
    to_slot_frame,
    total_length,
    wrap_angle,
)
from .collision import FootprintChecker, plan_align, plan_parking
from .motion import DONE as EX_DONE
from .motion import FAILED as EX_FAILED
from .motion import ManeuverExecutor, MotionConfig
from . import viz
from .passability import GridInfo, PassabilityGrid, remaining_polyline
from .ultrasonic import GuardConfig, UltrasonicGuard

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

    경로를 포기하는 계기가 세 가지다.

      1. 선제 판정 - 경로를 시작하기 전에, 지도 + 실시간 감지 장애물로 그 경로가
         아직 뚫려 있는지 본다. 막혀 있으면 아예 출발하지 않고 다음 순위로 간다.
      2. 주행 중 감시 - 가는 도중 새 장애물이 나타나면 남은 경로를 다시 판정한다.
         비켜서 지나갈 수 있으면 그대로 두고(Nav2가 알아서 피한다), 통과 폭이
         안 나오면 목표를 취소하고 다음 순위로 전환한다.
      3. Nav2 경로생성/추종 실패 - max_attempts만큼 재시도 후 다음 순위로.

    2번이 핵심이다. 이게 없으면 막힌 통로로 끝까지 밀고 들어갔다가 좁은 데서
    오도가도 못하게 된다. 반대로 장애물만 보이면 무조건 우회하면 지나갈 수 있는
    길을 버리게 되므로, '피해서 갈 수 있는가'를 실제로 계산해서 가른다.

    다음 경로로 넘어갈 때 차를 되돌리는 별도 동작은 하지 않는다. Nav2가 현재
    위치에서 새 경로의 첫 경유점까지 알아서 계획하고, 플래너가 REEDS_SHEPP이라
    필요하면 후진해서 빠져나온다.
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
        self._next_check = 0.0    # 다음 통행 판정 시각
        self._blocked_hits = 0    # 연속 '막힘' 판정 횟수 (스파이크 방지)
        self._cancelling = False
        self._retreating = False  # 다음 경로로 넘어가기 전 후진 중
        self._retreat_done = False

    # -- 진입 --------------------------------------------------------------

    def enter(self, mgr: 'MissionManager') -> None:
        mgr.set_nav_enabled(True)
        self._ri = 0
        self._entered = self._start_route(mgr)

    def _start_route(self, mgr: 'MissionManager') -> str:
        """이 경로를 시작한다. 선제 판정에서 막혀 있으면 다음 순위로 넘긴다."""
        # 반드시 여기서 다시 켠다.
        # 후진(retreat)이나 주차 기동 때 set_nav_enabled(False)로 꺼두는데,
        # 예전에는 enter()에서 한 번만 켜서 후진 뒤에 그대로 꺼진 채 남았다.
        # 그러면 경로를 바꿔 목표를 보내도 cmd_vel_bridge가 모터 명령을 막아
        # 차가 후진만 하고 앞으로 가지 않았다.
        mgr.set_nav_enabled(True)
        while self._ri < len(self.leg.routes):
            route = self.leg.routes[self._ri]
            self._wi = 0
            self._tries = 0
            self._blocked_hits = 0
            self._cancelling = False
            self._retreating = False
            self._retreat_done = False
            self._next_check = mgr.now() + mgr.check_period

            verdict = mgr.check_route(route, 0)
            if verdict is not None and not verdict.passable:
                mgr.log('  경로 "%s" 선제 판정: %s -> 건너뜀'
                        % (route.name, verdict.describe()))
                self._ri += 1
                continue

            mgr.log('  경로 %d/%d "%s" 시작 (경유 %d점, 실측 마진 %.2fm)'
                    % (self._ri + 1, len(self.leg.routes), route.name,
                       len(route.waypoints), route.margin))
            if verdict is not None:
                mgr.log('    통행 판정: %s' % verdict.describe())
            if route.note:
                mgr.log('    %s' % route.note)
            mgr.publish_route(route, self._ri)
            self._send(mgr)
            return RUNNING

        mgr.log('  모든 경로가 막힘. 이 구간을 통과할 길이 없음')
        return FAILED

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
        if getattr(self, '_entered', RUNNING) == FAILED:
            return FAILED

        # 후진해서 빠져나오는 중이면 끝날 때까지 기다린다
        if self._retreating:
            st = mgr.executor_.state
            if st in (EX_DONE, EX_FAILED):
                if st == EX_FAILED:
                    mgr.log('  후진 중단됨 (후방 장애물). 현 위치에서 경로 전환')
                else:
                    mgr.log('  후진 완료. 다음 경로로 전환')
                self._retreating = False
                self._retreat_done = True
                return self._switch_route(mgr, '후진 완료')
            return RUNNING

        # 목표 취소를 요청해둔 상태면 취소가 끝나기를 기다렸다가 경로를 바꾼다
        if self._cancelling:
            if self._result_future is not None and not self._result_future.done():
                return RUNNING
            self._cancelling = False
            self._result_future = None
            self._goal_handle = None
            return self._switch_route(mgr, '통과 불가 판정')

        # 중간 경유점 통과 판정 - 반경 안에 들어오면 목표를 취소하고 다음으로.
        # 마지막 경유점(진입 지점)은 정확히 서야 하므로 제외한다.
        route = self.leg.routes[self._ri]
        if (self._goal_handle is not None and not self._cancelling
                and self._wi < len(route.waypoints) - 1):
            cur = mgr.map_pose()
            if cur is not None:
                tgt = route.waypoints[self._wi]
                if math.hypot(cur.x - tgt.x, cur.y - tgt.y) <= mgr.wp_pass_radius:
                    mgr.log('    경유점 %d 통과 (반경 %.2fm 안)'
                            % (self._wi + 1, mgr.wp_pass_radius))
                    self._goal_handle.cancel_goal_async()
                    self._goal_handle = None
                    self._result_future = None
                    return self._on_waypoint_reached(mgr)

        # 주행 중 통행 감시
        if self._goal_handle is not None and mgr.now() >= self._next_check:
            self._next_check = mgr.now() + mgr.check_period
            verdict = mgr.check_route(route, self._wi)

            # 막힌 지점이 아직 멀면 판단을 미룬다.
            #
            # 라이다가 6m까지 보므로, 저 앞의 장애물 하나로 통행 판정이 바로
            # '불가'가 되어 한참 전에 경로를 포기하는 일이 있었다. 그런데
            # 멀리서 본 것은 위치도 부정확하고, 가까이 가보면 Nav2가 지역적으로
            # 비켜갈 수 있는 경우도 많다.
            # 그래서 막힌 지점이 block_min_distance 안으로 들어왔을 때만
            # 전환을 검토한다. 그 전까지는 계속 전진하며 다시 본다.
            far = False
            if verdict is not None and not verdict.passable and verdict.blocked_at:
                cur = mgr.map_pose()
                if cur is not None:
                    d = math.hypot(cur.x - verdict.blocked_at[0],
                                   cur.y - verdict.blocked_at[1])
                    if d > mgr.block_min_distance:
                        far = True
                        mgr.log('    통행 감시: %.2fm 앞이 막힘 - 아직 머니 '
                                '접근하며 재확인 (기준 %.2fm)'
                                % (d, mgr.block_min_distance),)

            if verdict is not None and not verdict.passable and not far:
                self._blocked_hits += 1
                mgr.log('    통행 감시: %s (%d/%d)'
                        % (verdict.describe(), self._blocked_hits,
                           mgr.blocked_confirm))
                if self._blocked_hits >= mgr.blocked_confirm:
                    mgr.log('  "%s" 경로가 막힌 것으로 확정. 목표 취소 후 전환'
                            % route.name)
                    self._cancelling = True
                    if self._goal_handle is not None:
                        self._goal_handle.cancel_goal_async()
                    return RUNNING
            elif not far:
                if self._blocked_hits and verdict is not None:
                    mgr.log('    통행 감시: 회복 - %s' % verdict.describe())
                self._blocked_hits = 0

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
        self._blocked_hits = 0
        self._send(mgr)
        return RUNNING

    def _on_waypoint_failed(self, mgr: 'MissionManager') -> str:
        route = self.leg.routes[self._ri]
        self._tries += 1
        if self._tries < route.max_attempts:
            mgr.log('    재시도 %d/%d' % (self._tries, route.max_attempts))
            self._send(mgr)
            return RUNNING

        return self._switch_route(
            mgr, '경유점 %d에서 %d회 실패' % (self._wi + 1, self._tries))

    def _switch_route(self, mgr: 'MissionManager', why: str) -> str:
        route = self.leg.routes[self._ri]

        # 좁은 통로 안에서 막혔다면, 그 자리에서 다음 경로 목표를 던져봐야
        # Nav2가 경로를 못 만든다(차를 돌릴 공간이 없다). 왔던 길로 곧게
        # 후진해 넓은 곳까지 빠져나온 뒤에 전환한다.
        if route.retreat_distance > 0.0 and not self._retreat_done:
            mgr.log('  "%s" 경로 포기 (%s) - 먼저 %.2fm 후진해서 빠져나온다'
                    % (route.name, why, route.retreat_distance))
            mgr.set_nav_enabled(False)
            mgr.publish_motor(0.0, 0.0)
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None
            self._result_future = None
            self._goal_future = None
            mgr.executor_.cfg.turn_radius = 0.5
            mgr.executor_.start(
                [Prim('S', route.retreat_distance, 0, -1, 0.0, 'retreat')],
                tag='%s 후퇴' % route.name)
            self._retreating = True
            return RUNNING

        mgr.log('  "%s" 경로 포기 (%s)' % (route.name, why))
        self._ri += 1
        if self._ri >= len(self.leg.routes):
            mgr.log('  모든 경로 실패. 통로가 전부 막혔거나 위치추정이 어긋남')
            return FAILED
        return self._start_route(mgr)


class ParkStep(Step):
    """슬롯 안 정밀 주차 기동. Nav2를 끄고 모터를 직접 잡는다."""

    def __init__(self, slot: cfg.SlotConfig):
        self.slot = slot
        self.name = '%s구역 주차' % slot.name

    def enter(self, mgr: 'MissionManager') -> None:
        self._corrections = 0
        self._aligned = False        # 정렬 기동을 이미 한 번 썼는가
        self._phase = 'park'
        # 1) Nav2 구동 차단 - 모터를 두고 싸우지 않도록
        mgr.set_nav_enabled(False)
        mgr.publish_motor(0.0, 0.0)
        self._failed = not self._plan_and_start(mgr)

    def _plan_and_start(self, mgr: 'MissionManager') -> bool:
        """지금 위치에서 주차 기동을 골라 실행한다. 성공하면 True.

        예전에는 해석해를 한 번 풀어 그대로 실행했다. 그런데 해석해는 기하학만
        풀 뿐 벽을 모른다 - 실제로 A구역 동측 진입에서 최단해가 구조물을
        관통하는 곡선이었고 차가 그대로 따라갔다.
        이제는 collision.plan_parking이 후보를 여럿 만들어 지도 스윕 검사를
        통과한 것만 고른다. 자세한 배경은 collision.py 머리말에.
        """
        cur_map = mgr.map_pose()
        if cur_map is None:
            mgr.log('  map->base_link TF를 못 읽음. 주차 불가')
            return False

        checker = mgr.footprint_checker()
        if checker is None:
            mgr.log('  지도 미수신 - 충돌 검사 없이 기본 해로 진행')

        start_slot = to_slot_frame(cur_map, self.slot.slot_pose)
        mgr.log('  실제 진입 %s' % cur_map)
        mgr.log('  슬롯좌표 %s (이상 %s)'
                % (start_slot, self.slot.staging_slot_frame()))

        plan = plan_parking(checker, cur_map, self.slot)

        if plan is None:
            # 여기서 바로 실패로 끝내지 않는다. 지금 자리에서 슬롯으로 들어가는
            # 길이 전부 막혔다는 뜻일 뿐, 진입점으로 옮겨 다시 풀면 되는 경우가
            # 많다. 사람이 각이 안 나올 때 차를 빼서 다시 대는 것과 같다.
            if self._aligned:
                mgr.log('  정렬 후에도 벽을 피하는 주차 기동이 없음. 주차 실패')
                return False
            staging = cfg.staging_pose_map(self.slot)
            align = plan_align(checker, cur_map, staging)
            if align is None:
                mgr.log('  벽을 피하는 주차 기동이 없고 진입점 정렬도 불가. 주차 실패')
                return False
            mgr.log('  벽을 피하는 주차 기동 없음 -> 진입점 %s 로 정렬 후 재시도'
                    % staging)
            mgr.log('  정렬 기동: %s' % align.describe())
            self._phase = 'align'
            self._aligned = True
            mgr.publish_maneuver(cur_map, align.prims, align.radius, None)
            mgr.executor_.cfg.turn_radius = align.radius
            mgr.executor_.start(align.prims, tag='%s정렬' % self.slot.name)
            return True

        mgr.log('  주차 기동 채택: %s' % plan.describe())

        # RViz로 눈으로 볼 수 있게 계획 경로를 발행 (전진=초록/후진=주황)
        mgr.publish_maneuver(cur_map, plan.prims, plan.radius,
                             self.slot.slot_pose)

        # 탈출 때 되짚어 쓸 수 있게 보관. 반경도 함께 - 후보 중에서 골랐으므로
        # slot.radius와 다를 수 있고, 다른 반경으로 되짚으면 경로가 어긋난다.
        mgr.last_park_prims = plan.prims
        mgr.last_park_radius = plan.radius

        self._phase = 'park'
        mgr.executor_.cfg.turn_radius = plan.radius
        mgr.executor_.start(plan.prims, tag=self.slot.name)
        return True

    def update(self, mgr: 'MissionManager') -> str:
        if self._failed:
            return FAILED

        if self._phase == 'align':
            st = mgr.executor_.state
            if st not in (EX_DONE, EX_FAILED):
                return RUNNING
            if st == EX_FAILED:
                mgr.log('  정렬 기동이 중단됨 - 그 자리에서 주차를 다시 풀어본다')
            # 정렬이 끝났든 중간에 멈췄든, 지금 위치에서 다시 푸는 게 맞다.
            if not self._plan_and_start(mgr):
                return FAILED
            return RUNNING

        st = mgr.executor_.state
        if st == EX_FAILED:
            # '보정' 기동이 중단된 거라면 이미 슬롯 안에 들어가 있고 다듬는
            # 중이었을 뿐이다. 조금 삐뚤어진 채로 끝내는 게 낫다.
            if self._corrections > 0:
                mgr.log('  보정 %d회차가 중단됨 - 현 위치로 주차 종료'
                        % self._corrections)
                return DONE

            # 본 기동이 중단된 경우. 예전에는 무조건 실패로 처리해 미션을
            # 끝냈는데, 그게 잘못이었다. 후방 초음파가 잡는 시점은 늘 기동의
            # 74~83% 지점, 즉 슬롯에 거의 다 들어가 벽에 가까워지는 것이
            # 정상인 국면이다. 거기서 미션을 통째로 버릴 이유가 없다.
            #
            #   [INFO] 정지: 후방 0.23m (정지선 0.25m)
            #   [INFO] 기동 중단: 후방 장애물이 6s간 유지됨
            #   [INFO] 스텝 실패: B구역 주차
            #   [INFO] === 미션 실패 ===
            #
            # 그래서 얼마나 갔는지로 가른다. 많이 갔으면 그 자리를 주차로 보고
            # 아래 평가/보정 경로로 넘긴다. 초반에 막힌 거라면 진짜 실패다.
            prog = mgr.executor_.progress()
            if prog < mgr.park_min_progress:
                mgr.log('  본 기동이 %.0f%% 지점에서 중단됨 (기준 %.0f%%) - 주차 실패'
                        % (100.0 * prog, 100.0 * mgr.park_min_progress))
                return FAILED
            mgr.log('  본 기동이 %.0f%% 진행 후 중단됨 - 현 위치를 주차로 보고 '
                    '남은 오차는 보정으로 다듬는다' % (100.0 * prog))
            # 아래 평가/보정 경로로 그대로 떨어진다
        elif st != EX_DONE:
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

        radius = mgr.last_park_radius or self.slot.radius
        prims = solve_correction(err, radius)
        if prims is None:
            mgr.log('  보정 기동 해 없음. 현 상태로 종료')
            return DONE

        # 보정도 벽을 지날 수 있다. 슬롯 안이라 폭이 좁고, 잔차를 지우려고
        # 옆으로 미는 2원호가 그대로 벽을 향하는 경우가 있다. 조금 삐뚤어진
        # 채로 끝내는 편이 벽에 닿는 것보다 낫다.
        checker = mgr.footprint_checker()
        if checker is not None:
            worst, at = checker.sweep(cur, prims, radius)
            if worst < 0.0:
                where = ' @%.2f,%.2f' % at if at else ''
                mgr.log('  보정 기동이 벽에 닿음 (여유 %+.3fm%s). 현 상태로 종료'
                        % (worst, where))
                return DONE

        self._corrections += 1
        mgr.log('  보정 %d회차 (%.2fm)' % (self._corrections, total_length(prims)))
        mgr.publish_maneuver(cur, prims, radius, self.slot.slot_pose)
        mgr.executor_.start(prims, tag='%s보정%d' % (self.slot.name, self._corrections))
        return RUNNING


class ExitStep(Step):
    """직전 주차 기동을 되짚어 슬롯을 빠져나온다.

    slot.exit_arc_fraction으로 '어디까지' 되짚을지 정한다. 끝까지 되짚으면
    진입 때의 방위로 돌아가는데, 그게 다음 목적지 반대쪽이면 나오자마자
    크게 선회해야 한다. A구역이 그 경우라 절반만 되짚는다.
    """

    def __init__(self, name: str, fraction: float = 1.0):
        self.name = name
        self.fraction = fraction

    def enter(self, mgr: 'MissionManager') -> None:
        mgr.set_nav_enabled(False)
        if not mgr.last_park_prims:
            mgr.log('  되짚을 기동 기록이 없음')
            self._failed = True
            return
        self._failed = False
        prims = partial_exit(mgr.last_park_prims, self.fraction)
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


class HomeStep(Step):
    """출발지점 정밀 복귀.

    LegStep이 Nav2로 출발지 근처까지 데려다 놓으면, 여기서 남은 오차를 직접
    지운다. 주차와 같은 방식이다 - 도착한 실제 pose를 읽어 출발 pose까지 가는
    기동을 해석적으로 풀고 모터를 직접 잡는다.

    왜 Nav2에 맡기지 않나
    --------------------
    goal checker의 yaw_goal_tolerance가 3.15 rad(사실상 무검사)이기 때문이다.
    그건 경유점 때문에 반드시 그래야 하는 값이다(Ackermann은 제자리 회전이 안
    되므로 방위를 요구하면 목표를 영원히 못 만족한다). 그 설정 그대로 최종
    복귀까지 끝내면 차가 비스듬히 선 채로 미션이 '완료'된다. 실제로 그랬다.

    goal checker를 목표마다 바꾸는 방법도 있지만(BT에서 goal_checker_id 지정),
    이미 검증된 기동 실행기가 있으므로 그쪽을 쓰는 편이 단순하고 확실하다.
    """

    name = '출발지점 정밀 복귀'

    def enter(self, mgr: 'MissionManager') -> None:
        self._corrections = 0
        self._done = False
        # 모터를 직접 잡으므로 Nav2 구동을 끊는다 (주차 기동과 같은 이유)
        mgr.set_nav_enabled(False)
        mgr.publish_motor(0.0, 0.0)
        self._align(mgr, first=True)

    def _align(self, mgr: 'MissionManager', first: bool) -> None:
        """남은 오차를 재고, 남아 있으면 정렬 기동을 하나 시작한다."""
        cur = mgr.map_pose()
        if cur is None:
            mgr.log('  map->base_link TF를 못 읽음. 정렬 생략')
            self._done = True
            return

        # 출발 pose를 원점으로 하는 좌표계로 옮기면 목표가 (0,0,0)이 된다.
        # 주차에서 슬롯 좌표계를 쓰는 것과 같은 수법이다.
        err = to_slot_frame(cur, cfg.START_POSE)
        mgr.log('  현재 %s' % cur)
        mgr.log('  출발지 대비 종/횡 %.3f/%.3f m, 방위 %.1fdeg'
                % (err.x, err.y, math.degrees(err.yaw)))

        if (abs(err.x) <= cfg.HOME_POS_TOL and abs(err.y) <= cfg.HOME_POS_TOL
                and abs(err.yaw) <= cfg.HOME_YAW_TOL):
            mgr.log('  복귀 완료 (허용오차 이내)')
            self._done = True
            return

        if not first and self._corrections >= cfg.HOME_MAX_CORRECTIONS:
            # 여기서 실패 처리하지 않는다. 미션은 이미 끝났고, 조금 어긋난 복귀가
            # 보정을 반복하다 시간을 다 쓰는 것보다 낫다. (주차와 같은 판단)
            mgr.log('  정렬 %d회 소진. 현 상태로 종료' % self._corrections)
            self._done = True
            return

        # 좁은 한계부터 시도하고, 해가 없을 때만 넓힌다. 작은 잔차를 큰 기동으로
        # 지우면 기동 자체의 실행 오차가 새 오차를 만든다(mission_config 주석 참고).
        prims = None
        for max_arc, max_length in cfg.HOME_SOLVE_STAGES:
            prims = solve_correction(err, cfg.HOME_RADIUS,
                                     max_arc=max_arc, max_length=max_length)
            if prims is not None:
                mgr.log('  해 탐색: 원호한계 %.0f도 / 거리한계 %.1fm 단계에서 성공'
                        % (math.degrees(max_arc), max_length))
                break
        if prims is None:
            mgr.log('  정렬 기동 해 없음. 현 상태로 종료')
            self._done = True
            return

        # 푼 해가 실제로 목표에 닿는지 검산 (주차와 같은 절차)
        end = integrate(err, prims, cfg.HOME_RADIUS)
        residual = math.hypot(end.x, end.y)
        if residual > 0.02 or abs(wrap_angle(end.yaw)) > math.radians(2.0):
            mgr.log('  해 검산 실패: 잔차 %.3fm / %.1fdeg. 현 상태로 종료'
                    % (residual, math.degrees(wrap_angle(end.yaw))))
            self._done = True
            return

        if not first:
            self._corrections += 1
        mgr.log('  정렬 기동 %d회차 (%.2fm)'
                % (self._corrections, total_length(prims)))
        mgr.publish_maneuver(cur, prims, cfg.HOME_RADIUS, cfg.START_POSE)
        mgr.executor_.cfg.turn_radius = cfg.HOME_RADIUS
        mgr.executor_.start(prims, tag='복귀%d' % self._corrections)

    def update(self, mgr: 'MissionManager') -> str:
        if self._done:
            return DONE
        st = mgr.executor_.state
        if st == EX_FAILED:
            # 초음파가 뒤를 잡아 멈추는 건 정상 동작이다. 복귀가 조금 어긋나는
            # 것이 미션을 실패로 끝내는 것보다 낫다.
            mgr.log('  정렬 기동 중단. 현 위치로 종료')
            return DONE
        if st != EX_DONE:
            return RUNNING
        # 기동이 끝났다. 다시 재서 남아 있으면 한 번 더.
        self._align(mgr, first=False)
        return DONE if self._done else RUNNING


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

        # 통행 감시
        self.declare_parameter('check_period', 1.0)      # 판정 주기 (s)
        self.declare_parameter('blocked_confirm', 2)     # 연속 몇 번 막혀야 확정
        # 막힌 지점이 이 거리 안으로 들어와야 경로 전환을 검토한다.
        # 멀리서 본 장애물로 성급하게 포기하지 않기 위한 것.
        self.declare_parameter('block_min_distance', 1.2)
        self.declare_parameter('pass_margin', 0.07)      # 반폭에 더할 여유 (m)
        # 중심선에서 이만큼까지 벗어나는 것은 '같은 경로'로 본다.
        #
        # 0.75로 뒀더니 통로를 통째로 벗어난 우회로도 '통과 가능'으로 나왔다.
        # 북측 통로 폭이 0.70m(반폭 0.35)인데 0.75를 허용하면 옆 통로까지
        # 탐색 범위에 들어오기 때문이다. 실제로 로그에 '중심선에서 0.69m
        # 비켜감'이 찍히면서, 북측이 막혔는데도 서쪽으로 새는 경로를 통과
        # 가능이라 판정해 2순위로 전환하지 않았다.
        #
        # 0.40이면 통로 안에서 장애물을 비켜갈 여유는 주되, 통로를 벗어나면
        # '이 경로는 막혔다'고 판정한다.
        self.declare_parameter('pass_band', 0.40)
        # 중간 경유점을 이 반경 안으로 지나가면 '통과'로 보고 다음으로 넘어간다.
        # Nav2 목표는 '정확히 그 자리에 서기'라 경유점마다 정차하게 되는데,
        # 경유점은 통과점일 뿐이라 그럴 이유가 없다. 멈췄다 다시 서는 동작이
        # 사라져 훨씬 매끄럽고 빨라진다.
        self.declare_parameter('waypoint_pass_radius', 0.35)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('obstacle_grid_topic', '/obstacles/grid')

        # 초음파 후방 감시
        self.declare_parameter('ultrasonic_topic', '/xycar_ultrasonic')
        self.declare_parameter('use_ultrasonic', True)
        # 근거는 ultrasonic.py GuardConfig 주석의 실측 표 참고.
        self.declare_parameter('ultra_stop_distance', 0.18)
        self.declare_parameter('ultra_slow_distance', 0.30)

        # 본 주차 기동이 중단됐을 때, 이 비율 이상 진행됐으면 '거기까지 주차한
        # 것'으로 보고 남은 오차는 보정 기동으로 처리한다. 그 미만이면 진짜
        # 실패다. 후방 정지는 늘 기동의 74~83%에서 걸리므로 0.60이면 갈린다.
        self.declare_parameter('park_min_progress', 0.60)

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

        self.check_period = float(self.get_parameter('check_period').value)
        self.blocked_confirm = int(self.get_parameter('blocked_confirm').value)
        self.block_min_distance = float(
            self.get_parameter('block_min_distance').value)
        self._pass_margin = float(self.get_parameter('pass_margin').value)
        self._pass_band = float(self.get_parameter('pass_band').value)
        self.wp_pass_radius = float(
            self.get_parameter('waypoint_pass_radius').value)
        self.park_min_progress = float(
            self.get_parameter('park_min_progress').value)

        self._static_map: Optional[OccupancyGrid] = None
        self._obs_grid: Optional[OccupancyGrid] = None
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self._on_map, latched_qos())
        self.create_subscription(
            OccupancyGrid, self.get_parameter('obstacle_grid_topic').value,
            self._on_obs_grid, latched_qos())

        self._use_ultra = bool(self.get_parameter('use_ultrasonic').value)
        self.guard = UltrasonicGuard(GuardConfig(
            stop_distance=float(self.get_parameter('ultra_stop_distance').value),
            slow_distance=float(self.get_parameter('ultra_slow_distance').value),
        ))
        if self._use_ultra:
            self.create_subscription(
                Int32MultiArray, self.get_parameter('ultrasonic_topic').value,
                self._on_ultra, 10)

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
            mcfg, self.publish_motor, self.now, self.log,
            safety=self.ultra_check if self._use_ultra else None)

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
            steps.append(ExitStep('%s구역 탈출' % slot.name,
                                  slot.exit_arc_fraction))
        steps.append(LegStep(cfg.LEG_B_TO_START))
        # Nav2는 xy 0.20m / yaw 무검사로 끝난다. 출발 좌표 그대로 서려면
        # 주차와 같은 방식으로 한 번 더 다듬어야 한다.
        steps.append(HomeStep())
        steps.append(DwellStep('출발지점 정차', cfg.HOME_DWELL_SEC))
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

    def _on_ultra(self, msg: Int32MultiArray) -> None:
        self.guard.feed(list(msg.data), self.now())

    def ultra_check(self, direction: int, now: float):
        """ManeuverExecutor가 매 tick 부르는 안전 판정.

        후진 구간에서만 실제로 작동한다. 전진은 초음파가 앞에 없으므로
        라이다/costmap이 맡는다.
        """
        v = self.guard.check(direction, now)
        warn = self.guard.side_warning()
        if warn:
            self.get_logger().warn('측면 근접: %s' % warn,
                                   throttle_duration_sec=2.0)
        return v

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._static_map = msg
        self.log('정적 지도 수신: %dx%d @ %.3fm'
                 % (msg.info.width, msg.info.height, msg.info.resolution))

    def _on_obs_grid(self, msg: OccupancyGrid) -> None:
        self._obs_grid = msg

    def build_grid(self):
        """정적 지도(/map) + 실시간 감지 장애물(/obstacles/grid)을 겹친 격자.

        통행 판정(check_route)과 주차 기동 충돌 검사(collision.py)가 같은 격자를
        쓴다. 그래야 '통로에서는 피해 갔는데 주차하다 들이받는' 불일치가 없다.
        지도를 아직 못 받았으면 None.

        만드는 비용이 싸지 않다(21k셀 집합 + 거리변환 2패스). 주차 계획은 한 번에
        여러 후보를 스윕하느라 이걸 연속으로 부르므로, 입력이 그대로면 만들어둔
        것을 다시 쓴다. 새 장애물 메시지가 오면 자동으로 다시 만든다.
        """
        if self._static_map is None:
            return None
        # 메시지 객체를 그대로 들고 비교한다(is). id()로 비교하면 옛 메시지가
        # 회수된 자리에 새 메시지가 잡혔을 때 같은 값이 나와 오판할 수 있다.
        cached = getattr(self, '_grid_key', None)
        if (cached is not None and cached[0] is self._static_map
                and cached[1] is self._obs_grid):
            return self._grid_cache
        m = self._static_map
        info = GridInfo(m.info.width, m.info.height, m.info.resolution,
                        m.info.origin.position.x, m.info.origin.position.y)
        grid = PassabilityGrid.from_arrays(info, m.data)
        if self._obs_grid is not None and self._obs_grid.info.width == info.width:
            w = info.width
            grid.add_obstacle_cells(
                (i % w, i // w)
                for i, v in enumerate(self._obs_grid.data) if v >= 50)
        self._grid_key = (self._static_map, self._obs_grid)
        self._grid_cache = grid
        return grid

    def footprint_checker(self) -> Optional[FootprintChecker]:
        """주차 기동 스윕 검사기. 지도가 없으면 None (검사를 건너뛴다)."""
        grid = self.build_grid()
        return None if grid is None else FootprintChecker(grid)

    def check_route(self, route, from_index: int):
        """이 경로의 남은 구간이 아직 통과 가능한가.

        지도를 아직 못 받았으면 None을 돌려주고, 호출한 쪽은 판정을 건너뛴다
        (지도가 없다는 이유로 경로를 포기하면 안 된다).
        """
        grid = self.build_grid()
        if grid is None:
            return None
        cur = self.map_pose()
        if cur is None:
            return None

        poly = remaining_polyline(
            (cur.x, cur.y),
            [(wp.x, wp.y) for wp in route.waypoints],
            from_index)
        return grid.route_passable(
            (cur.x, cur.y), poly,
            margin=self._pass_margin, band=self._pass_band)

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

