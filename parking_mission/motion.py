#!/usr/bin/env python3
"""
기동 프리미티브를 실제 자이카 모터 명령으로 실행하는 폐루프 실행기.

geometry.py가 "무엇을 할지"(원호/직선 나열)를 풀고, 이 모듈이 "그걸 실제로
어떻게 굴릴지"를 담당한다. ROS 노드가 아니라 **노드에 얹어 쓰는 클래스**다.
(mission_manager 노드가 소유하고 자기 타이머에서 tick()을 불러준다. 노드를
분리하면 커스텀 action 인터페이스가 필요해지고 순수 파이썬 패키지로는
빌드가 번거로워지므로 이렇게 했다.)

구간 종료 판정
-------------
- 원호: **오도메트리 yaw 변화량**으로 끊는다. 거리로 끊지 않는 이유는 실제
  회전반경이 타이어 슬립/서보 유격 때문에 기구학적 이상값(R = L/tan(delta))보다
  대체로 크기 때문이다. yaw로 끊으면 반경 오차가 방향 정렬에는 영향을 주지 않고
  위치 오차로만 남고, 그 위치 오차는 마지막 직선 구간과 기동 후 보정으로 흡수된다.
- 직선: 오도메트리 위치 변화량(거리)으로 끊는다.

각 구간 사이에는 반드시 정지 + 선조향(pre-steer) 시간을 둔다. 자이카 조향 서보는
명령 즉시 각도에 도달하지 않으므로, 차가 움직이면서 조향이 따라오면 원호 앞부분이
설계보다 완만해진다. 정지 상태에서 먼저 조향을 다 꺾고 출발시키면 이 오차가 사라진다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

from .geometry import Prim, wrap_angle


# 실행기 상태
IDLE = 'IDLE'
PRE_STEER = 'PRE_STEER'
DRIVING = 'DRIVING'
HOLD = 'HOLD'          # 초음파가 후방 장애물을 잡아 정지 대기 중
SETTLE = 'SETTLE'
DONE = 'DONE'
FAILED = 'FAILED'


@dataclass
class MotionConfig:
    """자이카 Y모델 실측 기반 기본값. 모두 ROS 파라미터로 덮어쓸 수 있다."""

    # --- 실차 측정 완료 (track_parking/MEASUREMENTS.md) ---
    wheelbase: float = 0.33          # 뒷차축~앞차축 (m)
    steer_limit_deg: float = 35.0    # 기계적 최대 조향각. 명령 40 이상은 더 안 꺾임
    speed_gain: float = 9.86         # /xycar_motor speed 단위 per (m/s)
    min_move_speed: float = 3.0      # speed 2는 정지, 3부터 실제로 굴러감

    # --- 기동 전용 튜닝값 ---
    turn_radius: float = 0.55        # 기동 설계 반경. 실측 최소 0.471 + 여유
    park_speed: float = 0.30         # 주차 기동 속도 (m/s). 느릴수록 정확
    # 구간마다 이 둘이 고정으로 붙는다. 보정 기동은 이동거리가 짧아
    # 오버헤드 비중이 60~80%까지 올라가므로 필요한 최소로 줄였다.
    # 0.35초면 서보가 35도를 꺾기에 충분하다(실측 지연 시간상수 0.1~0.2초 가정).
    pre_steer_time: float = 0.35     # 출발 전 조향을 미리 꺾고 기다리는 시간 (s)
    settle_time: float = 0.20        # 구간 종료 후 완전 정지 대기 시간 (s)
    segment_timeout: float = 25.0    # 구간별 안전 타임아웃 (s)

    # 반경 보정 계수. 실제 원호가 설계보다 크게 그려지면(덜 꺾이면) 1보다 크게
    # 잡아 조향을 더 세게 준다. 현장 캘리브레이션용.
    radius_scale: float = 1.0

    # 초음파가 후방 장애물을 잡았을 때 이만큼 기다려본다. 사람이 지나가는 등
    # 일시적인 경우가 있으므로 즉시 포기하지 않는다. 넘기면 기동을 중단한다.
    hold_timeout: float = 6.0
    # 감속 시 속도 배율 (초음파가 slow_distance 안쪽을 볼 때)
    slow_factor: float = 0.5


class ManeuverExecutor:
    """Prim 리스트를 순차 실행하는 비차단 상태기계.

    사용법:
        ex = ManeuverExecutor(cfg, publish_fn, clock_fn, log_fn)
        ex.start(prims)
        # 20Hz 타이머에서:
        ex.tick(odom_pose)      # odom_pose: geometry.Pose2D (odom 프레임)
        if ex.state in (DONE, FAILED): ...
    """

    def __init__(self,
                 cfg: MotionConfig,
                 publish_motor: Callable[[float, float], None],
                 now_sec: Callable[[], float],
                 log: Callable[[str], None],
                 safety: Optional[Callable[[int, float], object]] = None):
        """safety: (direction, now) -> GuardVerdict 형태의 콜러블.

        None이면 안전 감시 없이 동작한다(시뮬레이션/기하 테스트용). 실차에서는
        mission_manager가 UltrasonicGuard.check을 넘겨준다.
        """
        self.cfg = cfg
        self._publish = publish_motor
        self._now = now_sec
        self._log = log
        self._safety = safety
        self._hold_since = 0.0
        self._hold_reason = ''

        self.state = IDLE
        self._prims: List[Prim] = []
        self._steer: List[float] = []
        self._idx = 0
        self._t_mark = 0.0

        # 구간 시작 시점의 오도메트리 기준값
        self._seg_start_xy: Optional[Tuple[float, float]] = None
        self._seg_start_yaw = 0.0
        self._yaw_accum = 0.0
        self._last_yaw: Optional[float] = None
        self._dist = 0.0
        self._last_xy: Optional[Tuple[float, float]] = None

    # -- 조향각 산출 -------------------------------------------------------

    def steer_for(self, prim: Prim) -> float:
        if prim.kind == 'S':
            return 0.0
        # 구간이 자기 반경을 들고 있으면 그것을 쓴다. 주행 궤적 되짚기처럼
        # 구간마다 곡률이 다른 기동에 필요하다.
        base = self.cfg.turn_radius if prim.radius is None else prim.radius
        radius = base / max(self.cfg.radius_scale, 1e-3)
        deg = math.degrees(math.atan2(self.cfg.wheelbase, radius))
        deg = min(deg, self.cfg.steer_limit_deg)
        return prim.turn * deg

    def speed_units(self, direction: int) -> float:
        raw = self.cfg.park_speed * self.cfg.speed_gain
        raw = max(raw, self.cfg.min_move_speed)
        return direction * raw

    # -- 제어 --------------------------------------------------------------

    def start(self, prims: List[Prim], tag: str = '') -> None:
        self._prims = [p for p in prims if p.length > 1e-3]
        self._steer = [self.steer_for(p) for p in self._prims]
        self._idx = 0
        self._reset_seg()
        if not self._prims:
            self._log('기동 %s: 실행할 구간이 없음 (이미 목표 도달)' % tag)
            self.state = DONE
            return
        self.state = PRE_STEER
        self._t_mark = self._now()
        self._log('기동 %s 시작: %d개 구간' % (tag, len(self._prims)))
        for i, (p, s) in enumerate(zip(self._prims, self._steer)):
            self._log('  [%d] %s' % (i, describe(p, s)))

    def progress(self) -> float:
        """계획된 전체 이동거리 대비 실제로 굴러간 비율 (0.0~1.0).

        기동이 중단됐을 때 "거의 다 왔는데 멈춘 것"과 "시작하자마자 막힌 것"을
        가르기 위해 필요하다. 둘은 대응이 완전히 다르다.

        - 초반 중단: 진입 자세나 장애물이 문제다. 주차가 안 된 것이다.
        - 막바지 중단: 이미 슬롯 안이다. 그 자리를 주차로 인정하고 남은 오차는
          보정 기동으로 다듬는 편이 낫다. 실제로 후방 정지는 늘 기동의 74~83%
          지점에서 걸린다(ultrasonic.py 표 참고).
        """
        total = sum(p.length for p in self._prims)
        if total <= 1e-6:
            return 1.0
        done = sum(p.length for p in self._prims[:self._idx]) + self._dist
        return max(0.0, min(1.0, done / total))

    def abort(self, reason: str) -> None:
        self.stop_motor()
        self.state = FAILED
        self._log('기동 중단: %s' % reason)

    def stop_motor(self) -> None:
        self._publish(0.0, 0.0)

    def _reset_seg(self) -> None:
        self._seg_start_xy = None
        self._seg_start_yaw = 0.0
        self._yaw_accum = 0.0
        self._last_yaw = None
        self._dist = 0.0
        self._last_xy = None

    # -- 매 tick -----------------------------------------------------------

    def tick(self, odom) -> None:
        """odom: geometry.Pose2D (odom 프레임의 현재 base_link pose)."""
        if self.state in (IDLE, DONE, FAILED):
            return

        now = self._now()
        prim = self._prims[self._idx]
        steer = self._steer[self._idx]

        if self.state == PRE_STEER:
            # 정지 상태로 조향만 먼저 꺾어둔다
            self._publish(steer, 0.0)
            if now - self._t_mark >= self.cfg.pre_steer_time:
                self.state = DRIVING
                self._t_mark = now
                self._reset_seg()
            return

        if self.state == HOLD:
            self._publish(steer, 0.0)
            verdict = self._safety(prim.direction, now) if self._safety else None
            if verdict is None or verdict.safe:
                held = now - self._hold_since
                self._log('  후방 안전 확보 (%.1fs 대기) - 재개' % held)
                self.state = DRIVING
                # 대기 중 멈춰 있었으므로 구간 타임아웃 기준을 밀어준다
                self._t_mark += held
                return
            if now - self._hold_since > self.cfg.hold_timeout:
                self.abort('후방 장애물이 %.0fs간 유지됨 (%s)'
                           % (self.cfg.hold_timeout, self._hold_reason))
            return

        if self.state == SETTLE:
            self._publish(0.0, 0.0)
            if now - self._t_mark >= self.cfg.settle_time:
                self._idx += 1
                if self._idx >= len(self._prims):
                    self.state = DONE
                    self._log('기동 완료')
                else:
                    self.state = PRE_STEER
                    self._t_mark = now
            return

        # --- DRIVING ---
        self._accumulate(odom)

        if now - self._t_mark > self.cfg.segment_timeout:
            self.abort('구간 %d 타임아웃 (%.1fs). 바퀴가 안 굴렀거나 오도메트리 미수신'
                       % (self._idx, self.cfg.segment_timeout))
            return

        # 초음파 후방 감시. 후진 구간에서만 실제로 작동한다.
        slow = False
        if self._safety is not None:
            verdict = self._safety(prim.direction, now)
            if not verdict.safe:
                self._hold_since = now
                self._hold_reason = verdict.describe()
                self._log('  정지: %s' % self._hold_reason)
                self._publish(steer, 0.0)
                self.state = HOLD
                return
            slow = bool(getattr(verdict, 'slow', False))

        if self._segment_finished(prim):
            self._publish(steer, 0.0)
            self._log('  구간 %d 종료: 이동 %.3fm, yaw %.1fdeg (목표 %.3fm / %.1fdeg)'
                      % (self._idx, self._dist, math.degrees(self._yaw_accum),
                         prim.length, math.degrees(prim.d_yaw)))
            self.state = SETTLE
            self._t_mark = now
            return

        speed = self.speed_units(prim.direction)
        if slow:
            reduced = speed * self.cfg.slow_factor
            # 감속하더라도 정지마찰을 넘는 최소 속도는 유지해야 굴러간다
            floor = self.cfg.min_move_speed
            speed = math.copysign(max(abs(reduced), floor), speed)
        self._publish(steer, speed)

    def _accumulate(self, odom) -> None:
        if self._last_xy is None:
            self._last_xy = (odom.x, odom.y)
            self._last_yaw = odom.yaw
            return
        dx = odom.x - self._last_xy[0]
        dy = odom.y - self._last_xy[1]
        self._dist += math.hypot(dx, dy)
        self._last_xy = (odom.x, odom.y)

        # yaw는 wrap 경계를 넘어도 누적이 끊기지 않도록 증분으로 더한다
        self._yaw_accum += wrap_angle(odom.yaw - self._last_yaw)
        self._last_yaw = odom.yaw

    def _segment_finished(self, prim: Prim) -> bool:
        if prim.kind == 'C':
            target = abs(prim.d_yaw)
            # yaw로 끊되, 반경 오차가 커서 yaw가 안 차는 경우를 대비해
            # 설계 호길이의 1.6배를 넘으면 거리로도 끊는다 (안전장치)
            if self._dist > prim.length * 1.6:
                return True
            return abs(self._yaw_accum) >= target
        return self._dist >= prim.length


def describe(prim: Prim, steer_deg: float) -> str:
    way = '전진' if prim.direction > 0 else '후진'
    if prim.kind == 'S':
        return '직선 %s %.3fm (조향 0)' % (way, prim.length)
    side = '좌' if prim.turn > 0 else '우'
    return '원호 %s %s%.1fdeg %.3fm (조향 %+.1fdeg)' % (
        way, side, math.degrees(prim.angle), prim.length, steer_deg)

