#!/usr/bin/env python3
"""
대회 공문 좌표 + 슬롯별 기동 파라미터.

여기 있는 값은 성격이 두 가지로 나뉘고, 섞으면 안 된다.

 (1) 공문 확정값 - 주최측이 배포한 값. 임의로 바꾸지 말 것.
 (2) 우리가 정한 튜닝값 - 현장에서 조정해야 하는 값. TODO 표시.

맵: parking_map.pgm 143x149 px, 0.05 m/px, origin (-2.15, -1.05)
    -> map 좌표 범위 x [-2.15, 5.00], y [-1.05, 6.40]
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

from .geometry import Pose2D, staging_parallel, staging_perpendicular

# ---------------------------------------------------------------------------
# (1) 공문 확정값 - 건드리지 말 것
# ---------------------------------------------------------------------------

START_POSE = Pose2D(1.8, 0.9, 3.14)       # 출발 영역 권장 Pose (도면 십자 표기점)
ZONE_A = Pose2D(0.0, 4.2, 0.0)            # 주차영역 A 중심 (T자 후진주차)
ZONE_B = Pose2D(2.1, 3.3, -1.57)          # 주차영역 B 중심 (평행주차)

MAP_RESOLUTION = 0.05
MAP_ORIGIN = (-2.15, -1.05, 0.0)


# ---------------------------------------------------------------------------
# (2) 슬롯별 기동 파라미터 - 현장 조정 대상
# ---------------------------------------------------------------------------

@dataclass
class SlotConfig:
    """주차 슬롯 하나의 기동 설정."""

    name: str
    slot_pose: Pose2D           # map 좌표계의 최종 주차 목표 Pose (공문값)
    kind: str                   # 'perpendicular'(T자 후진) | 'parallel'(평행)
    radius: float               # 기동 설계 회전반경 (m)
    side: int                   # +1 / -1 : 슬롯 기준 어느 쪽에서 진입하는가
    depth: float = 0.25         # perpendicular 전용: 원호 후 곧게 더 후진할 거리
    lateral: float = 0.70       # parallel 전용: 주행 차선 <-> 슬롯 중심 횡거리
    dwell_sec: float = 2.0      # 주차 완료 후 정차 시간 (판정 여유)

    # 기동 후 잔차가 이 이내면 완료로 본다. 넘으면 solve_correction으로 다듬는다.
    pos_tol: float = 0.05       # 종/횡 위치 허용오차 (m)
    yaw_tol: float = math.radians(5.0)
    max_corrections: int = 2    # 보정 반복 상한. 벽에 닿기 전에 멈추게 하는 안전장치

    def staging_slot_frame(self) -> Pose2D:
        """슬롯 좌표계에서의 이상적 진입 pose."""
        if self.kind == 'perpendicular':
            return staging_perpendicular(self.radius, self.depth, self.side)
        if self.kind == 'parallel':
            return staging_parallel(self.radius, self.lateral, self.side)
        raise ValueError('알 수 없는 주차 유형: %s' % self.kind)


# 실측 맵에 기동 경로를 4cm 간격으로 스윕해서 충돌 검사한 결과를 근거로 잡은 값.
#
#   A구역 R=0.50 -> 최소 여유 0.20 m   (R=0.55면 0.15 m로 줄어듦)
#   B구역 R=0.55 -> 최소 여유 0.25 m   (R=0.50이면 0.20 m)
#
# A의 병목은 주차칸이 아니라 **진입 자세에서 차 앞코와 북쪽 벽(y=5.35)의 간격**이다.
# 회전반경이 클수록 슬롯보다 더 북쪽으로 올라가야 해서 여유가 줄어든다.
# 그래서 A는 작게, B는 크게 잡았다.
#
# TODO(현장): 위 여유값은 전부 "AMCL 오차 0" 가정이다. 실제로는 위치추정 오차만큼
#             깎이므로, 대회 전 실차로 반드시 재검증할 것.
SLOT_A = SlotConfig(
    name='A',
    slot_pose=ZONE_A,
    kind='perpendicular',
    radius=0.50,
    side=+1,          # 슬롯을 지나쳐 북쪽으로 올라간 뒤 좌조향 후진으로 진입
    depth=0.25,
)

# TODO(현장): lateral=0.70은 통로 폭(x 1.50~3.30)에서 역산한 가정값이다.
#             실제 주행 차선 위치가 정해지면 반드시 바꿀 것.
SLOT_B = SlotConfig(
    name='B',
    slot_pose=ZONE_B,
    kind='parallel',
    radius=0.55,
    side=+1,
    lateral=0.70,
)

SLOTS: List[SlotConfig] = [SLOT_A, SLOT_B]


# ---------------------------------------------------------------------------
# 파생값
# ---------------------------------------------------------------------------

def staging_pose_map(slot: SlotConfig) -> Pose2D:
    """슬롯의 이상적 진입 pose를 map 좌표계로 변환. Nav2 목표로 쓴다.

    실제 주차 기동은 여기 도착한 '실제 위치'에서 다시 풀기 때문에, 이 값은
    어디까지나 Nav2에게 줄 목표일 뿐 기동 계산의 전제가 아니다.
    """
    from .geometry import from_slot_frame
    return from_slot_frame(slot.staging_slot_frame(), slot.slot_pose)


# ---------------------------------------------------------------------------
# (3) 이동 경로 - 경유점 고정 + 우선순위 폴백
# ---------------------------------------------------------------------------
#
# 왜 경로를 고정하나: 북행 통로가 세 개뿐이고 폭이 0.85~1.40m라, 목표만 던지고
# Hybrid A*가 알아서 고르게 두면 어느 길로 갈지 매번 달라진다. 대회에서 차가
# 어디로 갈지 예측이 안 되는 것 자체가 리스크다.
#
# 왜 여러 개를 두나: 당일 장애물이 어디 놓일지 모른다. 통로 하나가 막히면
# 다음 우선순위로 넘어간다.
#
# 실측 맵 기준 최악 지점의 '차체 마진'(= 중심선 여유 - 반폭 0.15m):
#   북측통로            -> 0.20m   최소 자유폭 0.70m (y=3.6). 중심선 추종 필요
#   서측 좁은목 x=-0.675 -> 0.10m   빠듯. 최소 자유폭 0.50m (y=3.7)
#   동측통로 x=2.80     -> 0.25m   가장 안전하지만 가장 멀다
#
# 중요: 통로 중심선은 y에 따라 움직인다. 두 점을 직선으로 이으면 벽에 붙는다.
#       북측은 5개 경유점으로 중심을 따라가서 0.10 -> 0.20m로 개선했다.
#
# 순위는 사용자 지시를 따른다: 북측 -> 서측 -> 동측.
# 북측/서측이 A구역(서쪽 상단)으로 곧장 향하는 길이라 우선한다. 동측은 마진이
# 가장 넓지만 경기장을 크게 돌아야 해서 마지막 보루로 둔다.
#
# 서측은 y=3.6 부근에서 반드시 x=-0.675로 붙어야 통과한다. x=-0.30으로 가면
# A 아래 가로벽(x -0.35~0.35, y 3.55~3.70)에 정면으로 막힌다.


@dataclass
class Route:
    """경유점으로 고정한 이동 경로 하나."""

    name: str
    waypoints: List[Pose2D]     # 순서대로 통과. 마지막이 이 구간의 최종 목표
    max_attempts: int = 1       # 이 경로 자체를 몇 번까지 재시도할지
    margin: float = 0.0         # 실측 맵 기준 최악 차체 마진 (m, 참고용)
    note: str = ''


@dataclass
class Leg:
    """한 구간. 여러 경로를 우선순위대로 시도한다."""

    name: str
    routes: List[Route]


def _wp(pts: Sequence[Tuple[float, float]], final: Pose2D) -> List[Pose2D]:
    """경유점 (x,y) 목록 + 최종 pose -> Pose2D 목록.

    경유점의 yaw는 '다음 점을 향하는 방향'으로 자동 계산한다. 통과점일 뿐이라
    방위를 직접 지정할 이유가 없고, 진행 방향과 맞춰두면 Nav2가 불필요한
    선회를 하지 않는다.
    """
    out: List[Pose2D] = []
    seq = list(pts) + [(final.x, final.y)]
    for i, (x, y) in enumerate(pts):
        nx, ny = seq[i + 1]
        out.append(Pose2D(x, y, math.atan2(ny - y, nx - x)))
    out.append(final)
    return out


A_STAGING = staging_pose_map(SLOT_A)     # (0.75, 4.70, +90deg)
B_STAGING = staging_pose_map(SLOT_B)     # (2.80, 2.28, -90deg)


def _approach(staging: Pose2D, back: float = 0.45) -> Tuple[float, float]:
    """진입 pose의 '뒤쪽'(진행방향 반대) 예비 경유점.

    진입 지점에는 반드시 그 지점의 방위 방향으로 다가가야 한다. 옆에서 도달하면
    도착 순간 진행방향과 목표 방위가 90도 어긋나고, Nav2가 좁은 공간에서 억지로
    선회를 짜내야 한다. 시뮬레이션에서 이 경유점 없이 돌렸더니 A 진입 방위가
    180도 뒤집힌 채로 끝났다.
    """
    return (staging.x - back * math.cos(staging.yaw),
            staging.y - back * math.sin(staging.yaw))


A_APPROACH = _approach(A_STAGING)        # (0.75, 4.25) - 남쪽에서 북향으로 진입
B_APPROACH = _approach(B_STAGING)        # (2.80, 2.73) - 북쪽에서 남향으로 진입


# --- 출발 -> A 진입 : 3단계 폴백 -------------------------------------------

LEG_TO_A = Leg('A구역 진입', [
    Route('북측', _wp([(0.90, 1.40), (0.80, 2.10), (0.95, 3.00),
                      (1.00, 3.65), (0.85, 4.20), A_APPROACH], A_STAGING),
          max_attempts=3, margin=0.20,
          note='1순위. ⌐벽과 ㄷ벽 사이를 북상. 통로 중심선이 y에 따라 x=0.73~1.03으로 '
               '움직이므로 경유점을 5개 두어 중심을 따라간다. 직선 두 점으로 이으면 '
               '마진이 0.10m까지 떨어진다'),
    Route('서측', _wp([(1.00, 1.35), (-0.30, 1.60), (-0.55, 2.80),
                      (-0.675, 3.30), (-0.675, 3.95), (-0.20, 4.55),
                      (0.75, 4.85), A_APPROACH], A_STAGING),
          max_attempts=4, margin=0.10,
          note='2순위. y=3.6 좁은목을 반드시 x=-0.675로 통과. '
               'x=-0.30으로 가면 A 아래 가로벽에 정면으로 막힌다. 재시도 넉넉히'),
    Route('동측', _wp([(2.80, 1.30), (2.80, 4.70), (1.60, 4.70),
                      (0.75, 4.85), A_APPROACH], A_STAGING),
          max_attempts=3, margin=0.25,
          note='3순위. 마진은 0.25m로 가장 넓지만 경기장을 크게 돈다. '
               '북측/서측이 모두 막혔을 때의 마지막 보루'),
])


# --- A 진입 -> B 진입 -------------------------------------------------------

LEG_A_TO_B = Leg('B구역 진입', [
    Route('북측-동측', _wp([(1.60, 4.70), (2.80, 4.70), B_APPROACH], B_STAGING),
          max_attempts=2, margin=0.25,
          note='북측 가장자리를 동향 후 동측통로로 남하'),
    Route('북측통로-남측', _wp([(0.85, 4.20), (1.00, 3.65), (0.95, 3.00),
                            (0.80, 2.10), (0.90, 1.40), (2.80, 1.30),
                            B_APPROACH], B_STAGING),
          max_attempts=2, margin=0.20,
          note='북측통로로 남하 후 남측을 동향해서 북상. 북측 가장자리가 막혔을 때. '
               '경유점은 북측 경로를 역순으로 탄다'),
])


# --- B 진입 -> 출발 복귀 ----------------------------------------------------

LEG_B_TO_START = Leg('출발지점 복귀', [
    Route('동측통로', _wp([(2.80, 1.10)], START_POSE),
          max_attempts=2, margin=0.60,
          note='그대로 남하 후 서향. 가장 넓다'),
    Route('북측우회', _wp([(2.80, 4.70), (0.90, 4.70), (0.85, 4.20), (1.00, 3.65),
                        (0.95, 3.00), (0.80, 2.10), (0.90, 1.40)], START_POSE),
          max_attempts=2, margin=0.20,
          note='남쪽이 막혔을 때 북측 가장자리로 돌아 북측통로로 남하. '
               'y=4.70을 x=0.90까지 유지한 뒤 내려가야 한다 - 더 동쪽에서 '
               '대각선으로 내려가면 x1.3~2.4/y1.7~4.3 구조물 모서리를 스친다'),
])


LEGS: List[Leg] = [LEG_TO_A, LEG_A_TO_B, LEG_B_TO_START]


def describe_all() -> str:
    """설정 요약 (노드 기동 시 로그로 한 번 찍어서 눈으로 확인하기 위함)."""
    lines = ['미션 설정:']
    lines.append('  출발/복귀 : %s' % START_POSE)
    for s in SLOTS:
        st = staging_pose_map(s)
        lines.append('  %s구역(%s) 목표=%s' % (s.name, s.kind, s.slot_pose))
        lines.append('           진입=%s  R=%.2f side=%+d' % (st, s.radius, s.side))
    return '\n'.join(lines)


if __name__ == '__main__':
    print(describe_all())
