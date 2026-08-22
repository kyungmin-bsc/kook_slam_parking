#!/usr/bin/env python3
"""주차 기동의 지도 충돌 검사와, 벽을 지나지 않는 기동 후보 고르기.

왜 필요한가
-----------
geometry.py의 해석해(solve_parallel / solve_parallel_straight_first)는
**기하학만** 푼다. 시작 자세에서 슬롯 (0,0,0)까지 닿는 원호·직선 조합을 구할
뿐, 그 경로 위에 벽이 있는지는 전혀 모른다. 그리고 여러 해가 나오면
**가장 짧은 것**을 골랐다.

실주행에서 이게 그대로 사고가 됐다. 동측 경로로 A구역 진입점에 도착한 뒤
주차 기동을 풀었더니 최단해가 슬롯 동편 구조물을 **관통**하는 곡선이었고,
차는 그 경로를 그대로 따라가려 했다.

오프라인으로 규모를 재봤다 (진입점 주변 격자 125개, 아래 self-test 그대로).
'벽 관통'은 예전 방식이 고른 해가 실제로 지도와 겹치는 횟수다.

                    현실적 오차          큰 오차
                    ±0.2m/±0.15m/±15도   ±0.4m/±0.3m/±25도
      A 동측 진입    50/125  (40%)       81/125  (65%)
      A 북측 진입     5/125               39/125
      B             11/125               51/125

동측 진입이 특히 나쁘다. 사용자가 실제로 본 것이 이 경우다.
이상적인 진입점에 정확히 섰을 때만 안전했는데, 진입점에 정확히 서는 건
Nav2 허용오차(xy 0.20m)상 애초에 보장되지 않는다.

무엇을 하는가
-------------
1) 기동 경로를 4cm 간격으로 샘플링하고, 각 자세마다 차체 사각형 외곽선을
   3cm 간격으로 찍어 지도 여유(clearance)를 잰다.
2) 해석해를 하나만 쓰지 않고 **후보를 여럿 만든다** - 두 가지 해법
   (직선우선 / 기본순서) x 회전반경 7종.
3) 그중 충돌 검사를 통과한 것만 남기고, 선호 해법 -> 짧은 순 -> 여유 큰 순으로
   고른다. 통과한 후보가 하나도 없으면 None을 돌려주고, 호출한 쪽은
   '진입점으로 정렬한 뒤 다시 푼다'는 2단계로 넘어간다 (plan_align).

같은 격자에서 이 방식의 결과:

    벽을 지나는 해를 고르는 경우 = 0 (설계상 불가능하다)
    현실적 오차에서 해를 못 찾는 경우 = 125개 중 0(A동측) / 5(A북측) / 8(B)
        그 경우는 주차를 시도조차 하지 않고 실패로 보고한다. 벽을 뚫는 것보다
        낫고, 애초에 그 자리에 서면 안 됐다는 뜻이다.

이상적 진입점에서는 예전과 똑같은 해가 나온다 - 후보 순위가 '설정 반경 우선'
이라서, 설정값(A 0.50 / B 0.55)으로 벽을 피할 수 있으면 그것을 그대로 쓴다.
반경을 바꾸는 것은 설정값이 벽에 걸릴 때뿐이다.

주의: 여기서 쓰는 지도는 정적 지도 + 실시간 감지 장애물이다. 그래서 대회
당일 놓인 라바콘도 주차 기동 계획에 그대로 반영된다.
"""

from __future__ import annotations

import math
from typing import Callable, Iterable, List, Optional, Tuple

try:
    from .geometry import (Pose2D, Prim, integrate, sample_path, to_slot_frame,
                           total_length, wrap_angle, solve_parallel,
                           solve_parallel_straight_first, solve_perpendicular,
                           solve_straight_only)
except ImportError:      # python3 parking_mission/collision.py 로 직접 실행할 때
    from geometry import (Pose2D, Prim, integrate, sample_path, to_slot_frame,
                          total_length, wrap_angle, solve_parallel,
                          solve_parallel_straight_first, solve_perpendicular,
                          solve_straight_only)

# nav2_params.yaml의 footprint와 반드시 같아야 한다. base_link = 뒷차축 중심.
FOOTPRINT_FRONT = 0.46
FOOTPRINT_REAR = -0.15
FOOTPRINT_HALF_WIDTH = 0.15

# 차체 외곽선 샘플 간격(m).
#
# 처음에는 차체를 '중심선 위 원판 사슬'로 근사했다. 간단하지만 앞뒤로 반폭
# 0.15m씩 부풀어난다 - 중심선 맨 앞(x=0.46)에 반경 0.15 원판을 놓으면 그 원판이
# x=0.61까지 뻗기 때문이다. 전장 0.61m 차를 0.91m로 보는 셈이라, 차 앞코에
# 6cm 여유가 있는 상황을 '4cm 겹침'으로 오판했다.
#
# 그래서 실제 사각형 외곽선을 따라 점을 찍는다. 그러면 여유값이 곧 '차체
# 표면에서 장애물까지의 실제 거리'가 되어 로그를 그대로 믿을 수 있다.
# 3cm 간격이면 격자 해상도 5cm보다 촘촘해서 셀 사이로 새지 않는다.
OUTLINE_STEP = 0.03

# 경로 샘플 간격. 4cm는 격자 해상도(5cm)보다 촘촘해서 셀을 건너뛰지 않는다.
SWEEP_STEP = 0.04

# 후보 회전반경. 하한은 조향한계에서 나온다 - 축거 0.33m, 최대조향 35도이면
# 최소반경 0.33/tan(35deg) = 0.47m. 여유를 두고 0.50부터 쓴다.
CANDIDATE_RADII: Tuple[float, ...] = (0.50, 0.55, 0.62, 0.70, 0.80, 0.92, 1.05)

# 여유 기준(m). 외곽선 모델이라 이 값은 '차체 표면에서 장애물까지의 실제 거리'다.
# 원판 사슬을 쓰던 때보다 앞뒤로 15cm씩 덜 보수적이므로, 그만큼 기준을 올렸다.
WANT_CLEARANCE = 0.10     # 확보하고 싶은 여유
HARD_CLEARANCE = 0.03     # 절대 하한. 이보다 좁으면 어떤 경우에도 안 쓴다.
                          # clearance는 0 이상만 나오고 0 = '장애물 셀 안'이므로,
                          # 한 셀(5cm)의 절반 이상은 떨어져 있어야 한다는 뜻이다.


class FootprintChecker:
    """PassabilityGrid 위에서 차체 스윕 충돌 검사를 한다."""

    def __init__(self, grid, half_width: float = FOOTPRINT_HALF_WIDTH,
                 front: float = FOOTPRINT_FRONT, rear: float = FOOTPRINT_REAR):
        self.grid = grid
        self.half_width = half_width
        self.front = front
        self.rear = rear
        self._outline_cache: Optional[List[Tuple[float, float]]] = None

    def _local_outline(self) -> List[Tuple[float, float]]:
        """차체 좌표계에서의 외곽선 점들. 자세마다 다시 만들 필요가 없다."""
        if self._outline_cache is None:
            hw, f, r = self.half_width, self.front, self.rear
            pts: List[Tuple[float, float]] = []
            corners = [(f, hw), (f, -hw), (r, -hw), (r, hw)]
            for i in range(4):
                ax, ay = corners[i]
                bx, by = corners[(i + 1) % 4]
                n = max(1, int(math.ceil(math.hypot(bx - ax, by - ay)
                                         / OUTLINE_STEP)))
                for k in range(n):          # 끝점은 다음 변의 시작점이라 뺀다
                    t = k / n
                    pts.append((ax + (bx - ax) * t, ay + (by - ay) * t))
            self._outline_cache = pts
        return self._outline_cache

    def outline(self, p: Pose2D) -> Iterable[Tuple[float, float]]:
        """map 좌표계에서 이 자세의 차체 외곽선 점들."""
        c, s = math.cos(p.yaw), math.sin(p.yaw)
        for a, b in self._local_outline():
            yield (p.x + a * c - b * s, p.y + a * s + b * c)

    def clearance_pose(self, p: Pose2D) -> float:
        """이 자세에서 차체 표면과 장애물 사이 최소 거리(m). 0이면 닿는다."""
        return min(self.grid.clearance_at(x, y) for x, y in self.outline(p))

    def sweep(self, start: Pose2D, prims: List[Prim], radius: float,
              step: float = SWEEP_STEP
              ) -> Tuple[float, Optional[Tuple[float, float]]]:
        """기동 전체의 최소 여유와 그 지점.

        경로를 4cm 간격으로 훑으면서 매 자세의 외곽선을 검사한다. 장애물이
        차체 '안쪽'에 통째로 들어가 있으면 외곽선만으로는 못 잡지만, 연속된
        스윕이라 그 전에 반드시 외곽선을 지나가므로 실질적으로 문제없다.
        """
        worst = float('inf')
        at: Optional[Tuple[float, float]] = None
        for p in sample_path(start, prims, radius, step):
            for (x, y) in self.outline(p):
                c = self.grid.clearance_at(x, y)
                if c < worst:
                    worst, at = c, (x, y)
        if worst == float('inf'):
            return 0.0, None
        return worst, at

    def need_at(self, start: Pose2D) -> float:
        """이 시작 자세에서 요구할 여유.

        시작 자세 자체가 이미 빠듯할 수 있다 - 좁은 통로 안 진입점이 그렇다.
        거기서 WANT_CLEARANCE를 그대로 요구하면 '어떤 기동도 불가'가 되어버린다.
        기동은 지금보다 나빠지지만 않으면 된다.
        """
        here = self.clearance_pose(start)
        return max(HARD_CLEARANCE, min(WANT_CLEARANCE, here - 0.01))


# ---------------------------------------------------------------------------
# 후보 생성 + 선택
# ---------------------------------------------------------------------------

class ManeuverPlan:
    """고른 기동 하나."""

    def __init__(self, prims: List[Prim], radius: float, clearance: float,
                 solver: str, tight_at: Optional[Tuple[float, float]] = None):
        self.prims = prims
        self.radius = radius
        self.clearance = clearance
        self.solver = solver
        self.tight_at = tight_at

    def describe(self) -> str:
        s = '%s R=%.2f 길이 %.2fm' % (self.solver, self.radius,
                                     total_length(self.prims))
        if self.clearance == self.clearance:      # NaN이 아니면
            s += ' 최소여유 %+.3fm' % self.clearance
            if self.tight_at:
                s += ' @%.2f,%.2f' % self.tight_at
        return s


def _perpendicular(start: Pose2D, radius: float) -> Optional[List[Prim]]:
    prims = solve_perpendicular(start, radius)
    if prims is None:
        # 이미 슬롯 축과 나란하면 원호 분해가 특이점이 된다. 직선으로 충분.
        prims = solve_straight_only(start)
    return prims or None


def _solver_order(slot) -> List[Tuple[str, Callable]]:
    """이 슬롯에서 시도할 해법을 선호 순서대로. mission_config의 뜻을 따른다."""
    if slot.kind == 'perpendicular':
        return [('수직', _perpendicular)]
    if slot.straight_first:
        return [('직선우선', solve_parallel_straight_first),
                ('기본순서', solve_parallel)]
    return [('기본순서', solve_parallel),
            ('직선우선', solve_parallel_straight_first)]


def plan_parking(checker: Optional[FootprintChecker], start_map: Pose2D, slot,
                 radii: Iterable[float] = CANDIDATE_RADII
                 ) -> Optional[ManeuverPlan]:
    """start_map에서 slot까지, 벽을 지나지 않는 주차 기동을 고른다.

    checker가 None이면(지도 미수신) 충돌 검사를 건너뛰고 예전처럼 슬롯의 기본
    반경으로 한 번만 푼다. 지도가 없다는 이유로 주차를 포기하지는 않는다.
    """
    slot_start = to_slot_frame(start_map, slot.slot_pose)
    order = _solver_order(slot)

    if checker is None:
        for label, fn in order:
            prims = fn(slot_start, slot.radius)
            if prims:
                return ManeuverPlan(prims, slot.radius, float('nan'),
                                    label + '(무검사)')
        return None

    need = checker.need_at(start_map)
    best: Optional[Tuple[tuple, ManeuverPlan]] = None

    # 설정된 회전반경(slot.radius)을 먼저 본다. 그 값은 mission_config에서
    # 스윕 검증을 거쳐 고른 것이라 아무 이유 없이 벗어나면 안 된다. 여기서
    # 반경을 바꾸는 것은 어디까지나 '설정값으로는 벽을 피할 수 없을 때'의
    # 대안이다. 그래서 설정값에서 먼 반경일수록 순위를 뒤로 민다.
    ranked = sorted(enumerate(radii),
                    key=lambda ir: (abs(ir[1] - slot.radius), ir[0]))

    for rank, (label, fn) in enumerate(order):
        for r_rank, (_, radius) in enumerate(ranked):
            prims = fn(slot_start, radius)
            if not prims:
                continue
            # 해석해가 실제로 목표에 닿는지 먼저 검산한다. 반경을 바꾸면
            # 이분법이 엉뚱한 가지로 수렴하는 경우가 있다.
            end = integrate(slot_start, prims, radius)
            if (math.hypot(end.x, end.y) > 0.02
                    or abs(wrap_angle(end.yaw)) > math.radians(2.0)):
                continue
            worst, at = checker.sweep(start_map, prims, radius)
            if worst < need:
                continue
            # 선호 해법 -> 설정 반경에 가까운 것 -> 짧은 것 -> 여유 큰 것.
            # 길이는 1cm 단위로 뭉개서, 사실상 같은 길이면 여유가 큰 쪽을 고른다.
            key = (rank, r_rank, round(total_length(prims), 2), -worst)
            if best is None or key < best[0]:
                best = (key, ManeuverPlan(prims, radius, worst, label, at))
        if best is not None:
            break      # 선호 해법에서 통과한 후보가 나왔으면 거기서 끝

    return None if best is None else best[1]


def plan_align(checker: Optional[FootprintChecker], start_map: Pose2D,
               staging_map: Pose2D, radii: Iterable[float] = CANDIDATE_RADII
               ) -> Optional[ManeuverPlan]:
    """진입점(staging_map)까지 데려다주는 정렬 기동.

    주차 기동을 어떤 후보로도 못 푸는 경우에 쓴다. 지금 자리에서 슬롯으로
    바로 들어가는 길이 전부 막혔다면, 먼저 '기동이 성립하는 것을 아는 자리'로
    옮긴 뒤 거기서 다시 푸는 게 맞다. 사람이 하는 것과 같다 - 각이 안 나오면
    차를 빼서 다시 댄다.

    슬롯 안이 아니라 개활지에서 도는 기동이므로 원호 한계를 80도까지 연다.
    """
    if checker is None:
        return None
    in_staging = to_slot_frame(start_map, staging_map)
    need = checker.need_at(start_map)
    best: Optional[Tuple[tuple, ManeuverPlan]] = None
    for radius in radii:
        for direction in (1, -1):
            prims = solve_parallel(in_staging, radius,
                                   max_arc=math.radians(80.0),
                                   direction=direction)
            if not prims:
                continue
            end = integrate(in_staging, prims, radius)
            if math.hypot(end.x, end.y) > 0.02:
                continue
            worst, at = checker.sweep(start_map, prims, radius)
            if worst < need:
                continue
            key = (round(total_length(prims), 2), -worst)
            if best is None or key < best[0]:
                best = (key, ManeuverPlan(
                    prims, radius, worst,
                    '정렬%s' % ('전진' if direction > 0 else '후진'), at))
    return None if best is None else best[1]


# ---------------------------------------------------------------------------
# self-test  (python3 parking_mission/collision.py)
# ---------------------------------------------------------------------------
#
# 실측 지도를 그대로 읽어서, 진입점 주변 격자마다
#   (a) 예전 방식 - 해석해를 한 번 풀어 그대로 채택
#   (b) 새 방식   - 후보를 만들어 스윕 검사를 통과한 것만 채택
# 을 비교한다. (a)가 벽을 몇 번 지나는지가 이 모듈이 존재하는 이유다.

if __name__ == '__main__':
    import os
    import sys

    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.dirname(here))
    from parking_mission import mission_config as cfg
    from parking_mission.passability import GridInfo, PassabilityGrid

    def load_map():
        raw = open(os.path.join(os.path.dirname(here), 'parking_map.pgm'),
                   'rb').read()
        toks, i = [], 0
        while len(toks) < 4:
            if raw[i:i + 1] == b'#':
                while raw[i:i + 1] not in (b'\n', b''):
                    i += 1
            elif raw[i:i + 1].isspace():
                i += 1
            else:
                j = i
                while not raw[j:j + 1].isspace():
                    j += 1
                toks.append(raw[i:j])
                i = j
        i += 1
        w, h = int(toks[1]), int(toks[2])
        px = raw[i:]
        info = GridInfo(w, h, 0.05, -2.15, -1.05)
        data = []
        for row in range(h - 1, -1, -1):
            for col in range(w):
                v = px[row * w + col]
                data.append(100 if v == 0 else (0 if v == 254 else -1))
        return PassabilityGrid.from_arrays(info, data)

    ck = FootprintChecker(load_map())

    def legacy(start, slot):
        """예전 방식: 선호 해법 하나, 설정 반경 하나, 충돌 검사 없음."""
        ss = to_slot_frame(start, slot.slot_pose)
        for _, fn in _solver_order(slot):
            prims = fn(ss, slot.radius)
            if prims:
                return prims, slot.radius
        return None, slot.radius

    GRIDS = (
        ('현실적 진입오차 (±0.2m, ±0.15m, ±15deg)',
         (-0.2, -0.1, 0.0, 0.1, 0.2), (-0.15, -0.07, 0.0, 0.07, 0.15),
         (-15, -7, 0, 7, 15)),
        ('큰 진입오차   (±0.4m, ±0.3m, ±25deg)',
         (-0.4, -0.2, 0.0, 0.2, 0.4), (-0.3, -0.15, 0.0, 0.15, 0.3),
         (-25, -12, 0, 12, 25)),
    )
    CASES = (
        ('A/동측진입', cfg.SLOT_A, cfg.A_STAGING_EAST),
        ('A/북측진입', cfg.SLOT_A, cfg.A_STAGING),
        ('B',         cfg.SLOT_B, cfg.B_STAGING),
    )

    fail = 0
    for title, xs, ys, ths in GRIDS:
        print('== %s ==' % title)
        print('   %-11s %-14s %s' % ('', '예전(무검사)', '새 방식(스윕 검사)'))
        for label, slot, base in CASES:
            hit = ok = align_ok = none = 0
            for dx in xs:
                for dy in ys:
                    for dth in ths:
                        st = Pose2D(base.x + dx, base.y + dy,
                                    base.yaw + math.radians(dth))
                        old, R = legacy(st, slot)
                        if old and ck.sweep(st, old, R)[0] <= 0.0:
                            hit += 1
                        plan = plan_parking(ck, st, slot)
                        if plan is not None:
                            ok += 1
                            if plan.clearance < HARD_CLEARANCE:
                                fail += 1     # 있어서는 안 되는 일
                            continue
                        al = plan_align(ck, st, cfg.staging_pose_map(slot))
                        if al and plan_parking(
                                ck, integrate(st, al.prims, al.radius), slot):
                            align_ok += 1
                        else:
                            none += 1
            n = len(xs) * len(ys) * len(ths)
            print('   %-11s 벽 관통 %3d/%d    직접 %3d / 정렬후 %3d / 불가 %3d'
                  % (label, hit, n, ok, align_ok, none))
        print()

    print('이상적 진입점에서 채택되는 기동 (설정 반경을 그대로 써야 정상):')
    for label, slot, base in CASES:
        p = plan_parking(ck, base, slot)
        print('   %-11s %s' % (label, p.describe() if p else '해 없음'))

    print()
    print('FAIL: 하한(%.2fm) 미만인 해가 채택됨 %d건' % (HARD_CLEARANCE, fail)
          if fail else 'ALL PASS (채택된 해 중 벽을 지나는 것은 0건)')
