#!/usr/bin/env python3
"""
주차 기동(maneuver) 기하 계산 모듈.

ROS에 전혀 의존하지 않는다. 순수 파이썬 + math만 사용하므로 노트북에서 그대로
`python3 geometry.py`로 자체 검증을 돌릴 수 있다 (파일 맨 아래 self-test).

핵심 아이디어
------------
주차 기동을 "원호(Curve)와 직선(Straight)의 나열"로 표현하고(=Prim 리스트),
**현재 실제 위치(AMCL)에서 목표 주차 Pose까지** 그 나열을 해석적으로 푼다.

미리 계산해둔 고정 시퀀스를 재생하지 않는 이유: 진입지점까지 Nav2로 이동하면
필연적으로 몇 cm ~ 십여 cm의 오차가 남는데, 고정 시퀀스는 그 오차를 그대로
주차 결과에 전달한다. 반면 여기서는 기동 시작 시점의 실제 pose를 입력으로
매번 새로 풀기 때문에 진입 오차가 기동 자체에 흡수된다.

좌표 규약
--------
- 모든 각도는 라디안. yaw는 +x축 기준 CCW 양수 (ROS REP-103 표준).
- 조향각 부호: **양수 = 좌회전**. 자이카 실측으로 확인된 규약이며 반전 불필요.
- 자전거 모델 기준점(base_link) = 뒷차축 중심.
- turn 부호 s: +1 = 좌회전(회전중심이 차량 왼쪽), -1 = 우회전.
- direction d: +1 = 전진, -1 = 후진.
- 자전거 모델에서 yaw 변화량 = d * s * (호길이 / R). 즉 **후진하며 좌조향하면
  yaw는 감소**한다. T자 후진주차 계산에서 이 부호가 핵심이다.

슬롯 좌표계(slot frame)
----------------------
각 주차구역의 최종 주차 Pose를 원점으로 하고, 그 heading을 +x축으로 삼는 국소
좌표계. 이 좌표계에서 목표는 항상 (0, 0, 0)이 되므로 A구역/B구역을 같은 수식으로
다룰 수 있다. 대회 공문의 Center Pose가 그대로 이 좌표계의 원점이 된다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

EPS = 1e-9


def wrap_angle(a: float) -> float:
    """각도를 (-pi, pi] 범위로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    def __repr__(self) -> str:  # 로그 가독성용
        return 'Pose2D(x=%.3f, y=%.3f, yaw=%.1fdeg)' % (
            self.x, self.y, math.degrees(self.yaw))


# ---------------------------------------------------------------------------
# 좌표계 변환
# ---------------------------------------------------------------------------

def to_slot_frame(pose_map: Pose2D, slot_map: Pose2D) -> Pose2D:
    """map 좌표계의 pose를 슬롯 좌표계(슬롯 중심이 원점, 슬롯 heading이 +x)로 변환."""
    dx = pose_map.x - slot_map.x
    dy = pose_map.y - slot_map.y
    c = math.cos(-slot_map.yaw)
    s = math.sin(-slot_map.yaw)
    return Pose2D(
        x=c * dx - s * dy,
        y=s * dx + c * dy,
        yaw=wrap_angle(pose_map.yaw - slot_map.yaw),
    )


def from_slot_frame(pose_slot: Pose2D, slot_map: Pose2D) -> Pose2D:
    """슬롯 좌표계의 pose를 map 좌표계로 되돌린다."""
    c = math.cos(slot_map.yaw)
    s = math.sin(slot_map.yaw)
    return Pose2D(
        x=slot_map.x + c * pose_slot.x - s * pose_slot.y,
        y=slot_map.y + s * pose_slot.x + c * pose_slot.y,
        yaw=wrap_angle(pose_slot.yaw + slot_map.yaw),
    )


# ---------------------------------------------------------------------------
# 기동 프리미티브
# ---------------------------------------------------------------------------

@dataclass
class Prim:
    """기동 한 구간.

    kind='S' 직선 / kind='C' 원호.
    length: 이동 경로 길이(m), 항상 >= 0. 방향은 direction이 따로 들고 있다.
    turn:   +1 좌, -1 우, 0 직진.
    direction: +1 전진, -1 후진.
    angle:  원호의 회전각 크기(rad, >= 0). 직선이면 0.
    """
    kind: str
    length: float
    turn: int = 0
    direction: int = 1
    angle: float = 0.0
    label: str = ''

    # 이 구간만 다른 회전반경을 쓸 때. None이면 기동 전체의 기본 반경을 쓴다.
    #
    # 해석해(solve_*)는 전부 단일 반경이라 여기를 안 쓴다. 필요한 곳은 딱
    # 하나 - 주행 궤적을 되짚는 후진(mission_manager의 되짚기)이다. 실제로
    # 지나온 길은 구간마다 곡률이 다르므로 구간별 반경이 있어야 한다.
    # label 뒤에 둔 이유: 앞에 끼우면 기존 위치인자 호출이 전부 깨진다.
    radius: Optional[float] = None

    @property
    def d_yaw(self) -> float:
        """이 구간에서 발생하는 yaw 변화량(부호 포함)."""
        if self.kind == 'S':
            return 0.0
        return self.direction * self.turn * self.angle


def _arc_end(pose: Pose2D, radius: float, prim: Prim) -> Pose2D:
    """원호 구간 적분. 회전중심 C = (x - s*R*sin(th), y + s*R*cos(th))."""
    s = prim.turn
    th0 = pose.yaw
    th1 = wrap_angle(th0 + prim.d_yaw)
    cx = pose.x - s * radius * math.sin(th0)
    cy = pose.y + s * radius * math.cos(th0)
    return Pose2D(
        x=cx + s * radius * math.sin(th1),
        y=cy - s * radius * math.cos(th1),
        yaw=th1,
    )


def _straight_end(pose: Pose2D, prim: Prim) -> Pose2D:
    signed = prim.direction * prim.length
    return Pose2D(
        x=pose.x + signed * math.cos(pose.yaw),
        y=pose.y + signed * math.sin(pose.yaw),
        yaw=pose.yaw,
    )


def prim_radius(prim: Prim, default: float) -> float:
    """이 구간에 적용할 회전반경. 구간이 자기 반경을 들고 있으면 그것을 쓴다."""
    return default if prim.radius is None else prim.radius


def integrate(start: Pose2D, prims: List[Prim], radius: float) -> Pose2D:
    """프리미티브 나열을 적분해서 최종 pose를 얻는다. 해(解) 검증용."""
    pose = Pose2D(start.x, start.y, start.yaw)
    for p in prims:
        pose = (_straight_end(pose, p) if p.kind == 'S'
                else _arc_end(pose, prim_radius(p, radius), p))
    return pose


def sample_prim(start: Pose2D, prim: Prim, radius: float,
                step: float = 0.03) -> List[Pose2D]:
    """구간 하나를 촘촘히 샘플링. 시작 pose를 포함한다.

    sample_path()는 전체를 한 줄로 이어붙이지만, 이건 구간 단위로 끊어준다.
    전진/후진 구간에 서로 다른 색을 칠하려면 구간 경계가 살아 있어야 한다.
    """
    poses = [Pose2D(start.x, start.y, start.yaw)]
    n = max(1, int(math.ceil(prim.length / step)))
    for i in range(1, n + 1):
        frac = i / n
        sub = Prim(
            kind=prim.kind,
            length=prim.length * frac,
            turn=prim.turn,
            direction=prim.direction,
            angle=prim.angle * frac,
        )
        poses.append(_arc_end(start, prim_radius(prim, radius), sub)
                     if prim.kind == 'C' else _straight_end(start, sub))
    return poses


def sample_by_prim(start: Pose2D, prims: List[Prim], radius: float,
                   step: float = 0.03) -> List[Tuple[Prim, List[Pose2D]]]:
    """전체 기동을 (구간, 그 구간의 pose들) 쌍의 리스트로 샘플링."""
    out = []
    pose = Pose2D(start.x, start.y, start.yaw)
    for p in prims:
        poses = sample_prim(pose, p, radius, step)
        out.append((p, poses))
        pose = poses[-1]
    return out


def sample_path(start: Pose2D, prims: List[Prim], radius: float,
                step: float = 0.03) -> List[Pose2D]:
    """RViz 시각화용으로 기동 경로를 촘촘히 샘플링한다."""
    poses = [Pose2D(start.x, start.y, start.yaw)]
    pose = poses[0]
    for p in prims:
        n = max(1, int(math.ceil(p.length / step)))
        for i in range(1, n + 1):
            frac = i / n
            sub = Prim(
                kind=p.kind,
                length=p.length * frac,
                turn=p.turn,
                direction=p.direction,
                angle=p.angle * frac,
            )
            poses.append(_arc_end(pose, prim_radius(p, radius), sub)
                         if p.kind == 'C' else _straight_end(pose, sub))
        pose = poses[-1]
    return poses


def total_length(prims: List[Prim]) -> float:
    return sum(p.length for p in prims)


def reverse_prims(prims: List[Prim]) -> List[Prim]:
    """기동을 그대로 되짚어 나오는 탈출 시퀀스.

    구간 순서를 뒤집고 각 구간의 진행방향만 반전한다(조향 방향 turn은 그대로).
    자전거 모델은 시간 대칭이므로 이렇게 하면 정확히 원래 출발 pose로 되돌아온다.
    """
    out = []
    for p in reversed(prims):
        out.append(Prim(
            kind=p.kind,
            length=p.length,
            turn=p.turn,
            direction=-p.direction,
            angle=p.angle,
            label=(p.label + '-exit') if p.label else 'exit',
            radius=p.radius,
        ))
    return out


def partial_exit(prims: List[Prim], fraction: float) -> List[Prim]:
    """탈출 기동을 마지막 원호의 fraction만큼만 수행하도록 자른다.

    탈출의 목적은 '슬롯을 안전하게 빠져나오는 것'이지 '진입지점으로 정확히
    복귀하는 것'이 아니다. 끝까지 되짚으면 진입 때의 방위로 되돌아가는데,
    그 방위가 다음 목적지 방향과 반대일 수 있다. 그 경우 나오자마자 크게
    선회해야 해서 좁은 곳에서 헤맨다.
    """
    out = reverse_prims(prims)
    if fraction >= 1.0 or not out:
        return out
    f = max(0.05, fraction)
    last = out[-1]
    out[-1] = Prim(last.kind, last.length * f, last.turn, last.direction,
                   last.angle * f, last.label, last.radius)
    return out


def trail_to_prims(trail: List[Pose2D], max_length: float = 0.0,
                   curvature_tol: float = 0.25,
                   straight_kappa: float = 0.25) -> List[Prim]:
    """지나온 자취(map pose 나열)를 **되짚어 후진**하는 구간들로 바꾼다.

    왜 필요한가
    -----------
    좁은 통로 안에서 장애물을 만나 경로를 포기할 때, 그 자리에서 다른 통로
    입구로 가는 길을 Nav2가 못 만든다. 차를 돌릴 공간이 없기 때문이다.
    예전에는 '곧게 N미터 후진'으로 때웠는데, 통로가 굽어 있으면 그 직선 후진이
    벽을 향한다. 실제로 지나온 길을 그대로 되짚는 것이 유일하게 항상 안전하다.
    이미 그 길로 왔으니 그 길은 비어 있다.

    어떻게
    ------
    자취의 이웃한 두 pose에서 자전거 모델 원호를 하나씩 역산한다
    (현 d = 두 점 거리, dyaw = 방위 변화 -> R = d/2 / sin(dyaw/2)).
    그런 다음 곡률이 비슷한 것끼리 묶는다. 묶지 않으면 10cm마다 구간이 생기고,
    구간마다 조향 정렬(pre_steer 0.35s)이 들어가서 후진 3m에 10초를 버린다.

    되짚기이므로 순서를 뒤집고 direction=-1로 만든다. turn 부호는 그대로 둔다 -
    자전거 모델은 시간 대칭이라 '핸들을 그대로 두고 후진'하면 왔던 길로 돌아간다.

    max_length > 0 이면 그만큼만 되짚는다(최근 구간부터). 0이면 전부.

    정확도: 자취를 10cm 간격으로 기록하면 되짚기 끝점이 출발점에서 3mm 이내로
    돌아온다(직선+원호+직선 / S자 / 완만한 긴 통로 3종 검증). curvature_tol을
    0.6으로 키우면 구간 수는 3->2로 줄지만 오차가 8cm까지 벌어져서 폭 0.70m
    통로에서는 위험하다. 그래서 기본값을 0.25로 둔다.
    """
    if len(trail) < 2:
        return []

    # 1) 이웃 pose쌍 -> 원호/직선 조각
    segs = []          # (length, kappa(부호), turn)
    for a, b in zip(trail, trail[1:]):
        d = math.hypot(b.x - a.x, b.y - a.y)
        if d < 1e-4:
            continue
        dyaw = wrap_angle(b.yaw - a.yaw)
        if abs(dyaw) < 1e-6:
            segs.append((d, 0.0))
            continue
        r = (d / 2.0) / max(abs(math.sin(dyaw / 2.0)), 1e-9)
        arc_len = r * abs(dyaw)
        segs.append((arc_len, math.copysign(1.0 / r, dyaw)))

    if not segs:
        return []

    # 2) 곡률이 비슷한 조각끼리 묶기
    groups = []        # [길이합, 곡률*길이 합]
    for length, kappa in segs:
        if groups:
            cur_len, cur_kl = groups[-1]
            cur_k = cur_kl / cur_len
            near_straight = abs(cur_k) < straight_kappa and abs(kappa) < straight_kappa
            same_sign = cur_k * kappa > 0 or abs(kappa) < 1e-9 or abs(cur_k) < 1e-9
            close = abs(kappa - cur_k) <= curvature_tol * max(abs(cur_k), 0.5)
            if near_straight or (same_sign and close):
                groups[-1] = [cur_len + length, cur_kl + kappa * length]
                continue
        groups.append([length, kappa * length])

    # 3) 뒤에서부터 꺼내 후진 구간으로. max_length에서 잘린다.
    out: List[Prim] = []
    used = 0.0
    for length, kl in reversed(groups):
        if max_length > 0.0 and used >= max_length - 1e-6:
            break
        # 곡률은 반드시 '자르기 전' 길이로 구한다. 자른 길이로 나누면
        # 남은 조각의 곡률이 부풀어 엉뚱한 조향이 나온다.
        kappa = kl / max(length, 1e-9)
        if max_length > 0.0:
            length = min(length, max_length - used)
        used += length
        if abs(kappa) < 1e-3 or 1.0 / abs(kappa) > 20.0:
            out.append(Prim('S', length, 0, -1, 0.0, 'retrace'))
        else:
            r = 1.0 / abs(kappa)
            out.append(Prim('C', length, 1 if kappa > 0 else -1, -1,
                            length / r, 'retrace', r))
    return [p for p in out if p.length > 0.02]


def prims_to_steer(prims: List[Prim], wheelbase: float, radius: float,
                   steer_limit_deg: float) -> List[Tuple[Prim, float]]:
    """각 구간에 실제로 내보낼 조향각(도)을 붙인다.

    delta = atan(wheelbase / R), 부호는 turn(+1 좌 = 양수)을 따른다.
    """
    delta_deg = math.degrees(math.atan2(wheelbase, radius))
    delta_deg = min(delta_deg, steer_limit_deg)
    return [(p, 0.0 if p.kind == 'S' else p.turn * delta_deg) for p in prims]


# ---------------------------------------------------------------------------
# 진입(staging) Pose - Nav2가 데려다 놓아야 할 지점
# ---------------------------------------------------------------------------

def staging_perpendicular(radius: float, depth: float, side: int) -> Pose2D:
    """T자 후진주차의 이상적 진입 pose (슬롯 좌표계).

    side=+1: 슬롯의 +y쪽 통로에서 접근(차량은 -y 방향을 보고 정차 -> 아니라
             차량 heading은 +pi/2, 즉 통로를 따라 +y로 진행해 온 상태).
    side=-1: 그 거울상.

    depth: 원호가 끝난 뒤 슬롯 안쪽으로 곧게 더 후진할 거리(m).
    """
    return Pose2D(
        x=depth + radius,
        y=side * radius,
        yaw=side * math.pi / 2.0,
    )


def staging_parallel(radius: float, lateral: float, side: int) -> Pose2D:
    """평행주차의 이상적 진입 pose (슬롯 좌표계).

    lateral: 주행 차선 중심과 주차 슬롯 중심 사이의 횡방향 거리(m, > 0).
    side=+1이면 차선이 슬롯의 +y쪽에 있다는 뜻.
    2*radius*(1-cos a) = lateral 을 만족하는 대칭 2원호 해에서 유도.
    """
    ratio = lateral / (2.0 * radius)
    if ratio > 1.0:
        # 횡방향 이동량이 회전반경 대비 과도 -> 2원호로 도달 불가
        raise ValueError(
            'lateral=%.3f m는 radius=%.3f m로 2원호 평행주차 불가 (최대 %.3f m)'
            % (lateral, radius, 2.0 * radius))
    alpha = math.acos(1.0 - ratio)
    return Pose2D(
        x=2.0 * radius * math.sin(alpha),
        y=side * lateral,
        yaw=0.0,
    )


# ---------------------------------------------------------------------------
# 해석적 solver - 실제 현재 pose -> 슬롯 원점
# ---------------------------------------------------------------------------

def solve_perpendicular(start: Pose2D, radius: float,
                        prefer_reverse: bool = True) -> Optional[List[Prim]]:
    """T자(수직) 주차: 직선 -> 원호 -> 직선 (S-C-S) 해석해.

    슬롯 좌표계에서 start -> (0, 0, 0).
    원호가 heading 전체(-start.yaw)를 담당하므로 미지수는 앞/뒤 직선 길이 2개,
    방정식은 최종 x, y 2개 -> 유일해(닫힌 형태).

    start.yaw가 0에 가까우면(=이미 슬롯 축과 나란하면) 이 분해는 특이점이므로
    None을 돌려준다. 그 경우는 그냥 직선 후진이면 충분하다.
    """
    th0 = wrap_angle(start.yaw)
    if abs(math.sin(th0)) < 0.15:  # 슬롯 축과 8.6도 이내로 나란함
        return None

    best: Optional[List[Prim]] = None
    best_cost = float('inf')

    for s in (1, -1):
        # 원호 구간의 변위: P2 = P1 + s*R*(-sin th0, cos th0 - 1)
        dx_arc = s * radius * (-math.sin(th0))
        dy_arc = s * radius * (math.cos(th0) - 1.0)

        # y 방정식으로 앞 직선 a를 먼저 구하고, x 방정식으로 뒤 직선 b를 구한다.
        a = -(start.y + dy_arc) / math.sin(th0)
        b = -(start.x + a * math.cos(th0) + dx_arc)

        # 원호의 진행방향: sign(d_yaw) = d * s 이고 d_yaw = -th0 이므로
        d_arc = 1 if (-th0) * s > 0 else -1
        if prefer_reverse and d_arc != -1:
            continue  # 전진하며 슬롯에 들어가는 해는 T자 '후진'주차가 아니다

        prims: List[Prim] = []
        if abs(a) > 1e-3:
            prims.append(Prim('S', abs(a), 0, 1 if a > 0 else -1, 0.0, 'approach'))
        prims.append(Prim('C', radius * abs(th0), s, d_arc, abs(th0), 'turn-in'))
        if abs(b) > 1e-3:
            prims.append(Prim('S', abs(b), 0, 1 if b > 0 else -1, 0.0, 'settle'))

        # 비용: 총 이동거리 + 전진/후진 전환 페널티(전환은 시간과 오차를 모두 늘린다)
        switches = sum(1 for i in range(1, len(prims))
                       if prims[i].direction != prims[i - 1].direction)
        cost = total_length(prims) + 0.5 * switches
        if cost < best_cost:
            best_cost = cost
            best = prims

    return best


def solve_parallel(start: Pose2D, radius: float,
                   max_arc: float = math.pi * 0.75,
                   tol: float = 1e-6,
                   direction: int = -1) -> Optional[List[Prim]]:
    """평행주차: 원호 -> 반대방향 원호 -> 직선 해.

    슬롯 좌표계에서 start -> (0, 0, 0). 두 원호는 조향을 서로 반대로 꺾으며
    같은 방향(direction)으로 움직인다. direction=-1이 후진 진입(표준 평행주차),
    +1은 전진하며 옆으로 밀어넣는 형태로 기동 후 미세보정에 쓴다.

    미지수 (phi1, phi2, b) 3개, 방정식 (x, y, yaw) 3개지만 yaw 방정식이
    phi1 - phi2 = -start.yaw / (direction*s) 로 바로 풀리므로, 남은 자유도
    1개(t = phi2)를 y 방정식의 1차원 이분법으로 찾고 b는 x 방정식에서 바로 얻는다.
    scipy 없이도 되고, y가 t에 대해 단조라 이분법이 안정적으로 수렴한다.
    """
    th0 = wrap_angle(start.yaw)
    d = 1 if direction > 0 else -1

    best: Optional[List[Prim]] = None
    best_cost = float('inf')

    for s in (1, -1):
        # yaw 방정식: th0 + d*s*(phi1 - phi2) = 0
        offset = -th0 / (d * s)

        def build(t: float) -> Optional[List[Prim]]:
            phi2 = t
            phi1 = t + offset
            if phi1 < -tol or phi2 < -tol:
                return None
            if phi1 > max_arc or phi2 > max_arc:
                return None
            return [
                Prim('C', radius * phi1, s, d, phi1, 'swing-in'),
                Prim('C', radius * phi2, -s, d, phi2, 'straighten'),
            ]

        def y_after_arcs(t: float) -> Optional[float]:
            prims = build(t)
            if prims is None:
                return None
            return integrate(start, prims, radius).y

        # t 탐색 구간: phi1 >= 0 을 만족하는 최소값부터 max_arc까지
        lo = max(0.0, -offset)
        hi = min(max_arc, max_arc - offset) if offset > 0 else max_arc
        if hi <= lo:
            continue

        f_lo = y_after_arcs(lo)
        f_hi = y_after_arcs(hi)
        if f_lo is None or f_hi is None or f_lo * f_hi > 0:
            continue  # 이 조향 부호로는 y=0을 가로지르지 못함

        for _ in range(80):
            mid = 0.5 * (lo + hi)
            f_mid = y_after_arcs(mid)
            if f_mid is None:
                break
            if f_lo * f_mid <= 0:
                hi = mid
                f_hi = f_mid
            else:
                lo = mid
                f_lo = f_mid
        t = 0.5 * (lo + hi)

        prims = build(t)
        if prims is None:
            continue
        after = integrate(start, prims, radius)
        if abs(after.y) > 1e-3 or abs(wrap_angle(after.yaw)) > 1e-3:
            continue

        b = -after.x
        if abs(b) > 1e-3:
            prims.append(Prim('S', abs(b), 0, 1 if b > 0 else -1, 0.0, 'center'))

        prims = [p for p in prims if p.length > 1e-3]
        switches = sum(1 for i in range(1, len(prims))
                       if prims[i].direction != prims[i - 1].direction)
        cost = total_length(prims) + 0.5 * switches
        if cost < best_cost:
            best_cost = cost
            best = prims

    return best


def solve_parallel_straight_first(start: Pose2D, radius: float,
                                  max_arc: float = math.pi * 0.75,
                                  direction: int = -1) -> Optional[List[Prim]]:
    """평행주차인데 직선을 **앞에** 두는 변형: 직선 -> 원호 -> 반대 원호.

    solve_parallel은 [원호, 원호, 직선] 순서라 차가 먼저 옆으로 붙은 뒤 슬롯 축을
    따라 길게 들어온다. 슬롯 축 위에 장애물이 있으면 그 구간에서 걸린다.

    A구역이 그 경우다. (1.30, 4.10~4.30)에 얇은 벽이 있어서 슬롯 축(y=4.2)을
    따라 동쪽에서 들어오면 차 앞코가 스친다. 직선을 앞에 두면 차가 진입 높이
    (y=4.65)를 유지한 채 그 벽을 지나친 뒤에 내려오므로 걸리지 않는다.

    미지수는 (직선길이 a, 원호각 t) 둘이고 방정식은 최종 x, y 둘이다.
    각 t마다 '원호가 원점에서 끝나려면 원호 시작점이 어디여야 하는가'를 구하고,
    그 점이 시작 자세의 진행선 위에 있는지를 외적으로 본다. 그 외적이 0이 되는
    t를 이분법으로 찾고, a는 내적으로 얻는다.
    """
    th0 = wrap_angle(start.yaw)
    d = 1 if direction > 0 else -1
    ux, uy = math.cos(th0), math.sin(th0)

    best: Optional[List[Prim]] = None
    best_cost = float('inf')

    for sgn in (1, -1):
        offset = -th0 / (d * sgn)

        def arc_start_for(t):
            """원호 2개가 (0,0,0)에서 끝나도록 하는 원호 시작점과 구간들."""
            phi2 = t
            phi1 = t + offset
            if phi1 < -1e-9 or phi2 < -1e-9 or phi1 > max_arc or phi2 > max_arc:
                return None, None
            prims = [Prim('C', radius * phi1, sgn, d, phi1, 'swing-in'),
                     Prim('C', radius * phi2, -sgn, d, phi2, 'straighten')]
            # 시작 자세를 (0,0,th0)로 두고 적분한 변위를 빼면 시작점이 나온다
            disp = integrate(Pose2D(0.0, 0.0, th0), prims, radius)
            if abs(wrap_angle(disp.yaw)) > 1e-6:
                return None, None
            return (-disp.x, -disp.y), prims

        def cross(t):
            p, _ = arc_start_for(t)
            if p is None:
                return None
            vx, vy = p[0] - start.x, p[1] - start.y
            return vx * uy - vy * ux

        lo = max(0.0, -offset)
        hi = min(max_arc, max_arc - offset) if offset > 0 else max_arc
        if hi <= lo:
            continue
        flo, fhi = cross(lo), cross(hi)
        if flo is None or fhi is None or flo * fhi > 0:
            continue
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            fm = cross(mid)
            if fm is None:
                break
            if flo * fm <= 0:
                hi, fhi = mid, fm
            else:
                lo, flo = mid, fm
        t = 0.5 * (lo + hi)
        p, prims = arc_start_for(t)
        if p is None:
            continue
        a = (p[0] - start.x) * ux + (p[1] - start.y) * uy
        out: List[Prim] = []
        if abs(a) > 1e-3:
            out.append(Prim('S', abs(a), 0, 1 if a > 0 else -1, 0.0, 'run-past'))
        out.extend(prims)

        end = integrate(start, out, radius)
        if math.hypot(end.x, end.y) > 5e-3 or abs(wrap_angle(end.yaw)) > 1e-3:
            continue
        cost = total_length(out)
        if cost < best_cost:
            best_cost, best = cost, out
    return best


def solve_straight_only(start: Pose2D) -> List[Prim]:
    """이미 슬롯 축과 거의 나란할 때 쓰는 축약해: 직선 한 구간."""
    b = -start.x
    if abs(b) < 1e-3:
        return []
    return [Prim('S', abs(b), 0, 1 if b > 0 else -1, 0.0, 'settle')]


def solve_correction(start: Pose2D, radius: float,
                     lat_tol: float = 0.02,
                     max_arc: float = math.radians(40.0),
                     max_length: float = 1.2) -> Optional[List[Prim]]:
    """기동 완료 후 남은 잔차를 지우는 미세보정 기동.

    왜 필요한가: motion.py의 원호 구간은 **yaw 변화량**으로 종료를 판정한다.
    실제 회전반경이 설계값보다 크면(타이어 슬립 등) 방위각은 정확히 맞지만
    위치가 설계보다 바깥으로 밀린다. 이 오차는 계획 시점에 길이가 정해지는
    마지막 직선 구간으로는 흡수할 수 없다. 그래서 기동이 끝난 뒤 실제 위치를
    다시 읽고 남은 오차만큼 짧게 더 움직인다.

    - 횡방향 오차가 작으면: 직선 한 구간으로 끝 (가장 안전)
    - 횡방향 오차가 남았으면: 2원호 S자로 옆으로 민다. 슬롯 안이라 공간이
      좁으므로 max_arc/max_length로 기동 크기를 강하게 제한한다.

    전진/후진 양쪽을 다 시도해서 짧은 쪽을 고른다. 슬롯 깊숙이 들어가 있을 때
    더 후진하면 안쪽 벽에 닿을 수 있으므로 전진 해가 선택될 여지를 남겨둔다.

    보정 기동의 크기는 잔차 크기에 맞춘다.

    "찔끔거리지 말고 확실하게 크게 움직이면 빠르지 않나"를 실제로 시험해봤다
    (원호 최소 18도 강제). 결과는 반대였다 - 보정 횟수는 2~3회에서 0~1회로
    줄었지만 최종 오차가 0.9cm에서 13cm로 나빠졌다. 잔차가 3cm인데 0.8m를
    움직이면 그 기동 자체의 실행 오차(반경 오차 15%)가 새 오차를 만들기
    때문이다. 그래서 최소 원호 강제는 넣지 않는다.
    """
    if abs(start.y) <= lat_tol and abs(wrap_angle(start.yaw)) <= math.radians(3.0):
        prims = solve_straight_only(start)
        return prims if prims else None

    best: Optional[List[Prim]] = None
    best_cost = float('inf')
    for direction in (-1, 1):
        prims = solve_parallel(start, radius, max_arc=max_arc,
                               direction=direction)
        if prims is None:
            continue
        length = total_length(prims)
        if length > max_length:
            continue
        if length < best_cost:
            best_cost = length
            best = prims
    return best


# ---------------------------------------------------------------------------
# self-test
# ---------------------------------------------------------------------------

def _check(name: str, start: Pose2D, prims: Optional[List[Prim]],
           radius: float) -> bool:
    if prims is None:
        print('  [FAIL] %-28s 해 없음' % name)
        return False
    end = integrate(start, prims, radius)
    err = math.hypot(end.x, end.y)
    yaw_err = abs(wrap_angle(end.yaw))
    ok = err < 1e-6 and yaw_err < 1e-6
    print('  [%s] %-28s %s' % ('ok  ' if ok else 'FAIL', name, start))
    for p in prims:
        if p.kind == 'S':
            print('        %-11s 직선 %s %.3fm' % (
                p.label, '전진' if p.direction > 0 else '후진', p.length))
        else:
            print('        %-11s 원호 %s %s %.1fdeg (%.3fm)' % (
                p.label, '전진' if p.direction > 0 else '후진',
                '좌' if p.turn > 0 else '우',
                math.degrees(p.angle), p.length))
    print('        -> 잔차 %.2e m / %.2e rad, 총 %.3fm'
          % (err, yaw_err, total_length(prims)))
    return ok


if __name__ == '__main__':
    R = 0.55
    ok = True
    print('== T자 후진주차 (수직) ==')
    ok &= _check('이상적 진입 pose', staging_perpendicular(R, 0.25, +1),
                 solve_perpendicular(staging_perpendicular(R, 0.25, +1), R), R)
    ok &= _check('거울상 진입', staging_perpendicular(R, 0.25, -1),
                 solve_perpendicular(staging_perpendicular(R, 0.25, -1), R), R)
    noisy = staging_perpendicular(R, 0.25, +1)
    noisy = Pose2D(noisy.x + 0.12, noisy.y - 0.08,
                   noisy.yaw + math.radians(7.0))
    ok &= _check('진입 오차 12/8cm+7deg', noisy, solve_perpendicular(noisy, R), R)

    print('== 평행주차 ==')
    ok &= _check('이상적 진입 pose', staging_parallel(R, 0.5, +1),
                 solve_parallel(staging_parallel(R, 0.5, +1), R), R)
    ok &= _check('거울상 진입', staging_parallel(R, 0.5, -1),
                 solve_parallel(staging_parallel(R, 0.5, -1), R), R)
    noisy = staging_parallel(R, 0.5, +1)
    noisy = Pose2D(noisy.x + 0.10, noisy.y + 0.06,
                   noisy.yaw + math.radians(-6.0))
    ok &= _check('진입 오차 10/6cm-6deg', noisy, solve_parallel(noisy, R), R)

    print('== 탈출 시퀀스 되짚기 ==')
    st = staging_perpendicular(R, 0.25, +1)
    fwd = solve_perpendicular(st, R)
    back = integrate(Pose2D(0, 0, 0), reverse_prims(fwd), R)
    err = math.hypot(back.x - st.x, back.y - st.y)
    print('  [%s] 탈출 후 진입지점 복귀 잔차 %.2e m' % ('ok  ' if err < 1e-6 else 'FAIL', err))
    ok &= err < 1e-6

    print()
    print('ALL PASS' if ok else 'SOME FAILED')

