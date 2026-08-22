#!/usr/bin/env python3
"""
"이 장애물을 끼고도 이 경로를 지나갈 수 있는가" 판정.

ROS에 의존하지 않는다. 격자와 좌표만 다루므로 노트북에서 그대로 테스트할 수 있다
(파일 맨 아래 self-test).

왜 필요한가
----------
1순위 경로로 가다가 지도에 없던 장애물을 만났을 때, 두 경우를 갈라야 한다.

  (a) 통로가 아직 충분히 넓다  -> 옆으로 비켜서 그 경로를 계속 간다 (Nav2가 알아서 함)
  (b) 통로가 막혔다            -> 이 경로를 포기하고 다음 우선순위로 전환한다

이 둘을 구분하지 못하면 막힌 길로 끝까지 밀고 들어갔다가 좁은 통로 안에서
오도가도 못하게 된다. 반대로 장애물만 보이면 무조건 우회하면, 지나갈 수 있는
길을 버리고 더 위험한 경로로 가게 된다.

판정 방법
--------
정적 지도와 실시간 감지 장애물을 합친 격자에서 각 셀의 '여유'(가장 가까운
장애물까지 거리)를 구하고, 여유가 차량 반폭+안전마진 이상인 셀만 통행 가능으로
본다. 그 다음 현재 위치에서 구간 목표까지 BFS로 연결되는지 본다.

경로 주변 band 안으로만 탐색을 제한하는 게 핵심이다. 제한하지 않으면 "경기장
어딘가로는 갈 수 있다"는 답이 나와서, 지금 타고 있는 경로가 살아있는지를
판정하는 목적에 맞지 않는다.
"""

from __future__ import annotations

import math
import heapq
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Set, Tuple

# 자이카 실측: 차폭 0.30m (반폭 0.15). 여기에 안전마진을 더한 값이 통행 기준.
CAR_HALF_WIDTH = 0.15
DEFAULT_MARGIN = 0.07          # 반폭에 더할 여유 -> 최소 통과폭 0.44m
DEFAULT_BAND = 0.75            # 경로 중심선에서 이만큼까지는 비켜가도 같은 경로로 본다

_SQRT2 = math.sqrt(2.0)


@dataclass
class GridInfo:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (int((x - self.origin_x) / self.resolution),
                int((y - self.origin_y) / self.resolution))

    def to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        return (self.origin_x + (cx + 0.5) * self.resolution,
                self.origin_y + (cy + 0.5) * self.resolution)

    def inside(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.width and 0 <= cy < self.height


@dataclass
class PassResult:
    passable: bool
    reason: str
    tightest: float = 0.0                 # 경로 위 최소 여유 (m)
    tightest_at: Optional[Tuple[float, float]] = None
    blocked_at: Optional[Tuple[float, float]] = None
    detour: float = 0.0                   # 중심선에서 최대 얼마나 비켜야 하는가 (m)
    # 실제로 지나갈 수 있는 길 (world 좌표). 장애물을 비켜가는 모습을 그대로
    # 담고 있어 시각화와 시뮬레이션에 쓴다.
    path: List[Tuple[float, float]] = field(default_factory=list)

    def describe(self) -> str:
        if self.passable:
            s = '통과 가능 (통과폭 %.2fm' % (self.tightest * 2.0)
            if self.tightest_at:
                s += ' @%.2f,%.2f' % self.tightest_at
            if self.detour > 0.08:
                s += ', 중심선에서 %.2fm 비켜감' % self.detour
            return s + ')'
        s = '통과 불가: %s' % self.reason
        if self.blocked_at:
            s += ' @%.2f,%.2f' % self.blocked_at
        return s


class PassabilityGrid:
    """정적 지도 + 실시간 장애물을 합친 통행 판정 격자."""

    def __init__(self, info: GridInfo, occupied: Set[Tuple[int, int]]):
        self.info = info
        self._occupied = occupied
        self._clear: Optional[List[float]] = None

    # -- 생성 --------------------------------------------------------------

    @classmethod
    def from_arrays(cls, info: GridInfo, static_data: Sequence[int],
                    occ_threshold: int = 50,
                    unknown_blocks: bool = True) -> 'PassabilityGrid':
        """ROS OccupancyGrid.data 규약(-1 미지 / 0 자유 / 100 점유)으로 만든다.

        미지 영역은 기본적으로 막힌 것으로 본다. 경기장 밖이 전부 미지라
        자유로 취급하면 벽을 뚫고 지나가는 경로가 통과 가능으로 나온다.
        """
        occ: Set[Tuple[int, int]] = set()
        w = info.width
        for idx, v in enumerate(static_data):
            if v >= occ_threshold or (unknown_blocks and v < 0):
                occ.add((idx % w, idx // w))
        return cls(info, occ)

    def add_obstacle_cells(self, cells: Iterable[Tuple[int, int]]) -> None:
        """실시간 감지된 장애물 셀을 얹는다. 여유 계산은 무효화된다."""
        before = len(self._occupied)
        self._occupied.update(c for c in cells if self.info.inside(*c))
        if len(self._occupied) != before:
            self._clear = None

    def add_obstacle_points(self, points: Iterable[Tuple[float, float]],
                            radius: float = 0.0) -> None:
        """world 좌표 장애물을 얹는다. radius를 주면 그 반경만큼 부풀린다."""
        span = int(math.ceil(radius / self.info.resolution))
        cells = []
        for x, y in points:
            cx, cy = self.info.to_cell(x, y)
            if span <= 0:
                cells.append((cx, cy))
                continue
            for dy in range(-span, span + 1):
                for dx in range(-span, span + 1):
                    if dx * dx + dy * dy <= span * span:
                        cells.append((cx + dx, cy + dy))
        self.add_obstacle_cells(cells)

    # -- 여유 (거리 변환) ---------------------------------------------------

    def clearance_grid(self) -> List[float]:
        """각 셀에서 가장 가까운 장애물까지의 거리(m). 2패스 chamfer 거리변환.

        정확한 유클리드는 아니지만(오차 ~2%), 통행 판정에는 충분하고 격자
        21k개 기준 수 ms로 끝난다. BFS 다중소스보다 코드가 짧고 빠르다.
        """
        if self._clear is not None:
            return self._clear

        info = self.info
        w, h, res = info.width, info.height, info.resolution
        BIG = float(w + h) * 2.0
        d = [0.0 if (i % w, i // w) in self._occupied else BIG
             for i in range(w * h)]

        # 순방향
        for cy in range(h):
            base = cy * w
            for cx in range(w):
                i = base + cx
                if d[i] == 0.0:
                    continue
                best = d[i]
                if cx > 0:
                    best = min(best, d[i - 1] + 1.0)
                if cy > 0:
                    best = min(best, d[i - w] + 1.0)
                    if cx > 0:
                        best = min(best, d[i - w - 1] + _SQRT2)
                    if cx < w - 1:
                        best = min(best, d[i - w + 1] + _SQRT2)
                d[i] = best
        # 역방향
        for cy in range(h - 1, -1, -1):
            base = cy * w
            for cx in range(w - 1, -1, -1):
                i = base + cx
                if d[i] == 0.0:
                    continue
                best = d[i]
                if cx < w - 1:
                    best = min(best, d[i + 1] + 1.0)
                if cy < h - 1:
                    best = min(best, d[i + w] + 1.0)
                    if cx > 0:
                        best = min(best, d[i + w - 1] + _SQRT2)
                    if cx < w - 1:
                        best = min(best, d[i + w + 1] + _SQRT2)
                d[i] = best

        self._clear = [v * res for v in d]
        return self._clear

    def clearance_at(self, x: float, y: float) -> float:
        cx, cy = self.info.to_cell(x, y)
        if not self.info.inside(cx, cy):
            return 0.0
        return self.clearance_grid()[cy * self.info.width + cx]

    # -- 통행 판정 ----------------------------------------------------------

    def route_passable(self,
                       start: Tuple[float, float],
                       polyline: Sequence[Tuple[float, float]],
                       margin: float = DEFAULT_MARGIN,
                       band: float = DEFAULT_BAND,
                       goal_tol: float = 0.25) -> PassResult:
        """현재 위치에서 polyline 끝까지, 경로 주변 band 안에서 갈 수 있는가.

        margin: 차량 반폭에 더할 여유. 최소 통과폭 = 2*(CAR_HALF_WIDTH + margin)
        band:   경로 중심선에서 이만큼까지 비켜가는 건 '같은 경로'로 본다
        """
        info = self.info
        need = CAR_HALF_WIDTH + margin
        clear = self.clearance_grid()

        if len(polyline) < 2:
            return PassResult(False, '경로에 구간이 없음')

        def make_ok(cells):
            def ok(cell):
                cx, cy = cell
                if not info.inside(cx, cy) or cell not in cells:
                    return False
                return clear[cy * info.width + cx] >= need
            return ok

        first_ok = make_ok(self._band_cells(polyline[:2], band))
        start_cell = self._nearest_ok(info.to_cell(*start), first_ok, radius=6)
        if start_cell is None:
            return PassResult(
                False, '현재 위치 주변에 통과 가능한 셀이 없음 (차가 이미 갇힘)',
                blocked_at=start)

        # 경유점 사이 구간을 **하나씩 따로** 판정하고, 각 구간은 **그 구간의
        # band 안에서만** 탐색한다.
        #
        # 예전에는 시작->최종목표를 한 번에, 경로 전체 band의 합집합 위에서
        # 탐색했다. 두 가지가 동시에 깨졌다.
        #
        #  (1) 순서를 건너뛴다. 경로가 출발점 근처로 되돌아오는 모양이면
        #      중간을 통째로 생략하고 지름길로 빠진다. 'B->출발 북측우회'가
        #      그랬다 - 북측 가장자리가 완전히 막혔는데도 반환된 통과 경로가
        #      y=0.97 아래에서만 움직이고 '통과 가능'이 나왔다.
        #  (2) 옆 통로로 샌다. 합집합 band는 경로가 지나는 모든 통로를 포함하므로,
        #      한 구간이 막히면 다른 구간의 통로로 크게 우회하는 길을 찾아낸다.
        #      실제로 동측 y=4.70이 막혔는데 남쪽->서쪽->북측통로로 한 바퀴 도는
        #      경로를 찾아 '통과 가능'이라고 했다.
        #
        # 구간별 band로 좁히면 둘 다 사라진다. '이 경로는 이 통로로만 간다'는
        # 원래 의도와도 맞는다. 구간별 band는 경유점 근처에서 서로 겹치므로
        # 이어지는 데 문제가 없고, 통로 안에서 장애물을 비켜가는 여유는 그대로다.
        legs = list(zip(polyline[:-1], polyline[1:]))
        goal_span = max(2, int(goal_tol / info.resolution))

        cur_cell = start_cell
        tightest = float('inf')
        tight_at = None
        detour = 0.0
        full_path: List[Tuple[float, float]] = []
        reached_all: Set[Tuple[int, int]] = set()

        for li, leg in enumerate(legs):
            last_leg = li == len(legs) - 1
            ok = first_ok if li == 0 else make_ok(self._band_cells(leg, band))
            goal_cell = self._nearest_ok(info.to_cell(*leg[1]), ok,
                                         radius=goal_span)
            if goal_cell is None:
                return PassResult(False,
                                  '구간 목표 지점이 막힘' if last_leg
                                  else '경유점 %d이 막힘' % (li + 1),
                                  blocked_at=leg[1])

            # 이전 구간의 끝이 이번 구간의 band 밖일 수 있다(경유점에서 꺾일 때).
            # 가장 가까운 통행 가능 셀로 옮겨 붙인다.
            leg_start = cur_cell if ok(cur_cell) else self._nearest_ok(
                cur_cell, ok, radius=goal_span)
            if leg_start is None:
                return PassResult(False, '경유점 %d 부근이 막힘' % (li + 1),
                                  blocked_at=leg[0])

            res = self._widest(leg_start, goal_cell, ok, clear, polyline,
                               want_reached=True)
            if res is None or res[0] is None:
                # 막힌 지점은 **이 구간 위에서** 찾아야 한다. 경로 전체에서
                # 찾으면 이미 통과한 앞 구간의 엉뚱한 점이 나온다.
                settled = res[1] if res is not None else reached_all
                blocked = self._first_blocked(list(leg), clear, need, settled)
                return PassResult(False, '경로가 장애물로 끊김',
                                  blocked_at=blocked or leg[1])
            part, settled = res
            reached_all |= settled
            if part.tightest < tightest:
                tightest, tight_at = part.tightest, part.tightest_at
            detour = max(detour, part.detour)
            full_path.extend(part.path if not full_path else part.path[1:])
            cur_cell = goal_cell

        if tightest == float('inf'):
            tightest = clear[start_cell[1] * info.width + start_cell[0]]
        return PassResult(True, '통과 가능', tightest, tight_at,
                          detour=detour, path=full_path)

    def _widest(self, start_cell, goal_cell, ok, clear, polyline,
                want_reached: bool = False):
        """병목 최대화(widest path) 탐색.

        찾으면 (PassResult, 도달셀집합). 못 찾으면 None - 다만 want_reached면
        (None, 도달셀집합)을 돌려줘서 호출한 쪽이 '어디서 막혔는지'를 그 구간
        안에서 찾을 수 있게 한다.

        단순 BFS를 쓰면 "최소 조건만 겨우 만족하는 아무 경로"를 찾아서, 보고되는
        통과폭이 항상 임계값과 같아진다. 실제로 얼마나 여유 있게 지날 수 있는지
        알 수 없다. 여기서는 경로 위 최소 여유가 '최대'가 되는 길을 찾는다.
        Prim/Dijkstra 변형으로, 우선순위 큐에서 병목이 가장 큰 셀을 먼저 편다.
        """
        info = self.info
        best = {start_cell: clear[start_cell[1] * info.width + start_cell[0]]}
        parent = {start_cell: None}
        heap = [(-best[start_cell], start_cell)]
        settled: Set[Tuple[int, int]] = set()

        while heap:
            neg_b, cur = heapq.heappop(heap)
            if cur in settled:
                continue
            settled.add(cur)
            b = -neg_b
            if cur == goal_cell:
                return self._summarize(cur, parent, clear, polyline, b), settled
            cx, cy = cur
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = (cx + dx, cy + dy)
                if nxt in settled or not ok(nxt):
                    continue
                nb = min(b, clear[nxt[1] * info.width + nxt[0]])
                if nb > best.get(nxt, -1.0):
                    best[nxt] = nb
                    parent[nxt] = cur
                    heapq.heappush(heap, (-nb, nxt))
        return (None, settled) if want_reached else None

    # -- 내부 --------------------------------------------------------------

    def _band_cells(self, polyline, band) -> Set[Tuple[int, int]]:
        """경로 중심선에서 band 이내의 셀 집합."""
        info = self.info
        span = int(math.ceil(band / info.resolution))
        out: Set[Tuple[int, int]] = set()
        step = info.resolution * 0.8
        for a, b in zip(polyline[:-1], polyline[1:]):
            dist = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(1, int(dist / step))
            for k in range(n + 1):
                t = k / n
                x = a[0] + (b[0] - a[0]) * t
                y = a[1] + (b[1] - a[1]) * t
                cx, cy = info.to_cell(x, y)
                for dy in range(-span, span + 1):
                    for dx in range(-span, span + 1):
                        if dx * dx + dy * dy <= span * span:
                            out.add((cx + dx, cy + dy))
        return out

    def _nearest_ok(self, cell, ok, radius) -> Optional[Tuple[int, int]]:
        """cell 주변에서 통행 가능한 가장 가까운 셀. 위치추정 오차 흡수용."""
        if ok(cell):
            return cell
        cx, cy = cell
        for r in range(1, radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    c = (cx + dx, cy + dy)
                    if ok(c):
                        return c
        return None

    def _summarize(self, goal_cell, parents, clear, polyline,
                   bottleneck) -> PassResult:
        """통과 가능할 때, 가장 넓은 길의 병목과 중심선 이탈량을 요약."""
        info = self.info
        path = []
        cur = goal_cell
        while cur is not None:
            path.append(cur)
            cur = parents[cur]
        path.reverse()

        tightest = bottleneck
        at = None
        for cx, cy in path:
            if abs(clear[cy * info.width + cx] - bottleneck) < 1e-9:
                at = tuple(round(v, 2) for v in info.to_world(cx, cy))
                break

        # 통과 경로가 중심선에서 최대 얼마나 벗어나는가
        detour = 0.0
        for cx, cy in path[::3]:
            wx, wy = info.to_world(cx, cy)
            detour = max(detour, self._dist_to_polyline(wx, wy, polyline))

        world = [tuple(round(v, 3) for v in info.to_world(cx, cy))
                 for cx, cy in path]
        return PassResult(True, '통과 가능', tightest, at,
                          detour=detour, path=world)

    @staticmethod
    def _dist_to_polyline(x, y, polyline) -> float:
        best = float('inf')
        for a, b in zip(polyline[:-1], polyline[1:]):
            vx, vy = b[0] - a[0], b[1] - a[1]
            L2 = vx * vx + vy * vy
            if L2 < 1e-12:
                d = math.hypot(x - a[0], y - a[1])
            else:
                t = max(0.0, min(1.0, ((x - a[0]) * vx + (y - a[1]) * vy) / L2))
                d = math.hypot(x - (a[0] + t * vx), y - (a[1] + t * vy))
            best = min(best, d)
        return best

    def _first_blocked(self, polyline, clear, need, reached):
        """경로를 따라가며 BFS가 도달하지 못한 첫 지점."""
        info = self.info
        step = info.resolution
        for a, b in zip(polyline[:-1], polyline[1:]):
            dist = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(1, int(dist / step))
            for k in range(n + 1):
                t = k / n
                x = a[0] + (b[0] - a[0]) * t
                y = a[1] + (b[1] - a[1]) * t
                if self.info.to_cell(x, y) not in reached:
                    return (round(x, 2), round(y, 2))
        return None


def remaining_polyline(current: Tuple[float, float],
                       waypoints: Sequence[Tuple[float, float]],
                       from_index: int) -> List[Tuple[float, float]]:
    """현재 위치 + 아직 안 지난 경유점들. 판정 대상 경로를 만든다."""
    rest = list(waypoints[from_index:])
    return [current] + rest if rest else [current]


# ---------------------------------------------------------------------------
# self-test
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    import io
    import os
    import sys

    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

    here = os.path.dirname(os.path.abspath(__file__))
    pgm = os.path.join(os.path.dirname(here), 'parking_map.pgm')
    raw = open(pgm, 'rb').read()
    i, toks = 0, []
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
    W, H = int(toks[1]), int(toks[2])
    px = raw[i:]

    info = GridInfo(W, H, 0.05, -2.15, -1.05)
    # PGM은 위가 y최대. ROS OccupancyGrid는 아래가 y최소이므로 행을 뒤집는다.
    data = []
    for row in range(H - 1, -1, -1):
        for col in range(W):
            v = px[row * W + col]
            data.append(100 if v == 0 else (0 if v == 254 else -1))

    sys.path.insert(0, os.path.dirname(here))
    from parking_mission import mission_config as cfg

    def poly(route):
        return [(w.x, w.y) for w in route.waypoints]

    def check(label, route, obstacles, radius=0.06):
        g = PassabilityGrid.from_arrays(info, data)
        if obstacles:
            g.add_obstacle_points(obstacles, radius)
        start = (cfg.START_POSE.x, cfg.START_POSE.y)
        # 실제 노드와 똑같이 현재 위치를 경로 앞에 붙인다. 구간별 band로
        # 판정하므로, 이걸 빼면 첫 구간 band 밖에 서 있는 셈이 되어
        # '차가 이미 갇힘'이 나온다.
        r = g.route_passable(start, remaining_polyline(start, poly(route), 0),
                             band=0.40)
        mark = 'ok  ' if r.passable else 'BLOCK'
        print('  [%s] %-28s %s' % (mark, label, r.describe()))
        return r.passable

    north, west, east = cfg.LEG_TO_A.routes

    print('== 장애물 없음 (전 경로 통과해야 정상) ==')
    check('북측', north, [])
    check('서측', west, [])
    check('동측', east, [])

    print()
    print('== 북측 통로 한가운데 라바콘 1개 (x=0.85, y=2.4) ==')
    print('   자유폭 1.25m 구간이라 비켜서 통과 가능해야 한다')
    check('북측', north, [(0.85, 2.40)])

    print()
    print('== 북측 통로 최협부를 가로막음 (y=3.6, 자유폭 0.70m) ==')
    print('   0.70m에 장애물이 들어오면 통과 불가여야 한다')
    check('북측', north, [(0.90, 3.60), (1.00, 3.60), (1.10, 3.60)])
    check('서측 (대안)', west, [(0.90, 3.60), (1.00, 3.60), (1.10, 3.60)])

    print()
    print('== 서측 좁은목 봉쇄 (x=-0.675, y=3.6, 자유폭 0.50m) ==')
    check('서측', west, [(-0.675, 3.62)])
    check('북측 (대안)', north, [(-0.675, 3.62)])

    print()
    print('== 북측·서측 동시 봉쇄 -> 동측만 살아야 한다 ==')
    obs = [(0.90, 3.60), (1.00, 3.60), (1.10, 3.60), (-0.675, 3.62)]
    check('북측', north, obs)
    check('서측', west, obs)
    check('동측', east, obs)
