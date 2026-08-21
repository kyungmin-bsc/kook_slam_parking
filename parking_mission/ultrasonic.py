#!/usr/bin/env python3
"""
초음파 기반 후진 안전 감시.

ROS에 의존하지 않는다. 판정 로직만 담고 있어 노트북에서 그대로 테스트할 수 있다
(파일 맨 아래 self-test). ROS 쪽은 mission_manager가 토픽을 받아 feed()로 넣어준다.

왜 초음파인가
------------
라이다가 base_link 기준 x=+0.41m, 즉 차 맨 앞에 달려 있다. 뒷범퍼는 -0.15m라
**라이다와 뒷범퍼 사이가 0.56m**다. 그런데 우리 주차 기동은 T자든 평행이든 전부
후진이다. 진행 방향에 있는 센서가 차체 반대편 끝에 있는 셈이고, 후방은 차체와
전장 부품에 가려질 가능성도 크다.

뒤 3개 초음파는 정확히 그 사각지대에서 진행 방향을 직접 본다.

한계 - 반드시 알고 써야 한다
--------------------------
경기장 벽이 매끈한 회색 평판이다. 초음파에는 최악의 조건이다.

  - **경면 반사**: 평평한 면이 비스듬히 놓이면 펄스가 센서로 안 돌아오고 옆으로
    튕긴다. 15도만 넘어가도 반사파를 못 받는다. 즉 비스듬한 벽 앞에서
    "아무것도 없음"이라고 답할 수 있다.
  - **원뿔 빔(±15도 내외)**: "이 원뿔 어딘가에 X cm 물체가 있다"까지만 알려주고
    방향은 모른다.
  - **라바콘**: 작고 둥글어서 반사 단면적이 나쁘다.
  - **갱신 주기**: 노드는 20Hz지만 아두이노가 5개를 순차 발사하면 센서당 실효
    4Hz까지 떨어진다. 0.3m/s에서 측정 간 7.5cm를 이동한다.

그래서 설계 원칙은 하나다:

    초음파는 "멈춰"는 신뢰할 수 있지만 "가도 돼"는 신뢰할 수 없다.

에코가 왔으면 진짜로 뭐가 있는 것이므로 비상정지 근거로 충분하다. 에코가
없다고 비어 있다는 뜻은 아니므로 안전 확인 근거로는 쓰지 않는다. 경로 판단은
계속 라이다/costmap이 맡는다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# 채널 배치 (실차 확인값)
# ---------------------------------------------------------------------------
# 뒤에 3개가 나란히, 좌우에 1개씩. 총 5개.
CH_LEFT = 0          # 좌측면
CH_RIGHT = 4         # 우측면
CH_REAR_RIGHT = 5    # 후방 우
CH_REAR_CENTER = 6   # 후방 중앙
CH_REAR_LEFT = 7     # 후방 좌

REAR_CHANNELS = (CH_REAR_LEFT, CH_REAR_CENTER, CH_REAR_RIGHT)
SIDE_CHANNELS = (CH_LEFT, CH_RIGHT)
ALL_CHANNELS = REAR_CHANNELS + SIDE_CHANNELS

# 배열 길이가 이만큼은 되어야 ch7까지 읽을 수 있다.
# ultra_node.py는 시리얼 라인 앞에서부터 num_sensors개만 잘라 발행하는데
# 기본값이 4다. 그대로 두면 후방 3개가 통째로 잘린다. num_sensors:=8 필수.
MIN_ARRAY_LEN = 8

CHANNEL_NAMES = {
    CH_LEFT: '좌측',
    CH_RIGHT: '우측',
    CH_REAR_LEFT: '후좌',
    CH_REAR_CENTER: '후중',
    CH_REAR_RIGHT: '후우',
}

# TODO(현장): 좌/우가 뒤바뀌었는지 반드시 확인할 것.
# "차량 뒤에서 바라보는 기준"이라는 설명은 두 가지로 읽힌다.
#   (a) 차 뒤에 서서 차와 같은 방향을 볼 때의 좌우 -> 차량 자체 좌우와 동일
#   (b) 차 뒤에 서서 차를 마주볼 때의 좌우      -> 차량 자체 좌우와 반대
# 둘이 뒤바뀌면 평행주차 측면 간격 판정이 반대로 나온다.
# 확인법: 차를 세워두고 좌측면 센서를 손으로 가린 뒤
#   ros2 topic echo /xycar_ultrasonic
# 에서 인덱스 0이 변하는지 본다. 인덱스 4가 변하면 SWAP_SIDES를 True로.
SWAP_SIDES = False


@dataclass
class GuardConfig:
    """후진 안전 감시 설정. 전부 ROS 파라미터로 덮어쓸 수 있다."""

    unit_to_m: float = 0.01      # 센서 값 단위 -> m. 실차 확인 완료: cm
    min_valid: float = 0.02      # HC-SR04 최소 측정거리. 이하는 무효
    # xycar_ultrasonic.cpp가 140을 넘는 값을 0으로 만든다.
    #   if (value < 0 || value > 140) { value = 0; }
    # 그래서 140cm가 이 드라이버로 얻을 수 있는 최대다. 200 같은 값을 기대하면
    # 안 된다(뷰어는 >=200을 INF로 표시하지만 그 값은 영원히 오지 않는다).
    max_valid: float = 1.40

    # 실측 근거: 지도에 기동 궤적을 얹고 후방 3개 센서의 원뿔 빔(+-15도)을
    # 광선 추적해서 잰 값이다.
    #   A구역 T자 후진 - 후진 중 후방 최소 47cm
    #   B구역 평행주차 - 후진 중 후방 최소 53cm
    # 정상 기동에서는 25cm에 닿지 않는다. 여기서 정지가 걸리면 지도에 없는
    # 진짜 장애물이다.
    stop_distance: float = 0.25
    # 40cm로 두면 A구역 막바지(47cm)를 스치며 불필요하게 감속한다.
    slow_distance: float = 0.35
    side_warn: float = 0.12      # 측면이 이보다 가까우면 경고 (정지는 안 함)

    confirm_hits: int = 2        # 연속 몇 번 임계 이하여야 정지로 확정
    stale_sec: float = 0.6       # 이 시간 넘게 수신 없으면 후진 금지
    resume_clear_hits: int = 2   # 연속 몇 번 안전해야 재개


@dataclass
class GuardVerdict:
    safe: bool
    slow: bool
    reason: str
    rear: Optional[float] = None       # 후방 최소 거리 (m), 유효값 없으면 None
    trigger: Optional[str] = None      # 정지를 유발한 센서 이름

    def describe(self) -> str:
        if self.rear is None:
            d = '후방 유효값 없음'
        else:
            d = '후방 %.2fm' % self.rear
        if self.safe:
            return ('%s%s' % (d, ' (감속)' if self.slow else ''))
        return '%s - %s' % (d, self.reason)


class UltrasonicGuard:
    """초음파 판정기. 원시 배열을 받아 후진 가부를 돌려준다."""

    def __init__(self, cfg: Optional[GuardConfig] = None):
        self.cfg = cfg or GuardConfig()
        self._values: Dict[int, Optional[float]] = {c: None for c in ALL_CHANNELS}
        self._last_stamp: Optional[float] = None
        self._bad_hits = 0
        self._good_hits = 0
        self._holding = False
        self._short_count = 0

    # -- 입력 --------------------------------------------------------------

    def feed(self, raw: Sequence[int], stamp: float) -> None:
        """/xycar_ultrasonic 배열을 넣는다. 단위 변환과 유효성 판정을 여기서 한다."""
        if len(raw) < MIN_ARRAY_LEN:
            # ch7까지 못 읽는 배열이다. 드라이버가 시리얼을 라인 경계 없이 읽어서
            # 토큰이 8개 미만으로 잡히면 msg.data가 짧게 나올 수 있다.
            #
            # 이걸 즉시 '후진 차단'으로 다루면 간헐적인 짧은 읽기마다 후진이
            # 툭툭 끊긴다. 그래서 '이번 갱신은 무시'로 처리하고, 계속 짧게만
            # 들어오면 stale 타이머가 잡도록 둔다.
            self._short_count += 1
            return
        self._short_count = 0
        for ch in ALL_CHANNELS:
            self._values[ch] = self._to_meters(raw[ch])
        self._last_stamp = stamp

    def _to_meters(self, v) -> Optional[float]:
        """0/음수는 측정 실패다. '무한히 멀다'가 아니라 '모른다'로 다뤄야 한다."""
        try:
            m = float(v) * self.cfg.unit_to_m
        except (TypeError, ValueError):
            return None
        if m < self.cfg.min_valid or m > self.cfg.max_valid:
            return None
        return m

    # -- 조회 --------------------------------------------------------------

    def channel(self, ch: int) -> Optional[float]:
        if SWAP_SIDES:
            ch = {CH_LEFT: CH_RIGHT, CH_RIGHT: CH_LEFT,
                  CH_REAR_LEFT: CH_REAR_RIGHT,
                  CH_REAR_RIGHT: CH_REAR_LEFT}.get(ch, ch)
        return self._values.get(ch)

    def rear_min(self) -> Optional[float]:
        """후방 3개 중 최소. 유효값이 하나도 없으면 None."""
        vals = [self._values[c] for c in REAR_CHANNELS]
        vals = [v for v in vals if v is not None]
        return min(vals) if vals else None

    def side_min(self) -> Optional[float]:
        vals = [self._values[c] for c in SIDE_CHANNELS]
        vals = [v for v in vals if v is not None]
        return min(vals) if vals else None

    def snapshot(self) -> str:
        """로그용 한 줄 요약."""
        parts = []
        for ch in (CH_REAR_LEFT, CH_REAR_CENTER, CH_REAR_RIGHT,
                   CH_LEFT, CH_RIGHT):
            v = self._values[ch]
            parts.append('%s=%s' % (CHANNEL_NAMES[ch],
                                    '--' if v is None else '%.2f' % v))
        return ' '.join(parts)

    # -- 판정 --------------------------------------------------------------

    def check(self, direction: int, now: float) -> GuardVerdict:
        """direction: +1 전진 / -1 후진.

        전진은 초음파가 앞에 없으므로 항상 안전으로 답한다(라이다/costmap 담당).
        """
        c = self.cfg
        if direction >= 0:
            return GuardVerdict(True, False, '전진 - 초음파 감시 대상 아님')

        if self._last_stamp is None:
            return GuardVerdict(False, False, '초음파 미수신 - 후진 금지')
        age = now - self._last_stamp
        if age > c.stale_sec:
            return GuardVerdict(
                False, False, '초음파 %.1fs째 갱신 없음 - 후진 금지' % age)

        rear = self.rear_min()
        if rear is None:
            # 세 센서가 전부 0이다. 이 드라이버에서는 흔한 정상 상황이다 -
            # 140cm를 넘으면 0이 되므로, 뒤가 1.4m 넘게 비어 있으면 전부 0이다.
            # (실제로 B구역 평행주차 후진 구간의 22%가 여기 해당한다)
            #
            # 그러니 이걸 이상 상황으로 보고 감속하면 정상 주행을 계속 방해한다.
            # 원칙은 그대로다 - 0을 '비어 있음'으로 확신하는 게 아니라, 정지시킬
            # 근거가 없으니 라이다/costmap에 맡기고 진행하는 것이다.
            return GuardVerdict(True, False, '후방 1.4m 밖 (정상)', None)

        if rear <= c.stop_distance:
            self._bad_hits += 1
            self._good_hits = 0
            if self._bad_hits >= c.confirm_hits:
                self._holding = True
                trig = min(
                    ((self._values[ch], ch) for ch in REAR_CHANNELS
                     if self._values[ch] is not None),
                    default=(None, None))[1]
                return GuardVerdict(
                    False, False,
                    '후방 %.2fm (정지선 %.2fm)' % (rear, c.stop_distance),
                    rear, CHANNEL_NAMES.get(trig))
            return GuardVerdict(True, True,
                                '후방 근접 확인 중 %d/%d'
                                % (self._bad_hits, c.confirm_hits), rear)

        self._bad_hits = 0
        if self._holding:
            self._good_hits += 1
            if self._good_hits < c.resume_clear_hits:
                return GuardVerdict(False, False,
                                    '해제 확인 중 %d/%d'
                                    % (self._good_hits, c.resume_clear_hits),
                                    rear)
            self._holding = False
            self._good_hits = 0

        return GuardVerdict(True, rear <= c.slow_distance,
                            '정상', rear)

    def side_warning(self) -> Optional[str]:
        """측면이 너무 가까우면 경고 문구. 정지는 시키지 않는다."""
        out = []
        for ch in SIDE_CHANNELS:
            v = self.channel(ch)
            if v is not None and v <= self.cfg.side_warn:
                out.append('%s %.2fm' % (CHANNEL_NAMES[ch], v))
        return ' / '.join(out) if out else None


# ---------------------------------------------------------------------------
# self-test
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    import io
    import sys

    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

    def arr(left=120, right=120, rl=120, rc=120, rr=120):
        """8칸 배열 생성. 값은 cm."""
        a = [0] * 8
        a[CH_LEFT] = left
        a[CH_RIGHT] = right
        a[CH_REAR_LEFT] = rl
        a[CH_REAR_CENTER] = rc
        a[CH_REAR_RIGHT] = rr
        return a

    def show(label, verdict, expect_safe):
        mark = 'ok  ' if verdict.safe == expect_safe else 'FAIL'
        print('  [%s] %-34s safe=%-5s %s'
              % (mark, label, verdict.safe, verdict.describe()))
        return verdict.safe == expect_safe

    ok = True
    print('== 정상 후진 ==')
    g = UltrasonicGuard()
    g.feed(arr(rc=120), 0.0)
    ok &= show('후방 1.20m', g.check(-1, 0.05), True)

    print()
    print('== 전진은 감시 대상 아님 ==')
    ok &= show('전진', g.check(+1, 0.05), True)

    print()
    print('== 후방 접근 -> 연속 2회 확인 후 정지 ==')
    g = UltrasonicGuard()
    g.feed(arr(rc=20), 0.0)
    ok &= show('1회차 (아직 진행, 감속)', g.check(-1, 0.05), True)
    g.feed(arr(rc=20), 0.1)
    ok &= show('2회차 -> 정지', g.check(-1, 0.15), False)

    print()
    print('== 단발 스파이크는 무시 ==')
    g = UltrasonicGuard()
    g.feed(arr(rc=120), 0.0)
    g.check(-1, 0.01)
    g.feed(arr(rc=15), 0.1)
    ok &= show('스파이크 1회', g.check(-1, 0.11), True)
    g.feed(arr(rc=120), 0.2)
    ok &= show('복귀', g.check(-1, 0.21), True)

    print()
    print('== 정지 후 해제도 연속 확인 필요 ==')
    g = UltrasonicGuard()
    for t in (0.0, 0.1):
        g.feed(arr(rc=18), t)
        g.check(-1, t + 0.01)
    g.feed(arr(rc=120), 0.2)
    ok &= show('해제 1회차 (아직 정지)', g.check(-1, 0.21), False)
    g.feed(arr(rc=120), 0.3)
    ok &= show('해제 2회차 -> 재개', g.check(-1, 0.31), True)

    print()
    print('== 센서 죽음 / 배열 짧음 ==')
    g = UltrasonicGuard()
    ok &= show('미수신', g.check(-1, 0.0), False)
    g = UltrasonicGuard()
    g.feed(arr(rc=100), 0.0)
    ok &= show('0.9s 갱신 없음', g.check(-1, 0.9), False)
    g = UltrasonicGuard()
    g.feed([100, 100, 100, 100], 0.0)     # 짧은 배열 - 갱신 무시
    ok &= show('배열 4칸 직후 (미수신 상태)', g.check(-1, 0.05), False)
    g = UltrasonicGuard()
    g.feed(arr(rc=100), 0.0)              # 정상 1회
    g.feed([100, 100], 0.1)               # 짧은 배열 끼어듦 -> 무시
    ok &= show('정상 후 짧은 배열 끼어듦', g.check(-1, 0.15), True)

    print()
    print('== 후방 전부 0 = 1.4m 밖 (드라이버가 140 초과를 0으로 만듦) ==')
    g = UltrasonicGuard()
    g.feed(arr(rl=0, rc=0, rr=0), 0.0)
    v = g.check(-1, 0.05)
    ok &= show('후방 전부 0 -> 정상 진행', v, True)
    print('        slow=%s (감속하면 안 된다 - 흔한 정상 상황이다)' % v.slow)
    ok &= (v.slow is False)

    print()
    print('== 140 초과값은 애초에 오지 않지만, 와도 무효 처리 ==')
    g = UltrasonicGuard()
    g.feed(arr(rc=180), 0.0)
    v = g.check(-1, 0.05)
    ok &= show('rc=180 -> 무효', v, True)

    print()
    print('== 측면 경고 ==')
    g = UltrasonicGuard()
    g.feed(arr(left=8), 0.0)
    w = g.side_warning()
    print('  [%s] 좌측 0.08m -> %s' % ('ok  ' if w else 'FAIL', w))
    ok &= bool(w)

    print()
    print('== 스냅샷 ==')
    g = UltrasonicGuard()
    g.feed(arr(left=45, right=60, rl=33, rc=28, rr=31), 0.0)
    print('  %s' % g.snapshot())

    print()
    print('ALL PASS' if ok else '*** 일부 실패 ***')
