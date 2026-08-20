#!/usr/bin/env python3
"""
주차 기동 경로 시각화.

nav_msgs/Path로는 전진/후진을 색으로 구분할 수 없다. Path는 점 나열일 뿐이고
RViz의 Path 디스플레이는 경로 전체에 색 하나만 칠한다. 그래서 구간별로 색을
바꾸려면 visualization_msgs/MarkerArray로 LINE_STRIP을 구간 수만큼 따로 발행해야 한다.

발행 내용
  - 구간별 LINE_STRIP: 전진=초록, 후진=주황
  - 구간 시작점의 방향 화살표 (차가 어느 쪽을 보고 있는지)
  - 구간 번호/종류 텍스트
  - 최종 목표 주차 pose (파란 화살표)
  - 기동 시작 시점의 차량 footprint (실측 치수, 어디서 출발하는지 확인용)
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import Pose2D, Prim, sample_by_prim

# 실측 footprint (base_link = 뒷차축 중심)
FOOT_FRONT = 0.46
FOOT_REAR = -0.15
FOOT_HALFW = 0.15

FORWARD_COLOR = ColorRGBA(r=0.15, g=0.85, b=0.25, a=1.0)   # 초록 = 전진
REVERSE_COLOR = ColorRGBA(r=1.00, g=0.45, b=0.05, a=1.0)   # 주황 = 후진
GOAL_COLOR = ColorRGBA(r=0.20, g=0.55, b=1.00, a=1.0)      # 파랑 = 목표
FOOT_COLOR = ColorRGBA(r=1.00, g=1.00, b=0.30, a=0.9)      # 노랑 = 차체


def _quat(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _pt(x: float, y: float, z: float = 0.0) -> Point:
    return Point(x=float(x), y=float(y), z=float(z))


def _base(ns: str, mid: int, mtype: int, frame: str, stamp) -> Marker:
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.ns = ns
    m.id = mid
    m.type = mtype
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    return m


def clear_all(frame: str, stamp) -> MarkerArray:
    """이전 기동의 마커를 지운다. 새 기동을 그리기 전에 먼저 발행할 것."""
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.action = Marker.DELETEALL
    return MarkerArray(markers=[m])


def footprint_marker(pose: Pose2D, frame: str, stamp,
                     ns: str = 'maneuver_footprint', mid: int = 0) -> Marker:
    """주어진 pose에서의 차체 외곽선."""
    m = _base(ns, mid, Marker.LINE_STRIP, frame, stamp)
    m.scale.x = 0.015
    m.color = FOOT_COLOR
    corners = [
        (FOOT_FRONT, FOOT_HALFW), (FOOT_FRONT, -FOOT_HALFW),
        (FOOT_REAR, -FOOT_HALFW), (FOOT_REAR, FOOT_HALFW),
        (FOOT_FRONT, FOOT_HALFW),
    ]
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    m.points = [_pt(pose.x + c * lx - s * ly, pose.y + s * lx + c * ly)
                for lx, ly in corners]
    return m


def maneuver_markers(start: Pose2D, prims: List[Prim], radius: float,
                     goal: Optional[Pose2D], frame: str, stamp,
                     step: float = 0.02) -> MarkerArray:
    """기동 전체를 색 구분된 MarkerArray로 만든다.

    start/prims/goal 모두 표시할 프레임(보통 'map') 기준이어야 한다.
    """
    arr = MarkerArray()
    mid = 0

    for idx, (prim, poses) in enumerate(sample_by_prim(start, prims, radius, step)):
        forward = prim.direction > 0
        color = FORWARD_COLOR if forward else REVERSE_COLOR

        line = _base('maneuver_path', mid, Marker.LINE_STRIP, frame, stamp)
        mid += 1
        line.scale.x = 0.03
        line.color = color
        line.points = [_pt(p.x, p.y, 0.02) for p in poses]
        arr.markers.append(line)

        # 구간 시작점의 진행 방향 화살표. 후진 구간은 화살표를 뒤로 돌려서
        # '차가 어디를 보는지'가 아니라 '어디로 움직이는지'를 나타낸다.
        head = poses[0]
        arrow = _base('maneuver_dir', mid, Marker.ARROW, frame, stamp)
        mid += 1
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.18, 0.04, 0.04
        arrow.color = color
        arrow.pose.position = _pt(head.x, head.y, 0.05)
        arrow.pose.orientation = _quat(head.yaw if forward else head.yaw + math.pi)
        arr.markers.append(arrow)

        label = _base('maneuver_label', mid, Marker.TEXT_VIEW_FACING, frame, stamp)
        mid += 1
        label.scale.z = 0.09
        label.color = color
        label.pose.position = _pt(head.x, head.y, 0.22)
        kind = '직선' if prim.kind == 'S' else (
            '좌호' if prim.turn > 0 else '우호')
        label.text = '%d.%s%s %.2fm' % (
            idx + 1, '전진' if forward else '후진', kind, prim.length)
        arr.markers.append(label)

    # 출발 시점 차체
    arr.markers.append(footprint_marker(start, frame, stamp, mid=mid))
    mid += 1

    if goal is not None:
        g = _base('maneuver_goal', mid, Marker.ARROW, frame, stamp)
        mid += 1
        g.scale.x, g.scale.y, g.scale.z = 0.30, 0.06, 0.06
        g.color = GOAL_COLOR
        g.pose.position = _pt(goal.x, goal.y, 0.06)
        g.pose.orientation = _quat(goal.yaw)
        arr.markers.append(g)

        arr.markers.append(
            footprint_marker(goal, frame, stamp, ns='maneuver_goal_foot', mid=mid))

    return arr


def legend_text(frame: str, stamp, at: Tuple[float, float] = (-1.6, 5.9)) -> Marker:
    """RViz 화면 구석에 색 범례를 띄운다."""
    m = _base('maneuver_legend', 0, Marker.TEXT_VIEW_FACING, frame, stamp)
    m.scale.z = 0.13
    m.color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=1.0)
    m.pose.position = _pt(at[0], at[1], 0.3)
    m.text = '초록=전진  주황=후진  파랑=목표'
    return m


# 경로 우선순위별 색. 지금 몇 순위로 가고 있는지 색만 보고 알 수 있어야 한다.
ROUTE_COLORS = [
    ColorRGBA(r=0.20, g=0.80, b=1.00, a=0.95),   # 1순위 하늘
    ColorRGBA(r=1.00, g=0.85, b=0.10, a=0.95),   # 2순위 노랑
    ColorRGBA(r=1.00, g=0.30, b=0.60, a=0.95),   # 3순위 분홍
]


def route_markers(route, index: int, frame: str, stamp) -> MarkerArray:
    """선택된 이동 경로의 경유점과 연결선.

    폴백이 일어나면 색이 바뀌므로, RViz만 보고도 지금 몇 순위 경로를 타고
    있는지 즉시 알 수 있다.
    """
    color = ROUTE_COLORS[min(index, len(ROUTE_COLORS) - 1)]
    arr = MarkerArray()

    clear = Marker()
    clear.header.frame_id = frame
    clear.action = Marker.DELETEALL
    arr.markers.append(clear)

    line = _base('route_line', 0, Marker.LINE_STRIP, frame, stamp)
    line.scale.x = 0.025
    line.color = color
    line.points = [_pt(w.x, w.y, 0.01) for w in route.waypoints]
    arr.markers.append(line)

    for i, w in enumerate(route.waypoints):
        dot = _base('route_wp', i, Marker.SPHERE, frame, stamp)
        dot.scale.x = dot.scale.y = dot.scale.z = 0.07
        dot.color = color
        dot.pose.position = _pt(w.x, w.y, 0.01)
        arr.markers.append(dot)

    head = _base('route_label', 0, Marker.TEXT_VIEW_FACING, frame, stamp)
    head.scale.z = 0.13
    head.color = color
    head.pose.position = _pt(route.waypoints[0].x, route.waypoints[0].y, 0.35)
    head.text = '%d순위 %s (마진 %.2fm)' % (index + 1, route.name, route.margin)
    arr.markers.append(head)
    return arr
