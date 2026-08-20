<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: custom-rviz-overlay
description: 'Create custom RViz overlay displays. Use when showing battery status, velocity, or navigation state as 2D overlays in RViz.'
---

# Creating Custom RViz Overlay Displays

## Overview

RViz overlays render 2D text, plots, and status indicators directly on the 3D viewport. The primary package is `rviz_2d_overlay_plugins` (from `ros-jazzy-rviz-2d-overlay-plugins`).

```bash
sudo apt install ros-jazzy-rviz-2d-overlay-plugins
```

## OverlayText Message

```bash
# Message definition: rviz_2d_overlay_msgs/msg/OverlayText
# int32 width, height          — overlay box size in pixels
# int32 left, top              — position from viewport edges
# std_msgs/ColorRGBA bg_color  — background color (use alpha for transparency)
# std_msgs/ColorRGBA fg_color  — text color
# int32 line_width             — text line width
# int32 text_size              — font size in points
# string font                  — font name
# string text                  — content (supports <span style="color: red;">colored</span> HTML)
```

## Battery Status Overlay Publisher

```python
#!/usr/bin/env python3
"""Publish battery status as an RViz overlay with color-coded voltage."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import BatteryState
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA


class BatteryOverlayPublisher(Node):
    def __init__(self):
        super().__init__("battery_overlay_publisher")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.sub = self.create_subscription(
            BatteryState, "/battery_state", self.battery_callback, sensor_qos
        )
        self.pub = self.create_publisher(OverlayText, "/overlay/battery", 10)

    def battery_callback(self, msg: BatteryState):
        overlay = OverlayText()
        overlay.width = 280
        overlay.height = 100
        overlay.left = 10
        overlay.top = 10
        overlay.text_size = 14
        overlay.font = "DejaVu Sans Mono"
        overlay.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.6)

        voltage = msg.voltage
        current = abs(msg.current)
        pct = msg.percentage * 100

        # Color code: green > 50%, yellow 20-50%, red < 20%
        if pct > 50:
            overlay.fg_color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0)
        elif pct > 20:
            overlay.fg_color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        else:
            overlay.fg_color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)

        status_str = {0: "UNKNOWN", 1: "CHARGING", 2: "DISCHARGING",
                      3: "NOT_CHARGING", 4: "FULL"}.get(msg.power_supply_status, "?")

        overlay.text = (
            f"Battery: {pct:.0f}%\n"
            f"Voltage: {voltage:.1f}V\n"
            f"Current: {current:.1f}A\n"
            f"Status:  {status_str}"
        )

        self.pub.publish(overlay)


def main():
    rclpy.init()
    node = BatteryOverlayPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## Multi-Info Dashboard Node

```python
#!/usr/bin/env python3
"""Dashboard overlay: velocity, nav state, WiFi, CPU."""

import psutil
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA


class DashboardOverlay(Node):
    def __init__(self):
        super().__init__("dashboard_overlay")
        self.vel_linear = 0.0
        self.vel_angular = 0.0
        self.nav_state = "IDLE"

        self.create_subscription(Twist, "/cmd_vel", self.vel_cb, 10)
        self.create_subscription(String, "/nav_state", self.nav_cb, 10)

        # Velocity overlay — top right
        self.vel_pub = self.create_publisher(OverlayText, "/overlay/velocity", 10)
        # System overlay — bottom left
        self.sys_pub = self.create_publisher(OverlayText, "/overlay/system", 10)

        self.create_timer(0.5, self.publish_overlays)

    def vel_cb(self, msg: Twist):
        self.vel_linear = msg.linear.x
        self.vel_angular = msg.angular.z

    def nav_cb(self, msg: String):
        self.nav_state = msg.data

    def publish_overlays(self):
        # Velocity overlay
        vel = OverlayText()
        vel.width = 240
        vel.height = 70
        vel.left = -250  # negative = from right edge (not all plugins support this)
        vel.top = 10
        vel.text_size = 14
        vel.font = "DejaVu Sans Mono"
        vel.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.5)
        vel.fg_color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=1.0)
        vel.text = (
            f"Linear:  {self.vel_linear:+.2f} m/s\n"
            f"Angular: {self.vel_angular:+.2f} rad/s\n"
            f"Nav:     {self.nav_state}"
        )
        self.vel_pub.publish(vel)

        # System overlay
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory().percent
        sys_overlay = OverlayText()
        sys_overlay.width = 200
        sys_overlay.height = 50
        sys_overlay.left = 10
        sys_overlay.top = 120  # below battery overlay
        sys_overlay.text_size = 12
        sys_overlay.font = "DejaVu Sans Mono"
        sys_overlay.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.5)

        color_r = min(1.0, cpu / 100.0)
        sys_overlay.fg_color = ColorRGBA(r=color_r, g=1.0 - color_r, b=0.0, a=1.0)
        sys_overlay.text = f"CPU: {cpu:.0f}%  MEM: {mem:.0f}%"
        self.sys_pub.publish(sys_overlay)


def main():
    rclpy.init()
    rclpy.spin(DashboardOverlay())
    rclpy.shutdown()
```

## RViz Config for Overlays

Add these displays to your `.rviz` config:

```yaml
    - Class: rviz_2d_overlay_plugins/OverlayText
      Name: BatteryOverlay
      Enabled: true
      Topic:
        Value: /overlay/battery
        Depth: 5
        Reliability Policy: Reliable

    - Class: rviz_2d_overlay_plugins/OverlayText
      Name: VelocityOverlay
      Enabled: true
      Topic:
        Value: /overlay/velocity
        Depth: 5
        Reliability Policy: Reliable

    - Class: rviz_2d_overlay_plugins/OverlayText
      Name: SystemOverlay
      Enabled: true
      Topic:
        Value: /overlay/system
        Depth: 5
        Reliability Policy: Reliable
```

## Marker-Based HUD Alternative

When `rviz_2d_overlay_plugins` isn't available, use `visualization_msgs/Marker` with `TEXT_VIEW_FACING` type. These appear in 3D space (not screen-fixed) but can be placed relative to the robot:

```python
from visualization_msgs.msg import Marker

marker = Marker()
marker.header.frame_id = "base_link"
marker.type = Marker.TEXT_VIEW_FACING
marker.action = Marker.ADD
marker.pose.position.z = 1.0  # float above robot
marker.scale.z = 0.15  # text height in meters
marker.color.a = 1.0
marker.color.g = 1.0
marker.text = f"Battery: {pct:.0f}%"
```

## jsk_rviz_plugins Alternative

The `jsk_rviz_plugins` package provides additional overlay types: pie charts, bounding boxes, pictograms, and status indicators. Install with `sudo apt install ros-jazzy-jsk-rviz-plugins` (if available for your distro).

## Dashboard Layout

```
┌─────────────────────────────────────────┐
│ [Battery: 78%   ] ←top-left    [Vel]→  │
│ [Voltage: 12.4V ]             top-right │
│ [CPU: 45% MEM:62%]                      │
│                                         │
│            3D Viewport                  │
│                                         │
│                                         │
│ [Nav: NAVIGATING]          [Plot]→      │
│   bottom-left            bottom-right   │
└─────────────────────────────────────────┘
```
