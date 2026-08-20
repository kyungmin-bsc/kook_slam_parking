<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: velocity-smoother
description: 'Configure Nav2 velocity smoother. Use when smoothing cmd_vel output, setting acceleration limits, or reducing jerky motion.'
---

# Velocity Smoother

## Purpose

The velocity smoother (`nav2_velocity_smoother::VelocitySmoother`) sits between the controller's raw `cmd_vel` output and the final velocity command sent to the motors. It enforces acceleration and deceleration limits, preventing jerky motion that can cause wheel slip, mechanical stress, or sensor disturbance (e.g., shaking a LIDAR).

## Position in Pipeline

```
Controller Server → /cmd_vel_nav → Velocity Smoother → /cmd_vel_smoothed → Collision Monitor → /cmd_vel
```

The velocity smoother subscribes to the controller's output and publishes rate-limited velocities. The collision monitor then applies the final safety check.

## Configuration

```yaml
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [0.5, 0.0, 1.0]      # [vx, vy, wz] m/s, m/s, rad/s
    min_velocity: [-0.5, 0.0, -1.0]     # Reverse limits
    max_accel: [2.5, 0.0, 3.2]          # [ax, ay, aw] m/s², m/s², rad/s²
    max_decel: [-2.5, 0.0, -3.2]        # Deceleration (negative values)
    deadband_velocity: [0.0, 0.0, 0.0]  # Velocities below this treated as zero
    odom_topic: "odom"
    odom_duration: 0.1
```

## Parameters

### Core Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `smoothing_frequency` | double | Rate (Hz) at which smoothing is applied. Should match or exceed controller frequency. |
| `scale_velocities` | bool | If true, scales all velocity components proportionally when one exceeds limits, maintaining the trajectory shape. If false, each component is clamped independently. |
| `feedback` | string | `"OPEN_LOOP"` or `"CLOSED_LOOP"`. Determines velocity estimation source. |

### Velocity Limits

All velocity arrays are `[vx, vy, wz]` format. For a differential drive robot, `vy` is always 0:

```yaml
max_velocity: [0.5, 0.0, 1.0]    # Max forward 0.5 m/s, max rotation 1.0 rad/s
min_velocity: [-0.5, 0.0, -1.0]  # Max reverse 0.5 m/s, max rotation reverse
```

- `max_velocity`: Upper velocity bounds (positive)
- `min_velocity`: Lower velocity bounds (negative for reverse). For differential drive: `[-max_reverse, 0.0, -max_angular]`

### Acceleration Limits

```yaml
max_accel: [2.5, 0.0, 3.2]    # Maximum acceleration per axis
max_decel: [-2.5, 0.0, -3.2]  # Maximum deceleration (MUST be negative)
```

The smoother ramps velocity up/down at these rates. On each cycle:
- If target velocity > current: apply `max_accel × dt`
- If target velocity < current: apply `max_decel × dt`
- If target velocity = 0: apply `max_decel × dt` to stop

### Deadband Velocity

```yaml
deadband_velocity: [0.05, 0.0, 0.1]
```

Velocities below the deadband are set to zero. This filters out noise from the controller that would cause the robot to creep. Useful when the controller outputs very small velocities near the goal.

## Feedback Modes

### OPEN_LOOP (Default)
The smoother tracks velocity internally based on what it last commanded. It does NOT read odometry—it assumes the robot perfectly executes the commanded velocity.

**Pros**: Simple, no odom dependency, no feedback delay.
**Cons**: If the robot stalls or slips, the smoother doesn't know and continues ramping.

### CLOSED_LOOP
The smoother reads the `odom_topic` to get actual robot velocity and uses that as the starting point for acceleration calculations.

```yaml
feedback: "CLOSED_LOOP"
odom_topic: "odom"
odom_duration: 0.1  # Seconds of odom history to average for velocity estimate
```

**Pros**: Accurate velocity tracking even with slippage or external forces.
**Cons**: Sensitive to odom quality and latency. `odom_duration` too high = sluggish response; too low = noisy.

For a differential drive robot with good wheel encoders, `OPEN_LOOP` is usually sufficient.

## scale_velocities Behavior

When `scale_velocities: true` and a velocity component hits its acceleration limit:

```
Commanded: vx=0.5, wz=1.0
vx can increase by 0.1 this cycle (accel limited)
wz could increase by 0.2 this cycle
Scale factor = 0.1/requested_vx_change
Apply same scale to wz → wz increases by less
```

This preserves the trajectory curvature. Without scaling, the robot might turn correctly but not advance fast enough (or vice versa), causing path deviation.

**For differential drive**: `scale_velocities: true` is recommended to maintain arc consistency.

## Tuning Guidelines

### max_accel Too High
- Wheel slip on hard floors
- Jerky motion disturbing LIDAR or camera
- Excessive current draw stressing motor drivers
- Tipping risk on ramps

### max_accel Too Low
- Robot feels sluggish and unresponsive
- Controller can't track the planned path (velocity lag)
- Recovery behaviors (spin, backup) take too long
- May trigger progress checker timeout because the robot accelerates too slowly

### Recommended Starting Values for Differential Drive

```yaml
# Conservative start for ~10kg indoor robot
max_velocity: [0.5, 0.0, 1.0]
min_velocity: [-0.3, 0.0, -1.0]
max_accel: [1.0, 0.0, 2.0]
max_decel: [-1.0, 0.0, -2.0]
deadband_velocity: [0.01, 0.0, 0.05]
```

Increase `max_accel` gradually while observing wheel encoder data. If odometry velocity consistently lags commanded velocity, the robot's motors can't actually achieve the acceleration—lower the limit to match reality.

## Topic Remapping

By default, velocity smoother subscribes to `/cmd_vel_nav` and publishes to `/cmd_vel_smoothed`. Remap in launch:

```python
Node(
    package='nav2_velocity_smoother',
    executable='velocity_smoother',
    name='velocity_smoother',
    remappings=[
        ('cmd_vel', '/cmd_vel_nav'),            # Input from controller
        ('cmd_vel_smoothed', '/cmd_vel_smoothed')  # Output to collision monitor
    ],
)
```

Ensure the collision monitor's `cmd_vel_in_topic` matches the smoother's output topic.
