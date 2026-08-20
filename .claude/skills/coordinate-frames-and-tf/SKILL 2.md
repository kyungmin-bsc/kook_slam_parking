<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: coordinate-frames-and-tf
description: 'Understand ROS 2 coordinate frames (REP 105). Use when working with map, odom, base_link, base_footprint, or sensor frames.'
---

# Coordinate Frames and TF for Navigation

## REP 105 Standard Frames

ROS defines four standard frames for mobile robot navigation:

| Frame | Type | Description |
|-------|------|-------------|
| `map` | Fixed, global | World-fixed frame aligned to the map. Discontinuous (jumps when localization corrects drift). |
| `odom` | Continuous, local | Smooth and continuous but drifts over time. Origin is where the robot started (or was last reset). |
| `base_link` | Robot body | Rigidly attached to the robot chassis. Origin at rotation center. |
| `base_footprint` | Ground projection | Projection of `base_link` onto the ground plane. Used by Nav2 for 2D costmaps. |

---

## The TF Chain

```
map
 └── odom                    (published by AMCL or global EKF)
      └── base_link          (published by wheel odom or local EKF)
           ├── base_footprint
           ├── laser_frame    (published by robot_state_publisher from URDF)
           ├── imu_link       (published by robot_state_publisher from URDF)
           ├── camera_link    (published by robot_state_publisher from URDF)
           └── ...other sensor frames
```

### Who Publishes What

| Transform | Publisher | Notes |
|-----------|----------|-------|
| `map → odom` | AMCL, or global EKF (`world_frame: "map"`) | Corrects accumulated drift. May jump. |
| `odom → base_link` | robot_localization EKF, or raw wheel odometry node | Smooth, continuous. Never jumps. |
| `base_link → sensor_frames` | `robot_state_publisher` | Static transforms from URDF joints |
| `base_link → base_footprint` | `robot_state_publisher` | Usually identity or z-offset only |

---

## The Critical Rule: One Publisher Per Transform

**Only ONE node may publish each transform.** If two nodes both publish `odom → base_link`, TF2 receives conflicting data and downstream consumers (Nav2, costmaps) will see oscillation, warnings, or crashes.

### Common Violation

Your wheel odometry node publishes `odom → base_link` via `tf_broadcaster`. You also run robot_localization EKF with `publish_tf: true`, which also publishes `odom → base_link`.

**Fix**: Set `publish_tf: false` in the raw odometry publisher, or don't broadcast TF from the raw odometry node at all. Let the EKF be the sole publisher.

```python
# In your odometry node:
# self.tf_broadcaster.sendTransform(t)  # REMOVE THIS if using robot_localization
```

Or in the EKF config:
```yaml
publish_tf: true   # EKF publishes odom → base_link
```

And in the raw odometry node:
```yaml
publish_tf: false   # Raw odom does NOT publish TF
```

---

## REP 103 Coordinate Conventions

All frames follow right-hand rule:
- **X** → forward
- **Y** → left
- **Z** → up

Rotations:
- **Roll** → rotation about X (tilt left/right)
- **Pitch** → rotation about Y (tilt forward/backward)
- **Yaw** → rotation about Z (turn left/right)

Positive yaw is counterclockwise when viewed from above.

### Sensor Frame Alignment

Every sensor frame in your URDF must conform. If a physical sensor has a different convention (e.g., camera: z-forward, x-right, y-down), the URDF joint must include an appropriate rotation:

```xml
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0.0 0.3" rpy="0 0 0"/>
</joint>
<!-- camera_optical_frame follows camera convention: z-forward -->
<joint name="camera_optical_joint" type="fixed">
  <parent link="camera_link"/>
  <child link="camera_optical_frame"/>
  <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
</joint>
```

---

## Verifying the TF Tree

### List All Transforms

```bash
ros2 run tf2_tools view_frames
```

Generates a PDF of the complete TF tree with publishers and rates.

### Check a Specific Transform

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Shows the live transform between two frames. If this fails with "Could not transform", the chain is broken somewhere.

### Monitor for Problems

```bash
ros2 run tf2_ros tf2_monitor
```

Reports frequency and delay for all published transforms. Look for:
- **Transforms at 0 Hz** — broken publisher
- **High delay** — timestamps too far from current time
- **Multiple publishers** — conflicting sources

---

## map → odom: The Localization Correction

AMCL (or a global EKF) publishes `map → odom`. This transform encodes the accumulated drift correction:

```
true_robot_pose_in_map = map→odom * odom→base_link
```

When AMCL updates, `map → odom` may change discontinuously (the correction "jumps"). This is expected and correct — it means the odom frame has drifted and the map→odom transform compensates.

### Without AMCL

If you only run the local EKF (no AMCL), the `map → odom` transform is never published. Nav2 requires it. Options:
1. Run AMCL (recommended)
2. Publish a static identity transform: `map → odom` (works if odom drift is negligible for your use case)

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
)
```

---

## Launch File: Complete Frame Publishers

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # URDF → sensor frames
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
        ),

        # EKF → odom → base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config],  # publish_tf: true
        ),

        # AMCL → map → odom
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[amcl_config],
        ),
    ])
```

---

## Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| "Could not transform map to base_link" | AMCL not running or not yet localized | Start AMCL, set initial pose |
| Robot oscillates between two positions in RViz | Two nodes publishing the same transform | Check `ros2 run tf2_ros tf2_monitor`, disable duplicate publisher |
| Costmap doesn't align with walls | map→odom incorrect (bad localization) | Re-initialize AMCL, check laser scan alignment |
| "Extrapolation into the future" warning | Transform timestamps ahead of current time | Check `transform_tolerance` in AMCL and EKF |
| Sensor data appears offset in RViz | Sensor frame in URDF has wrong origin or orientation | Verify joint in URDF with `ros2 run tf2_ros tf2_echo base_link laser_frame` |
