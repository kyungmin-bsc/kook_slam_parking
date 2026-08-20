<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: rviz2-config
description: 'Configure RViz2 visualization. Use when setting up displays, camera views, QoS overrides, or saving .rviz config files.'
---

# RViz2 Configuration for Robot Visualization

## Launching

```bash
# Launch with a config file
rviz2 -d config/navigation.rviz

# Launch with default empty config
rviz2
```

## Config File Structure

RViz2 configs are YAML files (`.rviz`) with three main sections: `Visualization Manager` (displays, global options), `Views` (camera settings), and `Window Geometry`.

## Fixed Frame Selection

| Use Case | Fixed Frame | Why |
|----------|-------------|-----|
| Full navigation | `map` | See robot in world context |
| Odometry only (no map) | `odom` | Stable reference without AMCL |
| Sensor debugging | `base_link` | Robot-centric, sensor data stays fixed |
| Single sensor | sensor frame (e.g. `laser_frame`) | Raw sensor alignment check |

## Key Display Types

| Display | Topic | Use |
|---------|-------|-----|
| RobotModel | `/robot_description` | Show URDF mesh |
| TF | `/tf`, `/tf_static` | Transform tree visualization |
| LaserScan | `/scan` | 2D LIDAR data |
| Map | `/map` | Static/SLAM map |
| Path | `/plan` | Nav2 global plan |
| Path | `/local_plan` | Nav2 local trajectory |
| Costmap (as Map) | `/global_costmap/costmap` | Obstacle layer |
| Costmap (as Map) | `/local_costmap/costmap` | Local planning area |
| MarkerArray | `/waypoints` | Custom markers |
| Image | `/camera/image_raw` | Camera feed |
| PointCloud2 | `/camera/depth/points` | 3D depth data |

## QoS Overrides

Sensor topics typically use Best Effort reliability. If a display shows "no messages received," change QoS:

- **LaserScan, PointCloud2, Image**: Reliability = Best Effort, Durability = Volatile
- **Map, Path**: Reliability = Reliable, Durability = Transient Local
- **TF**: Reliability = Reliable

## Camera Views

| View Type | Use Case | Settings |
|-----------|----------|----------|
| Orbit | 3D inspection | Click+drag to rotate |
| TopDownOrtho | 2D map view | Set Angle = 0, scale for zoom |
| ThirdPersonFollower | Follow robot in 3D | Target Frame = `base_link` |
| FPS (First Person) | Robot POV | Target Frame = `base_link` |

## Complete Navigation Config

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description File: ""
      Description Source: Topic
      Description Topic:
        Value: /robot_description
        Depth: 5
        Reliability Policy: Reliable
        Durability Policy: Transient Local
      Alpha: 1.0

    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Frame Timeout: 15
      Frames:
        All Enabled: false
        base_link:
          Value: true
        odom:
          Value: true
        map:
          Value: true

    - Class: rviz_default_plugins/LaserScan
      Name: LaserScan
      Enabled: true
      Topic:
        Value: /scan
        Depth: 5
        Reliability Policy: Best Effort
        Durability Policy: Volatile
      Size (m): 0.03
      Color Transformer: Intensity
      Style: Points
      Min Intensity: 0
      Max Intensity: 4096

    - Class: rviz_default_plugins/Map
      Name: Map
      Enabled: true
      Topic:
        Value: /map
        Depth: 1
        Reliability Policy: Reliable
        Durability Policy: Transient Local
      Alpha: 0.7
      Color Scheme: map
      Draw Behind: true

    - Class: rviz_default_plugins/Map
      Name: GlobalCostmap
      Enabled: true
      Topic:
        Value: /global_costmap/costmap
        Depth: 1
        Reliability Policy: Reliable
        Durability Policy: Transient Local
      Alpha: 0.3
      Color Scheme: costmap

    - Class: rviz_default_plugins/Map
      Name: LocalCostmap
      Enabled: true
      Topic:
        Value: /local_costmap/costmap
        Depth: 1
        Reliability Policy: Reliable
        Durability Policy: Transient Local
      Alpha: 0.5
      Color Scheme: costmap

    - Class: rviz_default_plugins/Path
      Name: GlobalPlan
      Enabled: true
      Topic:
        Value: /plan
        Depth: 5
        Reliability Policy: Reliable
      Color: 0; 255; 0
      Line Style: Lines
      Width: 0.03
      Alpha: 0.8

    - Class: rviz_default_plugins/Path
      Name: LocalPlan
      Enabled: true
      Topic:
        Value: /local_plan
        Depth: 5
        Reliability Policy: Reliable
      Color: 255; 255; 0
      Line Style: Lines
      Width: 0.05

    - Class: rviz_default_plugins/MarkerArray
      Name: Waypoints
      Enabled: true
      Topic:
        Value: /waypoints_marker
        Depth: 5
        Reliability Policy: Reliable

  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30

  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/SetInitialPose
      Topic: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic: /goal_pose

  Value: true
Views:
  Current:
    Class: rviz_default_plugins/ThirdPersonFollower
    Distance: 8.0
    Focal Point:
      X: 0
      Y: 0
      Z: 0
    Pitch: 0.5
    Target Frame: base_link
    Yaw: 3.14
```

## Per-Use-Case Configs

Maintain separate configs to avoid clutter:

```
config/
├── navigation.rviz    # Map + costmaps + plans + robot model
├── mapping.rviz       # SLAM in progress: map, scan, TF, no costmaps
├── debugging.rviz     # All TF frames, scan, full costmaps, markers
└── camera.rviz        # Image + PointCloud2 + depth overlays
```

## Saving and Sharing

Save config: **File → Save Config As**. Configs are portable across machines if topic names match. Launch from a ROS 2 launch file:

```python
from launch_ros.actions import Node

Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    arguments=["-d", LaunchConfiguration("rviz_config")],
    parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
)
```
