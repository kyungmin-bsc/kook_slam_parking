<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: nav2-rviz-tools
description: 'Use Nav2 RViz tools and panels. Use when setting initial pose, sending goals, or visualizing costmaps and paths in RViz2.'
---

# Nav2 RViz2 Tools and Visualization

## Nav2 RViz Panel

The `nav2_rviz_plugins` package adds a dedicated panel to RViz2 for controlling Nav2. Add it via: **Panels → Add New Panel → nav2_rviz_plugins/Nav2 Panel**.

### Panel Features

- **Startup / Shutdown**: Buttons to transition all Nav2 lifecycle nodes (calls lifecycle managers)
- **Navigation**: Set and send 2D Nav Goals interactively
- **Waypoint mode**: Toggle waypoint/single-goal mode. In waypoint mode, each "2D Nav Goal" click adds to a queue. Click "Start Waypoint Following" to execute.
- **Acummulated waypoints**: View list of queued waypoints before execution
- **Cancel Navigation**: Abort current goal
- **Lifecycle indicators**: Green/red status for each managed node

## Setting Initial Pose (2D Pose Estimate)

Click the **2D Pose Estimate** button in the RViz toolbar, then click and drag on the map to set position and orientation. This publishes to `/initialpose` topic, which AMCL uses to initialize the particle filter.

**Critical**: The initial pose must be reasonably close to the robot's actual position for AMCL to converge. If AMCL doesn't converge, the robot's position will jump as particles reconverge.

After setting, watch the AMCL particle cloud (`/particlecloud` topic) converge. A tight cluster means good localization.

## Setting Nav2 Goals

### Single Goal
Click **Nav2 Goal** in the toolbar, then click and drag on the map. The drag direction sets the goal orientation. This sends a `NavigateToPose` action.

### Waypoint Mode
1. Check "Waypoint mode" in the Nav2 panel
2. Click **Nav2 Goal** multiple times to place waypoints
3. Click "Start Waypoint Following" in the panel
4. Or "Start Navigation Through Poses" for smooth traversal

## Essential Display Configuration

### Map Display
```
Topic: /map
Color Scheme: map (grey = unknown, white = free, black = occupied)
```
If map doesn't appear: check QoS. The map server publishes with **transient local** durability. Set:
```
Reliability: Reliable
Durability: Transient Local
```

### Global Costmap
```
Topic: /global_costmap/costmap
Color Scheme: costmap (useful for seeing inflation layers)
```

### Local Costmap
```
Topic: /local_costmap/costmap
Color Scheme: costmap
```

Display the local costmap to verify the rolling window is correctly tracking obstacles around the robot.

### Global Plan
```
Type: Path
Topic: /plan
Line Style: Lines
Color: green (or preference)
```

### Local Plan (Controller Trajectory)
```
Type: Path
Topic: /local_plan
Line Style: Lines
Color: blue
```

### Robot Footprint
```
Type: Polygon
Topic: /local_costmap/published_footprint
Color: red
```

### AMCL Particle Cloud
```
Type: PoseArray
Topic: /particlecloud
```
Shows localization confidence. A tight cluster = good localization. Spread-out particles = poor localization.

### Laser Scan Overlay
```
Type: LaserScan
Topic: /scan
Color Transformer: FlatColor or Intensity
Size: 0.03
```

Overlaying the laser scan on the map helps verify that AMCL localization is correct—scan points should align with map walls.

### TF Display
```
Type: TF
Show Names: true
Show Axes: true
Frames to show: map, odom, base_link, base_footprint
```

Essential for debugging transform issues. The `map → odom` transform comes from AMCL. The `odom → base_link` transform comes from odometry. If either is missing, navigation will fail.

## Useful Additional Displays

### PointCloud2 (Depth Camera)
```
Type: PointCloud2
Topic: /camera/depth/points
Color Transformer: AxisColor (Z axis = height)
Size: 0.01
```

### Robot Model
```
Type: RobotModel
Description Source: Topic
Description Topic: /robot_description
```

### Marker Arrays (for debugging)
```
Type: MarkerArray
Topic: /marker (or custom topics from your nodes)
```

### Collision Monitor Polygons
```
Type: Polygon
Topic: /polygon_stop        (stop zone)
Topic: /polygon_slowdown    (slowdown zone)
Topic: /polygon_approach    (approach zone)
```

Visualizing collision monitor zones helps verify they're correctly sized and positioned.

## Recommended RViz Configuration

A production Nav2 RViz config should include at minimum:

```yaml
Displays:
  - Class: rviz_default_plugins/Map
    Topic: /map
  - Class: rviz_default_plugins/Map
    Topic: /global_costmap/costmap
    Color Scheme: costmap
  - Class: rviz_default_plugins/Map
    Topic: /local_costmap/costmap
    Color Scheme: costmap
  - Class: rviz_default_plugins/Path
    Topic: /plan
  - Class: rviz_default_plugins/Path
    Topic: /local_plan
  - Class: rviz_default_plugins/LaserScan
    Topic: /scan
  - Class: rviz_default_plugins/TF
  - Class: rviz_default_plugins/RobotModel
  - Class: rviz_default_plugins/Polygon
    Topic: /local_costmap/published_footprint
```

Save as `.rviz` config file and load via:
```bash
rviz2 -d my_nav_config.rviz
```

Or in launch file:
```python
Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_path],
)
```

## Common Issues

### "No map received" Warning
**Cause**: QoS mismatch between map server and RViz map display.
**Fix**: In the Map display properties, set:
- Reliability: **Reliable**
- Durability: **Transient Local**

The map server publishes with transient local QoS so late subscribers still receive the map. If RViz uses volatile durability (default), it misses the map.

### Costmap Not Updating
**Cause**: The costmap topic has a different QoS than RViz expects.
**Fix**: Check the costmap display QoS settings. Also verify the costmap node is in the `active` lifecycle state.

### TF "No transform from [map] to [base_link]"
**Cause**: Either AMCL (map→odom) or odometry (odom→base_link) is not publishing.
**Fix**: Check `ros2 run tf2_tools view_frames` to see the TF tree. Ensure both localization and odometry nodes are running.

### Path Display Shows Old Path
The `/plan` topic updates each time a new path is computed. If the display appears stale, check that replanning is happening (the `RateController` in the BT controls replanning frequency).

### RViz Crashes on Launch
Often caused by bad config files or GPU driver issues. Try:
```bash
rviz2 --ros-args -p use_sim_time:=true  # Without config file
```

### Saving Poses from RViz for Waypoints
Use the "Publish Point" tool (click on map) to get coordinates printed to `/clicked_point` topic:
```bash
ros2 topic echo /clicked_point
```

Record coordinates and use them in your patrol waypoint definitions.
