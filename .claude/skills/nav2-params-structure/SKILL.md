<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: nav2-params-structure
description: 'Structure Nav2 YAML parameter files. Use when organizing navigation.yaml, understanding double-namespace pattern, or setting ROS parameters.'
---

# Nav2 YAML Parameter Structure

## The Double-Namespace Pattern

Nav2 parameters use a distinctive double-namespace pattern that confuses many users:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      rolling_window: true
```

### Why the Double Namespace?

Costmap nodes are created as **child nodes** of their parent server. The local costmap is a child node named `local_costmap` under the controller server's namespace. In ROS 2, a node's fully qualified name determines its parameter namespace:

- `controller_server` → parameters go under `controller_server: ros__parameters:`
- `local_costmap` (child of controller_server, but its own node) → `local_costmap: local_costmap: ros__parameters:`

The OUTER `local_costmap:` is the node name. The INNER `local_costmap:` is because the costmap creates itself with that name, leading to the doubled path. This is NOT a typo—it's a consequence of how ROS 2 child nodes inherit namespaces.

Similarly for global costmap:
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      robot_base_frame: base_link
```

## Full Parameter Hierarchy Example

```yaml
# Top-level: each key is a node name
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    default_nav_to_pose_bt_xml: ""  # Empty = use built-in default
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      # ... more BT node plugins

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      # MPPI params nested under FollowPath

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        # ... layer params
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      # ...
```

## RewrittenYaml in Launch Files

`RewrittenYaml` is a Nav2 utility that rewrites YAML parameters at launch time. This is how `use_sim_time` gets injected into every node:

```python
from nav2_common.launch import RewrittenYaml

configured_params = RewrittenYaml(
    source_file=params_file,
    root_key='',
    param_rewrites={
        'use_sim_time': use_sim_time,
        'default_nav_to_pose_bt_xml': bt_xml_file,
        'autostart': autostart,
    },
    convert_types=True  # CRITICAL: converts string "true" to bool True
)
```

`convert_types=True` is essential—launch arguments are strings, but ROS 2 parameters need proper types. Without this, `use_sim_time: "true"` (a string) won't work.

The `root_key` parameter prepends a namespace. If set to `''`, rewrites apply to ALL nodes in the YAML. If set to a node name, only that node's parameters are rewritten.

## Declaring vs Loading Parameters

Parameters in YAML are only loaded if the node **declares** them:

```cpp
// In node code - parameter must be declared to be loaded from YAML
declare_parameter("controller_frequency", 20.0);  // with default
declare_parameter("controller_frequency", rclcpp::PARAMETER_NOT_SET);  // no default

// Or allow undeclared parameters (Nav2 plugins do this)
auto options = rclcpp::NodeOptions();
options.allow_undeclared_parameters(true);
options.automatically_declare_parameters_from_overrides(true);
```

Most Nav2 servers use `automatically_declare_parameters_from_overrides(true)`, meaning ANY parameter in the YAML file under the correct namespace will be loaded.

## Common Mistakes

### Wrong Indentation
```yaml
# WRONG - controller params won't be found
controller_server:
ros__parameters:    # Must be indented under controller_server
  controller_frequency: 20.0

# CORRECT
controller_server:
  ros__parameters:
    controller_frequency: 20.0
```

### Missing ros__parameters
```yaml
# WRONG - not a valid ROS 2 parameter format
controller_server:
  controller_frequency: 20.0

# CORRECT
controller_server:
  ros__parameters:
    controller_frequency: 20.0
```

### Namespace Mismatch
If you remap a node name in the launch file but don't update the YAML, parameters won't load:
```python
# Launch file remaps node to 'my_controller'
Node(package='nav2_controller', executable='controller_server',
     name='my_controller', ...)

# YAML must match the remapped name
my_controller:          # NOT controller_server
  ros__parameters: ...
```

### Plugin Name Must Match YAML Key
```yaml
controller_plugins: ["FollowPath"]   # This name...
FollowPath:                          # ...must match this key
  plugin: "nav2_mppi_controller::MPPIController"
```

### use_sim_time Not Propagating
If a node doesn't respect `use_sim_time`, check:
1. `RewrittenYaml` has `convert_types=True`
2. The node actually declares `use_sim_time` (lifecycle nodes do by default)
3. The parameter is under the correct node namespace in YAML
