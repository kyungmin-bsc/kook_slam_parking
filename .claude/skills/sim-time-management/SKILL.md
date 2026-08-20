<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: sim-time-management
description: 'Manage simulation time in ROS 2. Use when setting use_sim_time, debugging clock desync, or configuring /clock topic bridging.'
---

# Simulation Time Management in ROS 2

## The /clock Topic

In simulation, time does not come from the system wall clock. Gazebo publishes a `/clock` topic via the `ros_gz_bridge`, and every ROS 2 node must use it. This is controlled by the `use_sim_time` parameter.

When `use_sim_time: true`:
- `node.get_clock().now()` returns simulation time from `/clock`
- TF timestamps use simulation time
- Message headers use simulation time
- Timers fire based on simulation time progression

When `use_sim_time: false` (default):
- System wall clock is used
- This is correct for real robot operation

## The Critical Bug

If **any** node uses wall time while others use sim time, you get:
- TF extrapolation errors: `"Could not find a connection between 'map' and 'base_link' because they are not part of the same tree"`
- `Lookup would require extrapolation X.XXs into the future`
- Stale sensor data warnings
- AMCL divergence
- Costmap not updating
- Controller timing mismatches

**Every single node** that touches TF, publishes messages, or subscribes to timestamped data must have `use_sim_time: true`.

## Setting use_sim_time in Launch Files

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': ['-r ', PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'), 'worlds', 'house.sdf'
            ])],
        }.items(),
    )

    # Bridge (publishes /clock among other topics)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': '...',  # Load from xacro
        }],
    )

    # Spawn robot into Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-x', '1.0', '-y', '1.0', '-z', '0.1',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # EKF (robot_localization)
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            PathJoinSubstitution([FindPackageShare('my_robot_bringup'), 'config', 'ekf.yaml']),
            {'use_sim_time': use_sim_time},  # Override YAML value
        ],
    )

    # Nav2 — use_sim_time must be set in BOTH the launch params AND the YAML
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('my_robot_bringup'), '/launch/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'), 'config', 'navigation.yaml'
            ]),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gz_sim,
        bridge,
        robot_state_publisher,
        spawn,
        ekf,
        nav2,
    ])
```

## YAML Configuration

In `navigation.yaml`, `use_sim_time` must appear for **each** node:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true

controller_server:
  ros__parameters:
    use_sim_time: true

planner_server:
  ros__parameters:
    use_sim_time: true

behavior_server:
  ros__parameters:
    use_sim_time: true

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true

map_server:
  ros__parameters:
    use_sim_time: true

amcl:
  ros__parameters:
    use_sim_time: true

lifecycle_manager:
  ros__parameters:
    use_sim_time: true
```

Missing `use_sim_time` in even one node causes cascading TF failures.

## Real-Time Factor

Gazebo's real-time factor (RTF) indicates simulation speed relative to wall clock:

| RTF | Meaning |
|-----|---------|
| 1.0 | Real-time: 1 sim second = 1 wall second |
| 0.5 | Half speed: sim runs at 50% of real-time |
| 2.0 | Double speed: sim runs 2× faster than real-time |
| 0.0 | Paused |

Check RTF: `gz stats` or look at the Gazebo GUI bottom bar.

Low RTF causes: complex meshes, many models, high sensor rates, insufficient GPU for rendering sensors. Solutions: simplify collision geometry, reduce sensor update rates, use primitive shapes, lower physics step rate.

## Pausing and Stepping

When Gazebo is paused:
- `/clock` stops publishing new values
- All ROS 2 nodes with `use_sim_time: true` freeze—timers don't fire, transforms appear stale
- This is **correct behavior**—the system waits for time to advance

Step mode (advance one physics step):
```bash
gz service -s /world/house_patrol/control --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 1000 \
  --req 'multi_step: {num_steps: 1}'
```

Useful for debugging: step one frame, inspect state, step again.

## Why Sim Time Matters for Navigation

- **AMCL**: Particle filter resampling is time-based. Wrong clock → particles diverge.
- **Costmap**: Update rates tied to time. Stale time → stale costmap → crashes into obstacles.
- **Controller**: DWB/MPPI compute velocity commands based on time deltas. Wrong dt → wrong velocities.
- **Recovery behaviors**: Timeout-based. Wall clock timeouts fire immediately if sim is slow.
- **TF**: Transform timeout defaults (e.g., 0.2s) are in sim time. Mixing clocks breaks lookups.

## Debugging Time Issues

```bash
# Check if /clock is being published
ros2 topic hz /clock

# Check a node's use_sim_time setting
ros2 param get /controller_server use_sim_time

# Check all nodes at once
for node in $(ros2 node list); do
  echo "$node: $(ros2 param get $node use_sim_time 2>/dev/null || echo 'N/A')"
done

# Check TF for extrapolation errors
ros2 run tf2_ros tf2_monitor
```
