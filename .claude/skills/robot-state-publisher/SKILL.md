<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: robot-state-publisher
description: 'Configure robot_state_publisher. Use when publishing TF from URDF, launching with xacro, or using joint_state_publisher for testing.'
---

# robot_state_publisher and joint_state_publisher

## robot_state_publisher

`robot_state_publisher` is the bridge between the URDF and the TF tree. It:

1. Reads the URDF from the `robot_description` parameter
2. Publishes **static transforms** for all fixed joints on `/tf_static`
3. Subscribes to `/joint_states` and publishes **dynamic transforms** for movable joints on `/tf`
4. Publishes the URDF on the `/robot_description` topic (used by RViz, Nav2, MoveIt)

### Launch File Configuration

```python
import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare('my_robot_description'), 'urdf', 'my_robot.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
    ])
```

### With Xacro Arguments

Pass arguments through to xacro for conditional URDF generation:

```python
from launch.substitutions import LaunchConfiguration

use_sim_time = LaunchConfiguration('use_sim_time', default='false')

robot_description = ParameterValue(
    Command([
        'xacro ', xacro_file,
        ' use_sim:=', use_sim_time,
        ' use_lidar:=true',
    ]),
    value_type=str
)
```

### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/tf` | `tf2_msgs/TFMessage` | Dynamic transforms from movable joints |
| `/tf_static` | `tf2_msgs/TFMessage` | Static transforms from fixed joints (latched) |
| `/robot_description` | `std_msgs/String` | The full URDF XML string |

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `robot_description` | string | required | The URDF XML content |
| `publish_frequency` | double | 20.0 | TF publish rate (Hz) |
| `ignore_timestamp` | bool | false | If true, use latest joint_states regardless of timestamp |
| `frame_prefix` | string | "" | Prefix for all frame names (multi-robot) |

## joint_state_publisher

`joint_state_publisher` publishes default (zero) joint states for joints not driven by hardware or simulation. It:

- Reads the URDF from `/robot_description`
- Finds all non-fixed joints
- Publishes them at zero position (or configured default) on `/joint_states`
- Merges with any joint states received from other sources on `source_list` topics

### When To Use

| Scenario | Use joint_state_publisher? |
|---|---|
| Visualization testing in RViz only | Yes |
| Simulation (Gazebo publishes joint states) | No |
| Real robot (motor drivers publish joint states) | No, unless passive joints exist |
| Robot with passive joints + driven joints | Yes, with `source_list` |

### Launch Configuration

```python
joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    parameters=[{
        'source_list': ['/motor_driver/joint_states'],
        'rate': 30,
    }],
)
```

The `source_list` parameter tells joint_state_publisher to merge its default states with states from other publishers. Joints reported by source topics override the defaults.

## joint_state_publisher_gui

Interactive sliders for testing URDF joint articulation:

```python
joint_state_publisher_gui = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
)
```

```bash
sudo apt install ros-jazzy-joint-state-publisher-gui
```

Use this **only for development/testing**—never in production. It opens a GUI window with sliders for each non-fixed joint.

## Differential Drive: Who Publishes What

For a differential drive robot:

```
Joint States Flow:
┌─────────────────┐     /joint_states     ┌──────────────────────┐
│  Motor Driver    │ ──────────────────▶   │ robot_state_publisher │ ──▶ /tf
│  (or Gazebo)     │  wheel positions      │                      │ ──▶ /tf_static
└─────────────────┘                        └──────────────────────┘
                                                     ▲
                                                     │ robot_description
                                                     │ (from parameter)
```

- **Wheel joints** (`left_wheel_joint`, `right_wheel_joint`): published by the motor driver node or Gazebo diff-drive plugin
- **Caster joints** (if continuous): either left at default or included in hardware joint states
- **Fixed joints**: handled automatically by robot_state_publisher (no joint_states needed)
- **Passive joints** (pan/tilt not driven): use joint_state_publisher if no other source

## Multi-Robot Setup

For multiple robots in the same ROS 2 network, use namespaces and frame prefixes:

```python
robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    namespace='robot1',
    parameters=[{
        'robot_description': robot_description,
        'frame_prefix': 'robot1/',
    }],
    remappings=[
        ('/joint_states', '/robot1/joint_states'),
    ],
)
```

This produces frames like `robot1/base_link`, `robot1/left_wheel_link`, etc. The Nav2 stack and other nodes must be configured with the matching prefix.

## Complete Launch Example

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('my_robot_description'), 'urdf', 'my_robot.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=', use_sim_time]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_gui', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        # Only for visualization testing
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),
    ])
```

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| "No transform from base_link to wheel" | No joint_states for wheel joints | Check motor driver or Gazebo plugin publishing `/joint_states` |
| Frames appear at origin in RViz | robot_description parameter empty or malformed | Check xacro processing: `xacro file.xacro \| check_urdf /dev/stdin` |
| Stale TF warnings | joint_states publish rate too low or timestamps wrong | Increase rate; check `use_sim_time` consistency |
| Duplicate frames in TF tree | Multiple robot_state_publishers without frame_prefix | Add unique `frame_prefix` per robot |
| "Robot model not found" in RViz | `/robot_description` topic not published | Ensure robot_state_publisher is running |

```bash
# Debug: check what robot_state_publisher sees
ros2 param get /robot_state_publisher robot_description

# Debug: check TF tree
ros2 run tf2_tools view_frames

# Debug: check joint_states topic
ros2 topic echo /joint_states --once
```
