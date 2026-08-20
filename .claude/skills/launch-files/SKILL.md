<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: launch-files
description: 'Write ROS 2 Python launch files. Use when creating launch descriptions, composing nodes, passing parameters, or including other launch files.'
---

# Role
You are an expert in ROS 2 Python launch files. You produce correct, well-structured launch files using the launch and launch_ros APIs for ROS 2 Jazzy/Rolling.

## Basic Launch File Structure
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, PushRosNamespace, SetParameter

def generate_launch_description():
    pkg_share = get_package_share_directory('my_package')

    # Declare arguments (available via CLI: key:=value)
    use_sim_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace'
    )

    # LaunchConfiguration references the argument VALUE (always a string/substitution)
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Node with parameter file + overrides
    my_node = Node(
        package='my_package',
        executable='my_node',
        name='my_node',
        namespace=namespace,
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'params.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('input', '/sensor/data'),
            ('output', '/processed/data')
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        condition=IfCondition(use_sim_time)  # only launch if use_sim_time=true
    )

    return LaunchDescription([
        use_sim_arg,
        namespace_arg,
        my_node,
    ])
```

## Including Other Launch Files
```python
included = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('other_pkg'), 'launch', 'other.launch.py')
    ),
    launch_arguments={
        'use_sim_time': use_sim_time,
        'robot_name': 'my_robot',
    }.items()
)
```

## GroupAction for Namespacing
```python
grouped = GroupAction([
    PushRosNamespace(namespace),
    SetParameter('use_sim_time', use_sim_time),
    Node(package='pkg_a', executable='node_a', name='node_a'),
    Node(package='pkg_b', executable='node_b', name='node_b'),
])
```

## OpaqueFunction for Dynamic Logic
```python
from launch.actions import OpaqueFunction

def launch_setup(context):
    # context.launch_configurations gives you resolved string values
    ns = context.launch_configurations.get('namespace', '')
    use_sim = context.launch_configurations['use_sim_time']

    nodes = []
    if use_sim == 'true':
        nodes.append(Node(package='sim_pkg', executable='sim_node'))
    else:
        nodes.append(Node(package='real_pkg', executable='real_node'))
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
```

## Conditions
```python
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals

# Boolean condition (string 'true'/'false')
condition=IfCondition(LaunchConfiguration('enable_rviz'))

# Negation
condition=UnlessCondition(LaunchConfiguration('headless'))

# Equality check
condition=LaunchConfigurationEquals('mode', 'mapping')
```

## Parameter YAML Files
```yaml
# config/params.yaml — node name must match
my_node:
  ros__parameters:
    rate: 10.0
    frame_id: "base_link"
    thresholds: [0.1, 0.5, 1.0]
    nested:
      key: value

# Wildcard: applies to any node name
/**:
  ros__parameters:
    use_sim_time: true
```

## Remappings
```python
Node(
    package='my_pkg', executable='my_node',
    remappings=[
        ('/cmd_vel', '/robot/cmd_vel'),
        ('scan', '/lidar/scan'),
    ]
)
```

## SetEnvironmentVariable, SetParameter
```python
from launch.actions import SetEnvironmentVariable
SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

# Global parameter for all nodes in this launch
SetParameter('use_sim_time', 'true')
```

## Event Handlers in Launch
```python
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart

RegisterEventHandler(
    OnProcessExit(
        target_action=my_node,
        on_exit=[LogInfo(msg='Node exited!')]
    )
)
```

## Critical Warnings
- **LaunchConfiguration is NOT a Python type**: `LaunchConfiguration('use_sim_time')` returns a substitution object, not a bool or string. You CANNOT use it in Python `if` statements. Use `OpaqueFunction` + `context.launch_configurations` to get resolved string values.
- **Parameter types from CLI**: All CLI arguments (`key:=value`) are strings. Use `{'param': LaunchConfiguration('arg')}` — ROS 2 will parse the string to the declared parameter type. But YAML booleans (`true`/`false`) must be lowercase strings.
- **Declare before use**: `DeclareLaunchArgument` must appear in the LaunchDescription BEFORE any action that references that `LaunchConfiguration`. Otherwise, you get an unresolved substitution error.
- **setup.py data_files**: Launch files must be installed via `data_files` in `setup.py` or `install(DIRECTORY ...)` in CMakeLists.txt, or they won't be found by `get_package_share_directory`.
- **Namespace + node name**: `PushRosNamespace` affects all nodes in the group. This changes topic resolution, parameter namespacing, and service names.
- **output='screen'**: Without this, node output goes to log files only. Always use `'screen'` during development.

## install in CMakeLists.txt
```cmake
install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME})
```

## install in setup.py (Python packages)
```python
data_files=[
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/config', glob('config/*')),
],
```
