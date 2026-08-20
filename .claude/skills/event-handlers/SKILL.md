<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: event-handlers
description: 'Use ROS 2 launch event handlers. Use when reacting to process start, exit, or shutdown events in launch files.'
---

# Role
You are an expert in ROS 2 launch event handlers. You guide correct usage of process monitoring, conditional actions, cleanup, and dynamic launch logic in ROS 2 Jazzy/Rolling.

## Event Handler Overview
Launch event handlers let you react to events during the launch lifecycle: process start/exit, shutdown signals, timer events, and custom events. They enable conditional restart, cleanup, graceful degradation, and dynamic launch configuration.

## OnProcessStart
```python
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart

RegisterEventHandler(
    OnProcessStart(
        target_action=my_node,
        on_start=[
            LogInfo(msg='Node has started!'),
            # Launch dependent nodes after this one starts
            other_node,
        ]
    )
)
```

## OnProcessExit
```python
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler, LogInfo, EmitEvent, Shutdown

# Log and clean up when a node exits
RegisterEventHandler(
    OnProcessExit(
        target_action=driver_node,
        on_exit=[
            LogInfo(msg=['Driver exited with code: ', launch.substitutions.LaunchConfiguration('__exit_code')]),
            # Shut down the entire launch if driver dies
            EmitEvent(event=Shutdown(reason='Driver node died'))
        ]
    )
)
```

## OnProcessIO (Process Output Monitoring)
```python
from launch.event_handlers import OnProcessIO

RegisterEventHandler(
    OnProcessIO(
        target_action=my_node,
        on_stdout=lambda event: LogInfo(
            msg=f'STDOUT: {event.text.decode()}'
        ),
        on_stderr=lambda event: LogInfo(
            msg=f'STDERR: {event.text.decode()}'
        ),
    )
)
```

## Conditional Restart on Exit
```python
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

def restart_node(event, context):
    """Restart a node after a delay if it exits unexpectedly."""
    exit_code = event.returncode
    if exit_code != 0:
        return [
            LogInfo(msg=f'Node crashed (code {exit_code}), restarting in 5s...'),
            TimerAction(
                period=5.0,
                actions=[driver_node],  # relaunch the node
            )
        ]
    return [LogInfo(msg='Node exited cleanly, not restarting.')]

RegisterEventHandler(
    OnProcessExit(
        target_action=driver_node,
        on_exit=restart_node,
    )
)
```

## TimerAction
```python
from launch.actions import TimerAction

# Delay an action by N seconds
delayed_start = TimerAction(
    period=10.0,
    actions=[
        Node(package='my_pkg', executable='delayed_node'),
    ]
)
```

## EmitEvent and Custom Events
```python
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnShutdown

# React to launch shutdown
RegisterEventHandler(
    OnShutdown(
        on_shutdown=[
            LogInfo(msg='Launch is shutting down!'),
            # Run cleanup commands
            ExecuteProcess(cmd=['pkill', '-f', 'my_background_process']),
        ]
    )
)

# Programmatically trigger shutdown
EmitEvent(event=Shutdown(reason='Task completed'))
```

## OpaqueFunction for Dynamic Logic
```python
from launch.actions import OpaqueFunction

def dynamic_setup(context, *args, **kwargs):
    """Generate launch actions based on runtime conditions."""
    robot_type = context.launch_configurations.get('robot_type', 'differential')

    nodes = []
    if robot_type == 'differential':
        nodes.append(Node(
            package='diff_drive', executable='controller',
            parameters=[{'wheel_radius': 0.05}]
        ))
    elif robot_type == 'ackermann':
        nodes.append(Node(
            package='ackermann_drive', executable='controller',
            parameters=[{'wheelbase': 0.3}]
        ))

    # Can also read files, check environment, etc.
    import os
    config_path = os.path.join(
        get_package_share_directory('my_pkg'), 'config',
        f'{robot_type}_params.yaml'
    )
    if os.path.exists(config_path):
        nodes[-1]._Node__parameters.append(config_path)

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='differential'),
        OpaqueFunction(function=dynamic_setup),
    ])
```

## Ordered Startup with Event Chains
```python
# Start nodes in sequence: driver → processor → controller
driver = Node(package='my_pkg', executable='driver')
processor = Node(package='my_pkg', executable='processor')
controller = Node(package='my_pkg', executable='controller')

def generate_launch_description():
    return LaunchDescription([
        driver,
        RegisterEventHandler(
            OnProcessStart(
                target_action=driver,
                on_start=[
                    LogInfo(msg='Driver started, launching processor...'),
                    processor,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=processor,
                on_start=[
                    LogInfo(msg='Processor started, launching controller...'),
                    controller,
                ]
            )
        ),
    ])
```

## Cleanup on Exit Pattern
```python
cleanup_cmd = ExecuteProcess(
    cmd=['bash', '-c', 'echo "Cleaning up..." && rm -f /tmp/robot.lock'],
    output='screen',
)

RegisterEventHandler(
    OnProcessExit(
        target_action=main_node,
        on_exit=[cleanup_cmd],
    )
)
```

## ExecuteProcess Event Handlers
```python
from launch.actions import ExecuteProcess
from launch.event_handlers import OnExecutionComplete

# For non-ROS processes
background_process = ExecuteProcess(
    cmd=['python3', 'data_logger.py'],
    output='screen',
)

RegisterEventHandler(
    OnExecutionComplete(
        target_action=background_process,
        on_completion=[LogInfo(msg='Background process finished')],
    )
)
```

## Lifecycle Node Events
```python
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg

# Trigger configure when lifecycle node starts
RegisterEventHandler(
    OnProcessStart(
        target_action=lifecycle_node,
        on_start=[
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda node: node == lifecycle_node,
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            ))
        ]
    )
)
```

## Critical Warnings
- **Event handler registration order**: Register event handlers BEFORE the actions they monitor. If the action starts before the handler is registered, events may be missed.
- **OnProcessStart is not "node ready"**: `OnProcessStart` fires when the OS process starts, NOT when the ROS node is fully initialized. There may be a delay before topics/services are available. Use timers or service waits for robust sequencing.
- **Shutdown cascades**: `EmitEvent(Shutdown())` stops ALL nodes in the launch. Use sparingly. For individual node failure handling, restart the specific node instead.
- **OpaqueFunction limitations**: Code in `OpaqueFunction` runs during launch description evaluation, not during execution. You cannot react to runtime events inside it.
- **TimerAction and sim_time**: `TimerAction` uses wall clock time, not sim time. A 5-second timer always waits 5 real seconds regardless of simulation speed.
- **Event handler memory**: Event handlers hold references to actions. Be careful about creating circular references or holding large objects in closures.
- **Process exit codes**: A node killed by SIGINT (Ctrl+C) typically returns code 0 or -2. A crash returns non-zero. Check exit codes carefully in restart logic to avoid infinite restart loops.
