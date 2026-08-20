<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: custom-interfaces
description: 'Define custom ROS 2 messages, services, and actions. Use when creating .msg, .srv, .action files or an _interfaces package.'
---

# Role
You are an expert in defining custom ROS 2 interfaces (.msg, .srv, .action files). You guide correct interface design, package setup, and build configuration for ROS 2 Jazzy/Rolling.

## Package Structure
```
my_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── RobotStatus.msg
│   └── SensorReading.msg
├── srv/
│   └── SetMode.srv
└── action/
    └── Navigate.action
```

Interfaces MUST be in a dedicated `_interfaces` package (e.g., `my_robot_interfaces`). Do NOT put interfaces in the same package as node code — this causes circular dependency issues with Python packages.

## Message Definition (.msg)
```
# msg/RobotStatus.msg
# Header for timestamp and frame_id
std_msgs/Header header

# Builtin types
string robot_name
uint8 battery_percent       # 0-100
float64 velocity            # m/s
bool is_charging

# Constants
uint8 MODE_IDLE=0
uint8 MODE_PATROL=1
uint8 MODE_CHARGE=2
uint8 current_mode

# Arrays
float64[] joint_positions           # dynamic array
float64[6] joint_efforts            # fixed-size array
string[] active_behaviors

# Nested message
geometry_msgs/Pose current_pose

# Bounded sequences (Jazzy+)
# sequence<float64, 100> bounded_data  # max 100 elements
```

## Service Definition (.srv)
```
# srv/SetMode.srv
# Request
uint8 desired_mode
string reason
---
# Response
bool success
string message
```

## Action Definition (.action)
```
# action/Navigate.action
# Goal
geometry_msgs/PoseStamped target_pose
float64 speed
---
# Result
bool success
float64 total_distance
float64 total_time
---
# Feedback
geometry_msgs/PoseStamped current_pose
float64 distance_remaining
float64 estimated_time_remaining
```

## CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/SensorReading.msg"
  "srv/SetMode.srv"
  "action/Navigate.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
```

## package.xml
```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_interfaces</name>
  <version>0.1.0</version>
  <description>Custom interfaces</description>
  <maintainer email="dev@example.com">dev</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

## Naming Conventions
- **Message/Service/Action names**: CamelCase — `RobotStatus`, `SetMode`, `Navigate`
- **Field names**: snake_case — `battery_percent`, `current_mode`
- **Package names**: lowercase_with_underscores — `my_interfaces`
- **Constants**: UPPER_SNAKE_CASE — `MODE_IDLE=0`

## Builtin Types
| Type | Description |
|------|-------------|
| `bool` | Boolean |
| `byte` | Unsigned 8-bit |
| `char` | Signed 8-bit |
| `float32`, `float64` | IEEE float |
| `int8/16/32/64` | Signed integers |
| `uint8/16/32/64` | Unsigned integers |
| `string` | UTF-8 string |
| `wstring` | Wide string |

## Using in Python
```python
from my_interfaces.msg import RobotStatus
from my_interfaces.srv import SetMode
from my_interfaces.action import Navigate

msg = RobotStatus()
msg.battery_percent = 85
msg.current_mode = RobotStatus.MODE_PATROL
```

## Using in C++
```cpp
#include "my_interfaces/msg/robot_status.hpp"
#include "my_interfaces/srv/set_mode.hpp"
#include "my_interfaces/action/navigate.hpp"

auto msg = my_interfaces::msg::RobotStatus();
msg.battery_percent = 85;
msg.current_mode = my_interfaces::msg::RobotStatus::MODE_PATROL;
```

## Critical Warnings
- **Circular dependencies**: An interfaces package CANNOT depend on a package that depends on it. Keep interfaces in a standalone package.
- **Rebuild required**: After modifying .msg/.srv/.action files, you MUST rebuild the package and re-source the workspace.
- **No default values for non-constant fields**: Unlike protobuf, .msg fields default to zero/empty. Use constants, not default values in the interface itself.
- **DEPENDENCIES in CMake**: Every external message type used in your interfaces must be listed in both `DEPENDENCIES` (CMake) and `<depend>` (package.xml). Missing deps cause obscure build errors.
- **member_of_group**: The `<member_of_group>rosidl_interface_packages</member_of_group>` line is required or the package won't be recognized as an interface package.
- **Array syntax**: `float64[]` is unbounded dynamic, `float64[6]` is fixed-size. No default values for arrays.
