<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: package-xml-cmake
description: 'Configure ROS 2 package.xml and CMakeLists.txt. Use when creating ament_cmake or ament_python packages, declaring dependencies.'
---

# Package Configuration: package.xml and CMakeLists.txt

## package.xml (Format 3)

Every ROS 2 package requires a `package.xml` at its root. Format 3 is the current standard.

### Dependency Types

| Tag | When Used | Example |
|-----|-----------|---------|
| `<depend>` | Build + export + exec (shorthand) | `<depend>rclcpp</depend>` |
| `<build_depend>` | Needed only at build time | `<build_depend>rosidl_default_generators</build_depend>` |
| `<exec_depend>` | Needed only at runtime | `<exec_depend>robot_state_publisher</exec_depend>` |
| `<test_depend>` | Needed only for testing | `<test_depend>ament_lint_auto</test_depend>` |
| `<buildtool_depend>` | The build system itself | `<buildtool_depend>ament_cmake</buildtool_depend>` |
| `<build_export_depend>` | Propagated to downstream build | `<build_export_depend>std_msgs</build_export_depend>` |

### Complete C++ Package: Navigation Node

**package.xml:**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
  schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_nav_controller</name>
  <version>1.0.0</version>
  <description>Navigation controller for the robot</description>
  <maintainer email="dev@example.com">Michael Wimble</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>nav2_msgs</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <exec_depend>robot_state_publisher</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**CMakeLists.txt:**

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_nav_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)

# Build the node executable
add_executable(nav_controller
  src/nav_controller_node.cpp
  src/path_follower.cpp
)

# Link ROS 2 dependencies (preferred over target_link_libraries for ROS pkgs)
ament_target_dependencies(nav_controller
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  nav2_msgs
)

# Install the executable
install(TARGETS nav_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config/param files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install URDF/xacro files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_follower test/test_path_follower.cpp)
  ament_target_dependencies(test_path_follower rclcpp geometry_msgs)
endif()

ament_package()
```

## Python Package (Jazzy)

In ROS 2 Jazzy, Python packages are transitioning from `setup.py`/`setup.cfg` to `ament_python` with `pyproject.toml`. The legacy `setup.py` approach still works.

**package.xml:**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_perception</name>
  <version>1.0.0</version>
  <description>Perception nodes for the robot</description>
  <maintainer email="dev@example.com">Michael Wimble</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>python3-opencv</exec_depend>
  <exec_depend>python3-numpy</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**setup.py (legacy, still functional in Jazzy):**

```python
from setuptools import setup

package_name = 'my_robot_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
        ('share/' + package_name + '/config', ['config/detector_params.yaml']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'detector_node = my_robot_perception.detector_node:main',
            'tracker_node = my_robot_perception.tracker_node:main',
        ],
    },
)
```

**setup.cfg:**

```ini
[develop]
script_dir=$base/lib/my_robot_perception
[install]
install_scripts=$base/lib/my_robot_perception
```

## Interface Package (Messages/Services/Actions)

For packages that define custom `.msg`, `.srv`, or `.action` files:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/SetPatrolRoute.srv"
  "action/NavigateWaypoints.action"
  DEPENDENCIES geometry_msgs std_msgs
)

# If the same package also has nodes using these interfaces:
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(my_node "${cpp_typesupport_target}")
```

## Key CMake Install Patterns

```cmake
# Install a library (for shared components)
add_library(path_utils SHARED src/path_utils.cpp)
ament_target_dependencies(path_utils rclcpp nav_msgs)
install(TARGETS path_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files (for library packages)
install(DIRECTORY include/
  DESTINATION include
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/calibrate.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies for downstream packages
ament_export_dependencies(rclcpp nav_msgs)
ament_export_targets(export_path_utils)
ament_export_include_directories(include)
```

## Validation

```bash
# Check package.xml is valid
ros2 pkg xml my_robot_nav_controller

# List all declared dependencies
rosdep keys --from-paths src/my_robot_nav_controller
```
