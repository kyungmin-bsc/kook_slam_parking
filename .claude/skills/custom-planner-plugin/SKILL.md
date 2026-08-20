<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: custom-planner-plugin
description: 'Create a custom Nav2 planner plugin. Use when implementing a new global planner inheriting from nav2_core::GlobalPlanner.'
---

# Custom Planner Plugin

## Overview

A custom global planner in Nav2 implements the `nav2_core::GlobalPlanner` interface. The planner receives planning requests (start pose, goal pose) and returns a `nav_msgs::msg::Path`. It has access to the global costmap for collision checking and cost evaluation.

## Interface: nav2_core::GlobalPlanner

```cpp
#include "nav2_core/global_planner.hpp"

class GlobalPlanner {
public:
  virtual ~GlobalPlanner() = default;

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  virtual void cleanup() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;

  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;
};
```

## Complete C++ Skeleton

### Header: `my_planner.hpp`

```cpp
#ifndef MY_PLANNER__MY_PLANNER_HPP_
#define MY_PLANNER__MY_PLANNER_HPP_

#include <string>
#include <memory>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace my_planner
{

class MyPlanner : public nav2_core::GlobalPlanner
{
public:
  MyPlanner() = default;
  ~MyPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_;

  // Custom parameters
  double interpolation_resolution_;
};

}  // namespace my_planner

#endif  // MY_PLANNER__MY_PLANNER_HPP_
```

### Source: `my_planner.cpp`

```cpp
#include "my_planner/my_planner.hpp"

#include <cmath>
#include <stdexcept>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_planner
{

void MyPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock parent node");
  }

  // Declare and get parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution",
    rclcpp::ParameterValue(0.1));
  node->get_parameter(
    name_ + ".interpolation_resolution",
    interpolation_resolution_);

  RCLCPP_INFO(node->get_logger(),
    "MyPlanner configured with resolution: %.3f",
    interpolation_resolution_);
}

void MyPlanner::cleanup()
{
  RCLCPP_INFO(
    node_.lock()->get_logger(), "Cleaning up %s", name_.c_str());
}

void MyPlanner::activate()
{
  RCLCPP_INFO(
    node_.lock()->get_logger(), "Activating %s", name_.c_str());
}

void MyPlanner::deactivate()
{
  RCLCPP_INFO(
    node_.lock()->get_logger(), "Deactivating %s", name_.c_str());
}

nav_msgs::msg::Path MyPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_.lock()->now();
  path.header.frame_id = global_frame_;

  // --- Validate start pose ---
  unsigned int start_mx, start_my;
  if (!costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y,
      start_mx, start_my))
  {
    throw nav2_core::StartOutsideMapBounds(
      "Start position is outside the costmap bounds");
  }
  if (costmap_->getCost(start_mx, start_my) ==
      nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    throw nav2_core::StartOccupied(
      "Start position is in a lethal obstacle");
  }

  // --- Validate goal pose ---
  unsigned int goal_mx, goal_my;
  if (!costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y,
      goal_mx, goal_my))
  {
    throw nav2_core::GoalOutsideMapBounds(
      "Goal position is outside the costmap bounds");
  }
  if (costmap_->getCost(goal_mx, goal_my) ==
      nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    throw nav2_core::GoalOccupied(
      "Goal position is in a lethal obstacle");
  }

  // --- Plan: straight-line interpolation (replace with real algorithm) ---
  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;
  double distance = std::hypot(dx, dy);
  int num_steps = static_cast<int>(
    std::ceil(distance / interpolation_resolution_));

  if (num_steps == 0) {
    path.poses.push_back(start);
    path.poses.push_back(goal);
    return path;
  }

  for (int i = 0; i <= num_steps; ++i) {
    double t = static_cast<double>(i) / num_steps;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x =
      start.pose.position.x + t * dx;
    pose.pose.position.y =
      start.pose.position.y + t * dy;
    pose.pose.position.z = 0.0;
    // Interpolate orientation (simple slerp for yaw)
    pose.pose.orientation = goal.pose.orientation;
    path.poses.push_back(pose);
  }

  return path;
}

}  // namespace my_planner

// Register the plugin
PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner, nav2_core::GlobalPlanner)
```

## Costmap Access API

Inside `createPlan()`, the `costmap_` pointer provides:

```cpp
// Get costmap dimensions
unsigned int size_x = costmap_->getSizeInCellsX();
unsigned int size_y = costmap_->getSizeInCellsY();
double resolution = costmap_->getResolution();

// Convert world coordinates to map cell indices
unsigned int mx, my;
bool in_map = costmap_->worldToMap(world_x, world_y, mx, my);

// Convert map cell indices to world coordinates
double wx, wy;
costmap_->mapToWorld(mx, my, wx, wy);

// Get cost at a cell (0-255)
unsigned char cost = costmap_->getCost(mx, my);

// Cost constants
// nav2_costmap_2d::FREE_SPACE = 0
// nav2_costmap_2d::NO_INFORMATION = 255
// nav2_costmap_2d::LETHAL_OBSTACLE = 254
// nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE = 253
```

## Error Handling

Throw specific exceptions from `nav2_core::exceptions`:

| Exception | When to Throw |
|-----------|---------------|
| `StartOutsideMapBounds` | Start pose is outside the costmap. |
| `GoalOutsideMapBounds` | Goal pose is outside the costmap. |
| `StartOccupied` | Start pose is in a lethal obstacle cell. |
| `GoalOccupied` | Goal pose is in a lethal obstacle cell. |
| `NoValidPathCouldBeFound` | The algorithm exhausted its search without finding a path. |
| `TimedOut` | Planning exceeded the allowed time budget. |

The planner server catches these exceptions, logs them, and returns the appropriate failure code to the BT.

## Plugin Registration

### Plugin Description XML: `my_planner_plugin.xml`

```xml
<library path="my_planner">
  <class type="my_planner::MyPlanner"
         base_class_type="nav2_core::GlobalPlanner">
    <description>My custom global planner plugin</description>
  </class>
</library>
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
project(my_planner)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/my_planner.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  nav2_core nav2_costmap_2d nav2_util nav_msgs
  geometry_msgs pluginlib rclcpp rclcpp_lifecycle tf2_ros
)

# Register the plugin description file
pluginlib_export_plugin_description_file(
  nav2_core my_planner_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
```

### package.xml Dependencies

```xml
<depend>nav2_core</depend>
<depend>nav2_costmap_2d</depend>
<depend>nav2_util</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<depend>pluginlib</depend>
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
<depend>tf2_ros</depend>
```

## Using the Custom Planner

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["MyCustomPlanner"]
    MyCustomPlanner:
      plugin: "my_planner::MyPlanner"
      interpolation_resolution: 0.1
```

Build and source:
```bash
colcon build --symlink-install --packages-select my_planner
source install/setup.bash
```
