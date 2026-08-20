<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: custom-behavior-plugin
description: 'Create a custom Nav2 behavior plugin. Use when implementing a new recovery behavior inheriting from nav2_core::Behavior.'
---

# Writing a Custom Behavior Plugin

## Overview

Custom behaviors extend `nav2_core::Behavior` to implement robot-specific recovery or operational actions. The behavior server loads them as plugins, making them available as BT action nodes. This guide walks through the full implementation of a custom behavior.

## Required Interface

Your plugin must inherit from `nav2_core::Behavior<ActionT>` and implement:

| Method | Called When | Purpose |
|---|---|---|
| `configure()` | Server startup | Grab parameters, set up publishers/subscribers |
| `cleanup()` | Server shutdown | Release resources |
| `activate()` | Server activation | Start any timers or subscriptions |
| `deactivate()` | Server deactivation | Stop timers or subscriptions |
| `onRun(goal)` | Behavior starts | Extract parameters from the action goal |
| `onCycleUpdate()` | Every cycle | Execute one tick of the behavior, return status |

`onCycleUpdate()` returns `Status::SUCCEEDED`, `Status::FAILED`, or `Status::RUNNING`.

## Header File — `drive_to_charger.hpp`

```cpp
#ifndef MY_ROBOT_BEHAVIORS__DRIVE_TO_CHARGER_HPP_
#define MY_ROBOT_BEHAVIORS__DRIVE_TO_CHARGER_HPP_

#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/back_up.hpp"   // reuse BackUp action or define custom
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"

namespace my_robot_behaviors
{

class DriveToCharger : public nav2_behaviors::TimedBehavior<nav2_msgs::action::BackUp>
{
public:
  using BackUpAction = nav2_msgs::action::BackUp;

  DriveToCharger();
  ~DriveToCharger() override = default;

  Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;
  Status onCycleUpdate() override;

protected:
  void onConfigure() override;
  void onCleanup() override;
  void onActionCompletion() override;

private:
  double approach_speed_;
  double approach_distance_;
  double distance_traveled_;
  geometry_msgs::msg::PoseStamped start_pose_;

  bool getStartPose();
  double getDistanceTraveled();
};

}  // namespace my_robot_behaviors

#endif  // MY_ROBOT_BEHAVIORS__DRIVE_TO_CHARGER_HPP_
```

## Source File — `drive_to_charger.cpp`

```cpp
#include "my_robot_behaviors/drive_to_charger.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

namespace my_robot_behaviors
{

DriveToCharger::DriveToCharger()
: TimedBehavior<BackUpAction>(),
  approach_speed_(0.0),
  approach_distance_(0.0),
  distance_traveled_(0.0)
{
}

void DriveToCharger::onConfigure()
{
  auto node = node_.lock();
  if (!node) {return;}

  nav2_util::declare_parameter_if_not_declared(
    node, "approach_speed", rclcpp::ParameterValue(0.05));
  node->get_parameter("approach_speed", approach_speed_);
}

void DriveToCharger::onCleanup()
{
  // Release any resources
}

Status DriveToCharger::onRun(const std::shared_ptr<const BackUpAction::Goal> command)
{
  // Extract distance from the BackUp action goal (reused interface)
  approach_distance_ = std::abs(command->target.x);
  approach_speed_ = std::abs(command->speed);
  distance_traveled_ = 0.0;

  if (!getStartPose()) {
    RCLCPP_ERROR(logger_, "Failed to get start pose for DriveToCharger");
    return Status::FAILED;
  }

  RCLCPP_INFO(logger_, "DriveToCharger: approaching %.2f m at %.2f m/s",
    approach_distance_, approach_speed_);
  return Status::SUCCEEDED;  // onRun succeeded, begin cycling
}

Status DriveToCharger::onCycleUpdate()
{
  distance_traveled_ = getDistanceTraveled();

  if (distance_traveled_ >= approach_distance_) {
    stopRobot();
    RCLCPP_INFO(logger_, "DriveToCharger: reached charger (%.2f m)", distance_traveled_);
    return Status::SUCCEEDED;
  }

  // Check for collision before commanding
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = approach_speed_;

  if (!isCollisionFree(
      distance_traveled_, cmd_vel->linear.x, cmd_vel->linear.y, cmd_vel->angular.z))
  {
    stopRobot();
    RCLCPP_WARN(logger_, "DriveToCharger: collision detected, stopping");
    return Status::FAILED;
  }

  vel_pub_->publish(std::move(cmd_vel));
  return Status::RUNNING;
}

void DriveToCharger::onActionCompletion()
{
  stopRobot();
}

bool DriveToCharger::getStartPose()
{
  auto current_pose = nav2_behaviors::TimedBehavior<BackUpAction>::getStartPose();
  if (!current_pose) {return false;}
  start_pose_ = *current_pose;
  return true;
}

double DriveToCharger::getDistanceTraveled()
{
  auto current_pose = nav2_behaviors::TimedBehavior<BackUpAction>::getStartPose();
  if (!current_pose) {return 0.0;}

  double dx = current_pose->pose.position.x - start_pose_.pose.position.x;
  double dy = current_pose->pose.position.y - start_pose_.pose.position.y;
  return std::hypot(dx, dy);
}

}  // namespace my_robot_behaviors

PLUGINLIB_EXPORT_CLASS(my_robot_behaviors::DriveToCharger, nav2_core::Behavior)
```

## Plugin Description — `behavior_plugin.xml`

```xml
<library path="my_robot_behaviors">
  <class type="my_robot_behaviors::DriveToCharger"
         base_class_type="nav2_core::Behavior">
    <description>Drive forward toward charger with collision checking</description>
  </class>
</library>
```

## CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
project(my_robot_behaviors)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/drive_to_charger.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  nav2_core nav2_behaviors nav2_msgs nav2_util
  pluginlib rclcpp geometry_msgs tf2 tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

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

## Registering in behavior_server YAML

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "wait", "drive_to_charger"]
    drive_to_charger:
      plugin: "my_robot_behaviors::DriveToCharger"
    approach_speed: 0.05
```

## Using a Custom Action Type

If the BackUp action interface doesn't fit, define a custom action in `my_robot_interfaces`:

```
# DriveToCharger.action
# Goal
float32 target_distance
float32 approach_speed
---
# Result
builtin_interfaces/Duration total_elapsed_time
---
# Feedback
float32 distance_remaining
```

Then template the behavior on your custom action type instead of `nav2_msgs::action::BackUp`.

## Key Utilities Available in TimedBehavior

- `vel_pub_`: publisher for `cmd_vel`
- `stopRobot()`: publishes zero velocity
- `isCollisionFree(dist, vx, vy, vtheta)`: checks costmap for collision along a projected trajectory
- `costmap_`: shared pointer to the costmap for direct cell queries
- `tf_`: TF buffer for pose lookups
- `logger_`: the node's logger
- `clock_`: the node's clock
- `cycle_frequency_`: configured tick rate
