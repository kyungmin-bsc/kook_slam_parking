<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: custom-controller-plugin
description: 'Create a custom Nav2 controller plugin. Use when implementing a new local controller inheriting from nav2_core::Controller.'
---

# Custom Controller Plugin Development

## Overview

Nav2 controllers implement the `nav2_core::Controller` interface. A custom controller receives a global path, the robot's current pose and velocity, and the local costmap, then produces velocity commands.

## Required Interface

Inherit from `nav2_core::Controller` and implement:

```cpp
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace my_controllers {

class MyController : public nav2_core::Controller
{
public:
  MyController() = default;
  ~MyController() override = default;

  /**
   * Configure the controller. Called once during lifecycle transition.
   * @param parent WeakPtr to the controller server node (for params, logging, clock).
   * @param name Controller plugin name (used as param namespace).
   * @param tf TF buffer for coordinate transforms.
   * @param costmap_ros Shared pointer to the local costmap.
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /** Cleanup resources. Called during deactivation. */
  void cleanup() override;

  /** Activate the controller. Called when transitioning to active state. */
  void activate() override;

  /** Deactivate the controller. Called when transitioning to inactive state. */
  void deactivate() override;

  /**
   * Set the global plan to follow.
   * Called whenever the planner produces a new path.
   * @param path The global plan in map frame.
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * Set the path for speed limiting along the path.
   * Optional override; base implementation is a no-op.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * Compute velocity commands.
   * Called at controller_frequency Hz.
   * @param pose Current robot pose (from TF, map frame).
   * @param velocity Current robot velocity (from odometry).
   * @param goal_checker Optional goal checker to determine if goal is reached.
   * @return TwistStamped velocity command to send to the robot.
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  rclcpp::Logger logger_{rclcpp::get_logger("MyController")};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace my_controllers
```

## Implementation Skeleton

```cpp
#include "my_controllers/my_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_controllers {

void MyController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  if (!node) {
    throw nav2_core::ControllerException("Failed to lock node");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Declare and read parameters (namespaced under plugin name)
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  double desired_vel;
  node->get_parameter(name_ + ".desired_linear_vel", desired_vel);

  RCLCPP_INFO(logger_, "MyController configured with desired_vel=%.2f", desired_vel);
}

void MyController::cleanup() {
  RCLCPP_INFO(logger_, "Cleaning up MyController");
}

void MyController::activate() {
  RCLCPP_INFO(logger_, "Activating MyController");
}

void MyController::deactivate() {
  RCLCPP_INFO(logger_, "Deactivating MyController");
}

void MyController::setPlan(const nav_msgs::msg::Path & path) {
  global_plan_ = path;
}

void MyController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  // Optionally handle speed limit zones
  (void)speed_limit;
  (void)percentage;
}

geometry_msgs::msg::TwistStamped MyController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;  // Use if you need current velocity for acceleration limiting

  // Access the costmap
  auto costmap = costmap_ros_->getCostmap();
  // costmap->getCost(mx, my) returns cost at map cell (mx, my)

  if (global_plan_.poses.empty()) {
    throw nav2_core::NoValidControl("Empty global plan");
  }

  // Check if goal is reached
  if (goal_checker && goal_checker->isGoalReached(
      pose.pose, global_plan_.poses.back().pose,
      velocity))
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = clock_->now();
    cmd.header.frame_id = pose.header.frame_id;
    // Zero velocity — we've arrived
    return cmd;
  }

  // --- Your control logic here ---
  // Example: drive toward next path point
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();
  cmd.header.frame_id = pose.header.frame_id;
  cmd.twist.linear.x = 0.3;   // Forward
  cmd.twist.angular.z = 0.0;  // Straight
  return cmd;
}

}  // namespace my_controllers

// Register the plugin
PLUGINLIB_EXPORT_CLASS(my_controllers::MyController, nav2_core::Controller)
```

## Plugin Registration

### plugin_description.xml

```xml
<library path="my_controller_lib">
  <class name="my_controllers::MyController"
         type="my_controllers::MyController"
         base_class_type="nav2_core::Controller">
    <description>My custom Nav2 controller</description>
  </class>
</library>
```

### CMakeLists.txt (relevant parts)

```cmake
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

add_library(my_controller_lib SHARED src/my_controller.cpp)
ament_target_dependencies(my_controller_lib
  nav2_core nav2_costmap_2d pluginlib rclcpp geometry_msgs nav_msgs)

pluginlib_export_plugin_description_file(nav2_core plugin_description.xml)

install(TARGETS my_controller_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)
```

### package.xml dependencies

```xml
<depend>nav2_core</depend>
<depend>nav2_costmap_2d</depend>
<depend>pluginlib</depend>
<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
```

## Error Handling

Throw specific exceptions from `nav2_core/controller_exceptions.hpp`:

| Exception | When to use |
|-----------|-------------|
| `nav2_core::ControllerException` | General controller failure |
| `nav2_core::NoValidControl` | Cannot compute a valid velocity (blocked, no path) |
| `nav2_core::ControllerTFError` | TF lookup failure |
| `nav2_core::FailedToMakeProgress` | Robot appears stuck (optional) |

The controller server catches these and triggers recovery behaviors or aborts navigation accordingly.

## Available Resources in computeVelocityCommands

- `costmap_ros_->getCostmap()`: Access the local costmap for collision checking.
- `tf_`: Look up transforms between frames.
- `clock_->now()`: Current time for timestamps.
- `global_plan_`: The path set by `setPlan()`.
- `pose`: Robot's current pose in the costmap frame.
- `velocity`: Robot's current velocity from odometry.

## Testing

Use the Nav2 test infrastructure:
```cpp
#include "nav2_controller/plugins/test_controller.hpp"
// Or write integration tests using nav2_system_tests
```

Unit test `computeVelocityCommands` with known poses, velocities, and paths. Verify:
- Returns non-zero velocity when path exists and goal not reached.
- Returns zero velocity when goal is reached.
- Throws appropriate exceptions on empty path or blocked state.
