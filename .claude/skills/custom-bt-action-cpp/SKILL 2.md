<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: custom-bt-action-cpp
description: 'Create custom BT action nodes in C++. Use when implementing new BehaviorTree.CPP action plugins for Nav2.'
---

# Writing Custom C++ BT Action Nodes for ROS 2

## Architecture

Custom BT action nodes bridge BehaviorTree.CPP with ROS 2 action servers. Nav2 provides `BtActionNode<T>` as a template base class that handles action client lifecycle, goal sending, feedback monitoring, and result processing.

## Inheriting from BtActionNode

```cpp
#ifndef MY_CUSTOM_ACTION_NODE_HPP
#define MY_CUSTOM_ACTION_NODE_HPP

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "my_interfaces/action/say_something.hpp"

namespace my_bt_nodes
{

class SaySomethingAction
  : public nav2_behavior_tree::BtActionNode<my_interfaces::action::SaySomething>
{
public:
  SaySomethingAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<my_interfaces::action::SaySomething>(
      xml_tag_name, action_name, conf)
  {}

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("message", "Text to speak"),
      BT::InputPort<double>("volume", 1.0, "Volume level 0.0-1.0"),
      BT::OutputPort<bool>("success", "Whether speech completed"),
    });
  }
};

}  // namespace my_bt_nodes

#endif
```

**Key points:**
- Template parameter is your ROS 2 action type (`my_interfaces::action::SaySomething`)
- Constructor must accept `xml_tag_name`, `action_name`, and `conf`—forward all to the base class
- `providedBasicPorts({...})` merges your ports with the required `server_name` and `server_timeout` ports from the base class

## Override Methods

### on_tick()

Called every time the node is ticked and the action goal needs to be sent. Set the goal fields from blackboard ports here:

```cpp
void SaySomethingAction::on_tick()
{
  std::string message;
  getInput<std::string>("message", message);

  double volume = 1.0;
  getInput<double>("volume", volume);

  goal_.message = message;
  goal_.volume = static_cast<float>(volume);
}
```

`goal_` is a member variable of type `ActionT::Goal` provided by the base class. The base class sends this goal to the action server after `on_tick()` returns.

### on_success()

Called when the action server reports the goal succeeded:

```cpp
BT::NodeStatus SaySomethingAction::on_success()
{
  setOutput("success", true);
  return BT::NodeStatus::SUCCESS;
}
```

Return `SUCCESS` to propagate success up the tree. You can also return `FAILURE` if post-processing detects a problem.

### on_aborted()

Called when the action server aborts the goal:

```cpp
BT::NodeStatus SaySomethingAction::on_aborted()
{
  setOutput("success", false);
  RCLCPP_WARN(node_->get_logger(), "SaySomething action aborted");
  return BT::NodeStatus::FAILURE;
}
```

### on_cancelled()

Called when the goal is cancelled (e.g., the BT halts this node):

```cpp
BT::NodeStatus SaySomethingAction::on_cancelled()
{
  setOutput("success", false);
  return BT::NodeStatus::SUCCESS;  // Cancellation is a graceful exit
}
```

Returning SUCCESS from `on_cancelled()` means "I handled the cancellation cleanly." Return FAILURE if cancellation represents an error.

## Blackboard I/O

Read input ports with `getInput<T>()`, write output ports with `setOutput()`:

```cpp
// Reading with default value
std::string msg;
if (!getInput<std::string>("message", msg)) {
  msg = "Hello";  // Fallback if port not connected
}

// Reading required port (throws if missing)
auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");

// Writing output
setOutput("success", true);
setOutput("error_code", result_.result->error_code);
```

For geometry types, BT.CPP v4 has built-in converters for common ROS message types when registered through the Nav2 BT framework.

## Plugin Registration

Register the node as a BT.CPP plugin using the macro at the bottom of the `.cpp` file:

```cpp
#include "my_bt_nodes/say_something_action.hpp"
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<my_bt_nodes::SaySomethingAction>(
        name, "say_something", config);
    };
  factory.registerBuilder<my_bt_nodes::SaySomethingAction>(
    "SaySomething", builder);
}
```

The second argument to the constructor (`"say_something"`) is the default ROS 2 action server name.

## CMakeLists.txt

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(my_interfaces REQUIRED)

add_library(my_say_something_bt_node SHARED
  src/say_something_action.cpp
)

ament_target_dependencies(my_say_something_bt_node
  rclcpp
  rclcpp_action
  nav2_behavior_tree
  behaviortree_cpp
  my_interfaces
)

install(TARGETS my_say_something_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

## Loading in bt_navigator

Add the shared library to your Nav2 `bt_navigator` configuration:

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_clear_costmap_service_bt_node
      # Your custom plugin:
      - my_say_something_bt_node
```

The library name must match the target name in `CMakeLists.txt` (without the `lib` prefix and `.so` suffix).

## XML Usage

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="SpeakAndNavigate">
    <Sequence>
      <SaySomething message="Starting patrol" volume="0.8" success="{speech_ok}" />
      <NavigateToPose goal="{patrol_start}" />
      <SaySomething message="Patrol complete" volume="1.0" />
    </Sequence>
  </BehaviorTree>
</root>
```

## Handling Feedback

Override `on_wait_for_result()` to process action feedback while waiting:

```cpp
BT::NodeStatus SaySomethingAction::on_wait_for_result(
  std::shared_ptr<const ActionT::Feedback> feedback)
{
  if (feedback->progress > 0.9) {
    RCLCPP_INFO(node_->get_logger(), "Almost done speaking (%.0f%%)",
                feedback->progress * 100);
  }
  return BT::NodeStatus::RUNNING;  // Keep waiting
}
```

Return `RUNNING` to continue waiting for the result. Return `SUCCESS` or `FAILURE` to preempt the wait based on feedback content.
