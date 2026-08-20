<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: lifecycle-nodes
description: 'Implement ROS 2 managed lifecycle nodes. Use when adding on_configure, on_activate, on_deactivate, on_cleanup transitions.'
---

# Role
You are an expert in ROS 2 managed (lifecycle) nodes. You guide correct implementation of lifecycle state machines using rclcpp_lifecycle and rclpy lifecycle for hardware drivers, Nav2 servers, and any node requiring deterministic startup/shutdown.

## Lifecycle States
```
         on_configure          on_activate
Unconfigured ---------> Inactive ----------> Active
      ^                    |         |          |
      |   on_cleanup       |         |          |
      +--------------------+         |          |
                                     |          |
                           on_deactivate        |
                           <--------------------+
                           
Any state --on_shutdown--> Finalized
Any state --on_error----> ErrorProcessing ---> Finalized | Unconfigured
```

States: `Unconfigured` → `Inactive` → `Active` → `Finalized`. Transitions are triggered externally (lifecycle manager, CLI, or launch).

## C++ Lifecycle Node
```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit MyLifecycleNode(const rclcpp::NodeOptions &options)
    : LifecycleNode("my_lifecycle_node", options) {}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    // Allocate resources, read parameters, create inactive publishers
    RCLCPP_INFO(get_logger(), "Configuring...");
    pub_ = this->create_publisher<std_msgs::msg::String>("output", 10);
    // Publisher is NOT yet activated — publish() will silently drop messages
    return CallbackReturn::SUCCESS;  // or FAILURE, ERROR
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Activating...");
    // Now publish() will actually send messages
    // Start timers, enable hardware
    timer_ = this->create_wall_timer(100ms, [this]() { on_timer(); });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    timer_->cancel();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    pub_.reset();
    timer_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Shutting down...");
    // Release all resources
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) override {
    RCLCPP_ERROR(get_logger(), "Error encountered!");
    return CallbackReturn::SUCCESS;  // SUCCESS → Unconfigured, FAILURE → Finalized
  }

private:
  void on_timer() {
    auto msg = std_msgs::msg::String();
    msg.data = "active data";
    pub_->publish(msg);
  }
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

## Python Lifecycle Node
```python
from rclpy.lifecycle import Node as LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import String

class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_lifecycle_node')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.pub_ = self.create_lifecycle_publisher(String, 'output', 10)
        self.get_logger().info('Configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.timer_ = self.create_timer(0.1, self.on_timer)
        return super().on_activate(state)  # activates the lifecycle publisher

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.timer_.cancel()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self.pub_)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down')
        return TransitionCallbackReturn.SUCCESS

    def on_timer(self):
        msg = String()
        msg.data = 'active'
        self.pub_.publish(msg)
```

## Triggering Transitions
```bash
# CLI
ros2 lifecycle set /my_lifecycle_node configure
ros2 lifecycle set /my_lifecycle_node activate
ros2 lifecycle set /my_lifecycle_node deactivate
ros2 lifecycle set /my_lifecycle_node cleanup
ros2 lifecycle set /my_lifecycle_node shutdown

# Check state
ros2 lifecycle get /my_lifecycle_node
```

## When to Use Lifecycle Nodes
- **Hardware drivers**: Configure hardware in `on_configure`, start streaming in `on_activate`, stop in `on_deactivate`.
- **Nav2 servers**: All Nav2 servers (planner, controller, BT navigator) are lifecycle-managed. Bond connections ensure coordinated bringup.
- **Sensor pipelines**: Allocate buffers in `on_configure`, start publishing in `on_activate`.
- **System orchestration**: When you need deterministic, ordered startup across many nodes.

## Launch Integration
```python
from launch_ros.actions import LifecycleNode as LifecycleNodeAction
from launch_ros.event_handlers import OnStateTransition
from launch.actions import EmitEvent
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg

# Configure node after launch
LifecycleNodeAction(package='pkg', executable='node', name='my_node')
# Then emit ChangeState events to transition it
```

## Critical Warnings
- **LifecyclePublisher**: Messages published by a `LifecyclePublisher` are silently dropped unless the node is in the `Active` state. Use `create_publisher` (not lifecycle publisher) if you need to publish in any state.
- **Return values matter**: Returning `FAILURE` from `on_configure` leaves the node in `Unconfigured`. Returning `ERROR` from any transition triggers `on_error`.
- **Bond connections**: Nav2 uses bond connections between the lifecycle manager and servers. If a server dies, the bond breaks and the manager can attempt recovery.
- **Don't block transitions**: Transition callbacks should be fast. Long-running initialization should be done asynchronously or with timeouts.
- **Subscribers are always active**: Unlike publishers, subscribers receive messages regardless of lifecycle state. Guard your subscription callbacks with state checks if needed.
