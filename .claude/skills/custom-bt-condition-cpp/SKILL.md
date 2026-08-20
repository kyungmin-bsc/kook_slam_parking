<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: custom-bt-condition-cpp
description: 'Create custom BT condition nodes in C++. Use when implementing new BehaviorTree.CPP condition plugins.'
---

# Writing Custom C++ BT Condition Nodes

## Condition Node Contract

Condition nodes return only **SUCCESS** or **FAILURE**—never RUNNING. They query world state synchronously and must complete within a single tick. They are typically placed in `ReactiveSequence` or `ReactiveFallback` nodes where they are re-evaluated every tick.

## Basic Structure

Inherit from `BT::ConditionNode`:

```cpp
#ifndef IS_INTRUDER_DETECTED_HPP
#define IS_INTRUDER_DETECTED_HPP

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <mutex>

namespace my_bt_nodes
{

class IsIntruderDetected : public BT::ConditionNode
{
public:
  IsIntruderDetected(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("detection_topic", "/detections",
        "Topic with Detection2DArray messages"),
      BT::InputPort<double>("max_distance", 5.0,
        "Maximum detection distance in meters"),
      BT::InputPort<std::string>("target_class", "person",
        "Object class to detect"),
      BT::OutputPort<double>("intruder_distance",
        "Distance to nearest intruder"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_;
  vision_msgs::msg::Detection2DArray::SharedPtr last_msg_;
  std::mutex msg_mutex_;

  void detection_callback(
    const vision_msgs::msg::Detection2DArray::SharedPtr msg);
};

}  // namespace my_bt_nodes

#endif
```

## Implementation

### Constructor: Set Up ROS 2 Subscription

Access the ROS 2 node from the BT blackboard (Nav2 stores it there) and create your subscription:

```cpp
#include "my_bt_nodes/is_intruder_detected.hpp"

namespace my_bt_nodes
{

IsIntruderDetected::IsIntruderDetected(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string topic;
  getInput<std::string>("detection_topic", topic);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort();

  sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
    topic, qos,
    std::bind(&IsIntruderDetected::detection_callback, this,
              std::placeholders::_1));
}
```

### Subscription Callback: Thread Safety

The subscription callback runs on a ROS 2 executor thread, while `tick()` runs on the BT thread. Protect shared data with a mutex:

```cpp
void IsIntruderDetected::detection_callback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(msg_mutex_);
  last_msg_ = msg;
}
```

**Critical**: Keep the callback minimal—just store the message. All processing happens in `tick()` to avoid blocking the executor.

### tick(): The Decision Logic

```cpp
BT::NodeStatus IsIntruderDetected::tick()
{
  double max_distance;
  getInput<double>("max_distance", max_distance);

  std::string target_class;
  getInput<std::string>("target_class", target_class);

  vision_msgs::msg::Detection2DArray::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    msg = last_msg_;
  }

  if (!msg) {
    return BT::NodeStatus::FAILURE;  // No detection data yet
  }

  // Check detection age — stale data should not trigger
  auto age = node_->now() - msg->header.stamp;
  if (age.seconds() > 2.0) {
    return BT::NodeStatus::FAILURE;  // Data too old
  }

  double nearest_distance = std::numeric_limits<double>::max();

  for (const auto & detection : msg->detections) {
    for (const auto & result : detection.results) {
      if (result.hypothesis.class_id == target_class &&
          result.hypothesis.score > 0.6)
      {
        // Use bounding box center to estimate distance
        // (in practice, use 3D detection or depth data)
        double dist = estimate_distance(detection.bbox);
        if (dist < nearest_distance) {
          nearest_distance = dist;
        }
      }
    }
  }

  if (nearest_distance <= max_distance) {
    setOutput("intruder_distance", nearest_distance);
    return BT::NodeStatus::SUCCESS;  // Intruder detected!
  }

  return BT::NodeStatus::FAILURE;  // No intruder within range
}

}  // namespace my_bt_nodes
```

## Plugin Registration

```cpp
#include "behaviortree_cpp/bt_factory.h"
#include "my_bt_nodes/is_intruder_detected.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<my_bt_nodes::IsIntruderDetected>("IsIntruderDetected");
}
```

For condition nodes, `registerNodeType` works directly because the constructor signature matches BT.CPP's expectations (name + config). No custom builder needed.

## CMakeLists.txt

```cmake
add_library(is_intruder_detected_bt_node SHARED
  src/is_intruder_detected.cpp
)

ament_target_dependencies(is_intruder_detected_bt_node
  rclcpp
  behaviortree_cpp
  vision_msgs
  nav2_behavior_tree
)

install(TARGETS is_intruder_detected_bt_node
  LIBRARY DESTINATION lib
)
```

## Service-Based Conditions

For conditions that query a ROS 2 service instead of a topic:

```cpp
BT::NodeStatus IsAreaSecure::tick()
{
  auto client = node_->create_client<my_interfaces::srv::CheckArea>(
    "check_area_secure");

  if (!client->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(node_->get_logger(), "CheckArea service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<my_interfaces::srv::CheckArea::Request>();
  getInput<std::string>("area_id", request->area_id);

  auto future = client->async_send_request(request);

  // Block with timeout — conditions must not return RUNNING
  if (future.wait_for(std::chrono::milliseconds(1000)) ==
      std::future_status::ready)
  {
    auto response = future.get();
    return response->is_secure ? BT::NodeStatus::SUCCESS
                               : BT::NodeStatus::FAILURE;
  }

  RCLCPP_WARN(node_->get_logger(), "CheckArea service timed out");
  return BT::NodeStatus::FAILURE;
}
```

**Warning**: Service calls block the BT thread. Keep timeouts short (≤1 second) to avoid stalling the entire tree. For long-running queries, use an action node instead.

## XML Usage

```xml
<ReactiveSequence>
  <Inverter>
    <IsIntruderDetected detection_topic="/oak_d/detections"
                        max_distance="3.0" target_class="person"
                        intruder_distance="{detected_distance}" />
  </Inverter>
  <!-- Normal patrol continues only if no intruder detected -->
  <NavigateToPose goal="{patrol_goal}" />
</ReactiveSequence>
```

Or trigger an alert when an intruder IS detected:

```xml
<ReactiveFallback>
  <IsIntruderDetected detection_topic="/oak_d/detections"
                      max_distance="5.0" target_class="person"
                      intruder_distance="{detected_distance}" />
  <!-- Fallback: if no intruder, continue patrol -->
  <SubTree ID="PatrolLoop" />
</ReactiveFallback>
```

## Thread Safety Summary

| Data Flow              | Protection Needed           |
|------------------------|-----------------------------|
| ROS callback → member  | `std::mutex` or atomic      |
| tick() reads member    | Same mutex as callback      |
| tick() reads ports     | None (BT.CPP handles this)  |
| tick() writes ports    | None (single-threaded ticks) |

The BT executor calls `tick()` sequentially—it never ticks two nodes simultaneously. But ROS 2 subscription callbacks run on a separate executor thread, so shared state between callbacks and `tick()` must be synchronized.
