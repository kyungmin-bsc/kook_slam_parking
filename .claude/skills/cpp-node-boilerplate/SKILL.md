<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: cpp-node-boilerplate
description: 'Create ROS 2 C++ rclcpp nodes. Use when writing a new C++ node, subscriber, publisher, or timer callback.'
---

# Role
You are an expert in writing ROS 2 C++ (rclcpp) nodes. You produce correct, idiomatic rclcpp implementations for ROS 2 Jazzy/Rolling.

## Minimal Standalone Node
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalNode : public rclcpp::Node {
public:
  MinimalNode() : Node("minimal_node") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MinimalNode::timer_callback, this));
  }

private:
  void timer_callback() {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalNode>());
  rclcpp::shutdown();
  return 0;
}
```

## Component-Style Node (Composable)
```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace my_package {

class MyComponent : public rclcpp::Node {
public:
  explicit MyComponent(const rclcpp::NodeOptions &options)
    : Node("my_component", options)
  {
    // Parameters
    this->declare_parameter<double>("rate", 10.0);
    double rate = this->get_parameter("rate").as_double();

    // Publisher
    pub_ = this->create_publisher<std_msgs::msg::String>("output", 10);

    // Subscriber with lambda
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "input", rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Got: %s", msg->data.c_str());
      });

    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&MyComponent::on_timer, this));
  }

private:
  void on_timer() {
    auto msg = std_msgs::msg::String();
    msg.data = "tick";
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace my_package

RCLCPP_COMPONENTS_REGISTER_NODE(my_package::MyComponent)
```

## CMakeLists.txt for Component
```cmake
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(my_component SHARED src/my_component.cpp)
ament_target_dependencies(my_component rclcpp rclcpp_components std_msgs)

rclcpp_components_register_nodes(my_component "my_package::MyComponent")

install(TARGETS my_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```

## Subscription with std::bind vs Lambda
```cpp
// std::bind — use for member functions, requires placeholder
sub_ = create_subscription<Msg>("topic", 10,
  std::bind(&MyNode::callback, this, std::placeholders::_1));

// Lambda — preferred in modern C++, avoids placeholder boilerplate
sub_ = create_subscription<Msg>("topic", 10,
  [this](const Msg::SharedPtr msg) { process(msg); });

// Lambda with unique_ptr for zero-copy intra-process
sub_ = create_subscription<Msg>("topic", 10,
  [this](Msg::UniquePtr msg) { process(std::move(msg)); });
```

## Parameter Declaration with Descriptors
```cpp
rcl_interfaces::msg::ParameterDescriptor desc;
desc.description = "Maximum velocity in m/s";
desc.floating_point_range.resize(1);
desc.floating_point_range[0].from_value = 0.0;
desc.floating_point_range[0].to_value = 2.0;
desc.floating_point_range[0].step = 0.01;
this->declare_parameter("max_vel", 1.0, desc);
```

## Critical Warnings
- **shared_from_this() in constructor**: NEVER call `shared_from_this()` inside a constructor. The shared_ptr does not exist yet. Use a separate `init()` method or a post-construction factory pattern if you need the shared pointer.
- **Subscription SharedPtr storage**: Always store the subscription/publisher/timer SharedPtr as a class member. If it goes out of scope, it is destroyed and silently stops.
- **RCLCPP_INFO format strings**: Use C printf-style format specifiers, NOT C++ streams. `RCLCPP_INFO(get_logger(), "val: %f", val)` not `<< val`.
- **Thread safety**: With `MultiThreadedExecutor`, use `MutuallyExclusiveCallbackGroup` or mutexes to protect shared state.
- **Header-only messages**: Message headers are auto-generated. Include as `"pkg/msg/type.hpp"` (lowercase, underscored).

## Logging Macros
```cpp
RCLCPP_INFO(this->get_logger(), "Info message");
RCLCPP_WARN(this->get_logger(), "Warning: %d", val);
RCLCPP_ERROR(this->get_logger(), "Error occurred");
RCLCPP_DEBUG(this->get_logger(), "Debug detail");
RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Every 5s");
RCLCPP_WARN_ONCE(this->get_logger(), "Only printed once");
```

## Best Practices
- Prefer component-style nodes for composability and shared-memory transport.
- Use `const SharedPtr&` or `SharedPtr` in callbacks; use `UniquePtr` for zero-copy.
- Declare all parameters in the constructor with descriptors.
- Use namespaces to avoid ODR violations when composing multiple components.
- Prefer `create_wall_timer` for non-sim-time timers, `create_timer` with node clock for sim-time-aware timers.
