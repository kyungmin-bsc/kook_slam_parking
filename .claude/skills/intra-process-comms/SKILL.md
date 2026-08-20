<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: intra-process-comms
description: 'Enable ROS 2 intra-process communication. Use when optimizing zero-copy message passing between nodes in the same process.'
---

# Role
You are an expert in ROS 2 intra-process (zero-copy) communication. You guide correct setup of shared-memory message passing within component containers for high-performance pipelines in ROS 2 Jazzy/Rolling.

## What is Intra-Process Communication?
When nodes run in the same process (component container), messages can be passed via pointer (zero-copy) instead of serialized through DDS. This eliminates serialization/deserialization overhead, dramatically reducing latency for large messages like images and point clouds.

## Enabling Intra-Process Communication

### Node Options
```cpp
// C++ — enable in node constructor
rclcpp::NodeOptions options;
options.use_intra_process_comms(true);

class MyNode : public rclcpp::Node {
public:
  explicit MyNode(const rclcpp::NodeOptions &options)
    : Node("my_node", options) {}
};
```

### In Launch File
```python
ComposableNode(
    package='my_package',
    plugin='my_package::MyNode',
    name='my_node',
    extra_arguments=[{'use_intra_process_comms': True}],
)
```

## Zero-Copy Publishing Pattern
```cpp
class ImagePublisher : public rclcpp::Node {
public:
  explicit ImagePublisher(const rclcpp::NodeOptions &options)
    : Node("image_publisher", options)
  {
    pub_ = create_publisher<sensor_msgs::msg::Image>("image", 10);
    timer_ = create_wall_timer(33ms, [this]() { publish_image(); });
  }

private:
  void publish_image() {
    // Allocate with unique_ptr for zero-copy
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = this->now();
    msg->height = 480;
    msg->width = 640;
    msg->encoding = "bgr8";
    msg->step = 640 * 3;
    msg->data.resize(640 * 480 * 3);

    // Fill image data...
    fill_image_data(msg->data);

    // Publish with std::move — transfers ownership, no copy
    pub_->publish(std::move(msg));
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

## Zero-Copy Subscription Pattern
```cpp
class ImageProcessor : public rclcpp::Node {
public:
  explicit ImageProcessor(const rclcpp::NodeOptions &options)
    : Node("image_processor", options)
  {
    // UniquePtr callback — receives ownership, zero-copy
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image", 10,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        // msg is exclusively owned — can modify in-place
        process(std::move(msg));
      });

    pub_ = create_publisher<sensor_msgs::msg::Image>("processed", 10);
  }

private:
  void process(sensor_msgs::msg::Image::UniquePtr msg) {
    // Modify in-place — zero-copy chain
    for (auto &pixel : msg->data) {
      pixel = 255 - pixel;  // invert
    }
    pub_->publish(std::move(msg));
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};
```

## Complete Pipeline Launch
```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

container = ComposableNodeContainer(
    name='image_pipeline',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='my_package', plugin='my_package::ImagePublisher',
            name='publisher',
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='my_package', plugin='my_package::ImageProcessor',
            name='processor',
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='my_package', plugin='my_package::ImageDisplay',
            name='display',
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ],
    output='screen',
)
```

## Performance Comparison
| Scenario | Message Size | Inter-Process | Intra-Process |
|----------|-------------|---------------|---------------|
| 640x480 RGB Image | ~900 KB | ~2-5 ms | ~0.01 ms |
| 1920x1080 RGB | ~6 MB | ~10-20 ms | ~0.01 ms |
| PointCloud2 (100k pts) | ~2.4 MB | ~5-10 ms | ~0.01 ms |
| Small message (Twist) | ~48 bytes | ~0.05 ms | ~0.02 ms |

Zero-copy benefits scale with message size. For small messages (<1 KB), the overhead difference is negligible.

## Fallback Behavior
Intra-process communication automatically falls back to inter-process (DDS) when:
- The subscriber is in a different process.
- Multiple subscribers exist on the same topic.
- The subscription callback takes `SharedPtr` instead of `UniquePtr`.
- `use_intra_process_comms` is not enabled on both publisher and subscriber nodes.

The fallback is seamless — no errors, just reduced performance.

## SharedPtr vs UniquePtr Callbacks
```cpp
// SharedPtr — may or may NOT be zero-copy
// If there's one subscription, intra-process MAY optimize.
// If there are multiple subscriptions, a copy is made.
sub_ = create_subscription<Msg>("topic", 10,
  [](const Msg::SharedPtr msg) { /* read-only access */ });

// UniquePtr — guaranteed zero-copy for single subscriber
// You get exclusive ownership, can modify in-place.
sub_ = create_subscription<Msg>("topic", 10,
  [](Msg::UniquePtr msg) { /* exclusive access, can modify */ });
```

## Verifying Intra-Process is Active
```bash
# Check with ros2 topic info
ros2 topic info /image --verbose
# Look for "Intra Process" in the output

# Or enable debug logging
ros2 run rclcpp_components component_container --ros-args --log-level debug
# Watch for "Intra-process" messages
```

## Critical Warnings
- **Single publisher per topic**: Intra-process comms only works with ONE publisher per topic in the container. Multiple publishers on the same topic fall back to DDS.
- **UniquePtr required for true zero-copy**: If your subscription callback takes `SharedPtr`, the system may still copy the message. Use `UniquePtr` callbacks for guaranteed zero-copy.
- **Invisible to rosbag2**: Intra-process messages bypass DDS. `ros2 bag record` will NOT capture them. If you need to record, add a separate inter-process subscriber (topic relay node) or record with a subscriber in a different process.
- **Invisible to ros2 topic echo**: Same as rosbag — `ros2 topic echo` runs in a separate process and won't see intra-process-only messages.
- **Python limitation**: As of Jazzy, Python (rclpy) does NOT support intra-process communication. All Python nodes serialize through DDS even in the same process.
- **ALL nodes must opt in**: If any node in the chain doesn't have `use_intra_process_comms=True`, that link in the chain falls back to DDS.
- **Don't hold references**: After `pub_->publish(std::move(msg))`, the `msg` pointer is null. Don't use it after publishing.
