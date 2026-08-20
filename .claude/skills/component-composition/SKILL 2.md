<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: component-composition
description: 'Use ROS 2 component composition. Use when loading multiple nodes in a single process for zero-copy transport.'
---

# Role
You are an expert in ROS 2 composable node (component) architecture. You guide correct implementation of shared-memory, in-process node composition for high-performance systems in ROS 2 Jazzy/Rolling.

## Why Composition?
- **Shared memory transport**: Nodes in the same process can pass messages via pointers (zero-copy) instead of serialization.
- **Reduced overhead**: No inter-process serialization, no DDS discovery overhead between co-located nodes.
- **Better cache locality**: All nodes share the same address space.
- **Use case**: Image pipelines, sensor processing chains, multi-node robots where latency matters.

## Creating a Composable Node (C++)
```cpp
// src/my_component.cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace my_package {

class ImageProcessor : public rclcpp::Node {
public:
  // MUST accept NodeOptions — this is the component interface
  explicit ImageProcessor(const rclcpp::NodeOptions &options)
    : Node("image_processor", options)
  {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        // UniquePtr enables zero-copy intra-process
        process(std::move(msg));
      });
    pub_ = create_publisher<sensor_msgs::msg::Image>("image_processed", 10);
  }

private:
  void process(sensor_msgs::msg::Image::UniquePtr msg) {
    // Modify in-place (zero-copy)
    pub_->publish(std::move(msg));
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

}  // namespace my_package

// Register macro — required for component loading
RCLCPP_COMPONENTS_REGISTER_NODE(my_package::ImageProcessor)
```

## CMakeLists.txt
```cmake
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# Build as shared library
add_library(image_processor SHARED src/image_processor.cpp)
ament_target_dependencies(image_processor rclcpp rclcpp_components sensor_msgs)

# Register the component
rclcpp_components_register_nodes(image_processor "my_package::ImageProcessor")

# Also create a standalone executable (optional)
rclcpp_components_register_node(image_processor
  PLUGIN "my_package::ImageProcessor"
  EXECUTABLE image_processor_node)

install(TARGETS image_processor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```

## Loading Components via Launch File
```python
from launch_ros.actions import ComposableNodeContainer, LoadComposableNode
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',  # single-threaded
        # executable='component_container_mt',  # multi-threaded
        composable_node_descriptions=[
            ComposableNode(
                package='my_package',
                plugin='my_package::ImageProcessor',
                name='processor1',
                parameters=[{'use_sim_time': True}],
                remappings=[('image_raw', '/camera/image')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='my_package',
                plugin='my_package::ImageDetector',
                name='detector1',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    return LaunchDescription([container])
```

## Loading Components Dynamically
```python
# Add a component to an already-running container
load_node = LoadComposableNode(
    composable_node=ComposableNode(
        package='my_package',
        plugin='my_package::ImageProcessor',
        name='processor2',
    ),
    target_container='my_container',
)
```

## CLI Component Loading
```bash
# Start an empty container
ros2 run rclcpp_components component_container

# Load a component into it
ros2 component load /ComponentManager my_package my_package::ImageProcessor

# List loaded components
ros2 component list /ComponentManager

# Unload a component
ros2 component unload /ComponentManager 1
```

## Container Executors
| Container | Executor | Use Case |
|-----------|----------|----------|
| `component_container` | SingleThreadedExecutor | Low-latency, deterministic ordering |
| `component_container_mt` | MultiThreadedExecutor | Concurrent callbacks, I/O-heavy nodes |
| `component_container_isolated` | Dedicated thread per node | Isolation between components |

## Python Composable Nodes
Python does not have the same native component system, but you can achieve similar results:
```python
# Manual composition in a single process
import rclpy
from rclpy.executors import SingleThreadedExecutor

rclpy.init()
executor = SingleThreadedExecutor()

node1 = ImageProcessor()
node2 = ImageDetector()

executor.add_node(node1)
executor.add_node(node2)

try:
    executor.spin()
finally:
    rclpy.shutdown()
```
Note: Python intra-process communication is NOT supported as of Jazzy. Python nodes in the same process still serialize through DDS.

## Critical Warnings
- **Constructor signature**: Components MUST accept `const rclcpp::NodeOptions &options`. Without this, the component loader cannot instantiate the node.
- **RCLCPP_COMPONENTS_REGISTER_NODE**: This macro MUST appear at the bottom of the source file with the fully qualified class name. Missing this = invisible component.
- **Shared library**: Components must be built as `SHARED` libraries, not executables. The container dlopens them.
- **Intra-process requires unique_ptr**: For zero-copy, use `UniquePtr` in subscription callbacks and `publish(std::move(msg))`. SharedPtr subscriptions force a copy.
- **One publisher per topic for intra-process**: Intra-process comms only works with a single publisher per topic in the same container. Multiple publishers fall back to DDS.
- **Recording with rosbag2**: Intra-process messages bypass DDS and are invisible to `ros2 bag record` unless you also have an inter-process subscriber.
- **Parameter namespacing**: Components in the same container each have their own parameter namespace (based on node name), so no conflicts.
