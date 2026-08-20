<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: topic-pub-sub
description: 'Implement ROS 2 topic publishers and subscribers. Use when setting up pub/sub communication with proper QoS.'
---

# Role
You are an expert in ROS 2 topic publisher/subscriber patterns. You guide correct pub/sub design, QoS selection, and message flow patterns in ROS 2 Jazzy/Rolling.

## Basic Publisher (Python)
```python
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello {self.count}'
        self.publisher_.publish(msg)
        self.count += 1
```

## Basic Subscriber (Python)
```python
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Heard: {msg.data}')
```

## Timer-Based Publishing Pattern
```python
# Preferred: decouple data production from publishing rate
class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_pub')
        self.pub = self.create_publisher(LaserScan, 'scan', 
            rclpy.qos.qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / 40.0, self.on_timer)  # 40 Hz

    def on_timer(self):
        msg = self.read_sensor()  # gather data
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
```

## Subscriber with QoS for Sensor Data
```python
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

# Using predefined profile
self.sub = self.create_subscription(
    LaserScan, 'scan', self.scan_cb, qos_profile_sensor_data)

# Custom QoS
custom_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=5
)
self.sub = self.create_subscription(Image, 'image', self.img_cb, custom_qos)
```

## Latched Topic (TRANSIENT_LOCAL)
```python
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# Publisher: latched — late subscribers get the last message
latched_qos = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE
)
self.map_pub = self.create_publisher(OccupancyGrid, 'map', latched_qos)

# Subscriber MUST also use TRANSIENT_LOCAL to receive latched messages
self.map_sub = self.create_subscription(
    OccupancyGrid, 'map', self.map_cb, latched_qos)
```

## C++ Publisher/Subscriber
```cpp
// Publisher
pub_ = create_publisher<std_msgs::msg::String>("topic", 10);
auto msg = std_msgs::msg::String();
msg.data = "hello";
pub_->publish(msg);

// Subscriber with lambda
sub_ = create_subscription<std_msgs::msg::String>(
    "topic", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Got: %s", msg->data.c_str());
    });

// Subscriber with member function
sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&MyNode::scan_callback, this, std::placeholders::_1));
```

## Multi-Topic Subscriber Pattern
```python
class Fusioner(Node):
    def __init__(self):
        super().__init__('fusioner')
        self._latest_scan = None
        self._latest_odom = None

        self.create_subscription(LaserScan, 'scan', self.scan_cb,
            qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.odom_cb,
            qos_profile_sensor_data)
        self.create_timer(0.1, self.process)

    def scan_cb(self, msg): self._latest_scan = msg
    def odom_cb(self, msg): self._latest_odom = msg

    def process(self):
        if self._latest_scan and self._latest_odom:
            # Fuse data
            pass
```
Note: For time-synchronized multi-topic fusion, prefer `message_filters` (see message_filters.md).

## Message Type Selection
| Data | Message Type | Package |
|------|-------------|---------|
| Velocity commands | `Twist` | geometry_msgs |
| Laser scans | `LaserScan` | sensor_msgs |
| Camera images | `Image` | sensor_msgs |
| Robot pose | `PoseStamped` | geometry_msgs |
| IMU | `Imu` | sensor_msgs |
| Maps | `OccupancyGrid` | nav_msgs |
| Point clouds | `PointCloud2` | sensor_msgs |
| Odometry | `Odometry` | nav_msgs |
| Wrench (force/torque) | `WrenchStamped` | geometry_msgs |
| Diagnostics | `DiagnosticArray` | diagnostic_msgs |

## Checking Topic Health
```bash
ros2 topic list
ros2 topic info /scan --verbose    # QoS, publishers, subscribers
ros2 topic hz /scan                # publishing rate
ros2 topic bw /scan                # bandwidth
ros2 topic echo /scan --once       # single message
ros2 topic type /scan              # message type
```

## Critical Warnings
- **Don't publish in __init__ before spin starts**: If you publish in the constructor, subscribers that connect later might miss the message (unless using TRANSIENT_LOCAL). Prefer timer-based publishing.
- **Store subscription references**: The subscription object MUST be stored as a class member (`self.sub = ...`). If it goes out of scope, the subscription is silently destroyed.
- **Callback queue depth**: With `KEEP_LAST(depth)`, if the callback is slow and the publisher is fast, older messages are dropped. Monitor with `ros2 topic hz` to detect this.
- **Large messages (Image, PointCloud2)**: Consider using compressed topics, shared memory transport (intra-process), or reducing resolution/rate to stay within bandwidth limits.
- **Topic name resolution**: Topic names starting with `/` are absolute. Without `/`, they are relative to the node's namespace. Use `~/topic` for private topics relative to the node name.
- **Header timestamps**: Always stamp messages using `self.get_clock().now().to_msg()` (not `time.time()`). This ensures sim_time compatibility.
