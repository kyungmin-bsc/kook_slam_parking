<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: qos-profiles
description: 'Configure ROS 2 Quality of Service profiles. Use when setting reliability, durability, history depth, or deadline policies.'
---

# Role
You are an expert in ROS 2 Quality of Service (QoS) profiles. You guide correct QoS configuration for reliable communication, sensor data, and debugging QoS mismatches in ROS 2 Jazzy/Rolling.

## QoS Policy Dimensions

| Policy | Options | Default |
|--------|---------|---------|
| **Reliability** | `RELIABLE` (retransmit), `BEST_EFFORT` (UDP-like) | `RELIABLE` |
| **Durability** | `TRANSIENT_LOCAL` (late-joining gets last), `VOLATILE` (no history) | `VOLATILE` |
| **History** | `KEEP_LAST(N)`, `KEEP_ALL` | `KEEP_LAST(10)` |
| **Depth** | Queue depth for KEEP_LAST | 10 |
| **Lifespan** | Auto-expire messages older than duration | Infinite |
| **Deadline** | Expected min publish rate; miss triggers event | Infinite |
| **Liveliness** | `AUTOMATIC`, `MANUAL_BY_NODE`, `MANUAL_BY_TOPIC` | `AUTOMATIC` |

## Common Profiles

### System Default (topics, services)
```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

default_qos = QoSProfile(depth=10)
# Reliability: RELIABLE, Durability: VOLATILE, History: KEEP_LAST(10)
```

### Sensor Data Profile
```python
from rclpy.qos import qos_profile_sensor_data
# Reliability: BEST_EFFORT, Durability: VOLATILE, History: KEEP_LAST(5)
# Use for: laser scans, IMU, camera images — high frequency, loss tolerable
```

### Latched Topic (TRANSIENT_LOCAL)
```python
latched_qos = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE
)
# Use for: map data, robot description, static transforms
# Late subscribers receive the last published message
```

### Best Effort Subscriber for Sensor Data
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

sensor_sub_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=5
)
sub = self.create_subscription(LaserScan, 'scan', callback, sensor_sub_qos)
```

## C++ QoS
```cpp
#include "rclcpp/qos.hpp"

// Sensor data
auto sub = create_subscription<LaserScan>("scan", rclcpp::SensorDataQoS(), callback);

// Custom
rclcpp::QoS qos(10);
qos.reliable();
qos.transient_local();
auto pub = create_publisher<OccupancyGrid>("map", qos);

// System default
auto pub = create_publisher<String>("topic", 10);  // shorthand for depth=10 default
```

## QoS Compatibility Rules

A subscription connects to a publisher ONLY if QoS is compatible:

| Publisher | Subscriber | Compatible? |
|-----------|------------|-------------|
| RELIABLE | RELIABLE | Yes |
| RELIABLE | BEST_EFFORT | Yes |
| BEST_EFFORT | BEST_EFFORT | Yes |
| BEST_EFFORT | RELIABLE | **NO** — subscriber demands guarantees publisher can't provide |
| TRANSIENT_LOCAL | TRANSIENT_LOCAL | Yes |
| TRANSIENT_LOCAL | VOLATILE | Yes |
| VOLATILE | TRANSIENT_LOCAL | **NO** — subscriber expects history publisher doesn't keep |

## Debugging QoS Mismatches
```bash
# Check QoS on a topic
ros2 topic info /scan --verbose

# Shows publisher and subscriber QoS profiles
# Look for "Requested incompatible QoS" warnings in logs

# Common error message:
# [WARN] New subscription discovered on topic '/scan', requesting incompatible QoS.
# No messages will be sent to it.
```

### Incompatible QoS Event Callback (C++)
```cpp
rclcpp::SubscriptionOptions options;
options.event_callbacks.incompatible_qos_callback =
  [this](rclcpp::QOSRequestedIncompatibleQoSInfo &info) {
    RCLCPP_WARN(get_logger(), "Incompatible QoS on subscription! Policy: %d", 
                info.last_policy_kind);
  };
sub_ = create_subscription<Msg>("topic", qos, callback, options);
```

## qos_overrides Parameter
In Jazzy/Rolling, you can override QoS at runtime via parameters:
```yaml
my_node:
  ros__parameters:
    qos_overrides:
      /scan:
        subscription:
          reliability: best_effort
          depth: 5
```

## Standard Topic QoS Conventions
| Topic Type | Reliability | Durability | Depth |
|------------|-------------|------------|-------|
| `/scan` (LaserScan) | BEST_EFFORT | VOLATILE | 5 |
| `/image_raw` (Image) | BEST_EFFORT | VOLATILE | 1-5 |
| `/odom` (Odometry) | BEST_EFFORT | VOLATILE | 10 |
| `/cmd_vel` (Twist) | RELIABLE | VOLATILE | 1-10 |
| `/map` (OccupancyGrid) | RELIABLE | TRANSIENT_LOCAL | 1 |
| `/tf` | RELIABLE | VOLATILE | 100 |
| `/tf_static` | RELIABLE | TRANSIENT_LOCAL | 100 |
| `/robot_description` | RELIABLE | TRANSIENT_LOCAL | 1 |

## Critical Warnings
- **Most common ROS 2 connectivity issue**: A subscriber uses RELIABLE but the publisher uses BEST_EFFORT. The subscriber silently receives NOTHING. Always check with `ros2 topic info --verbose`.
- **Depth matters for KEEP_LAST**: If your subscriber callback is slow and the publisher is fast, messages are dropped when the queue fills. Increase depth or use KEEP_ALL (with caution — memory!).
- **TRANSIENT_LOCAL publisher + VOLATILE subscriber**: This works, but the subscriber won't get the "latched" historical message. Both must be TRANSIENT_LOCAL for late-joining to work.
- **Service QoS**: Services in ROS 2 use RELIABLE + VOLATILE by default. You generally shouldn't change service QoS.
- **rosbag2 QoS**: `ros2 bag record` uses the publisher's QoS. On playback, it re-publishes with the recorded QoS. Override with `--qos-profile-overrides-path`.
- **Do not use KEEP_ALL casually**: It can cause unbounded memory growth if the subscriber can't keep up.
