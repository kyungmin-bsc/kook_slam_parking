<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: message-filters
description: 'Use ROS 2 message_filters for time synchronization. Use when synchronizing multiple topic callbacks with ApproximateTimeSynchronizer.'
---

# Role
You are an expert in ROS 2 message_filters for time-synchronized multi-topic subscription. You guide correct usage of TimeSynchronizer, ApproximateTimeSynchronizer, and related filters in ROS 2 Jazzy/Rolling.

## When to Use message_filters
Use when you need to correlate messages from multiple topics that arrive at approximately the same time:
- Camera image + depth image
- Image + laser scan
- IMU + GPS
- Multiple sensor readings that must be processed together

## ExactTime Synchronization (Python)
```python
import message_filters
from sensor_msgs.msg import Image, CameraInfo

class SyncNode(Node):
    def __init__(self):
        super().__init__('sync_node')

        # Create message_filters subscribers
        self.image_sub = message_filters.Subscriber(self, Image, 'camera/image_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, 'camera/camera_info')

        # Exact time synchronizer — timestamps must match exactly
        self.sync = message_filters.TimeSynchronizer(
            [self.image_sub, self.info_sub], queue_size=10)
        self.sync.registerCallback(self.synced_callback)

    def synced_callback(self, image_msg, info_msg):
        self.get_logger().info(
            f'Synced: image stamp={image_msg.header.stamp.sec}, '
            f'info stamp={info_msg.header.stamp.sec}')
```

## ApproximateTime Synchronization (Python)
```python
class ApproxSyncNode(Node):
    def __init__(self):
        super().__init__('approx_sync_node')

        self.image_sub = message_filters.Subscriber(self, Image, 'image')
        self.depth_sub = message_filters.Subscriber(self, Image, 'depth')
        self.scan_sub = message_filters.Subscriber(self, LaserScan, 'scan')

        # Approximate time sync — allows timestamp differences up to slop
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.scan_sub],
            queue_size=10,
            slop=0.1  # seconds — max allowed timestamp difference
        )
        self.sync.registerCallback(self.callback)

    def callback(self, image, depth, scan):
        self.get_logger().info('Got synchronized sensor data')
        # Process image, depth, and scan together
```

## C++ TimeSynchronizer
```cpp
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using sensor_msgs::msg::Image;
using sensor_msgs::msg::LaserScan;

class SyncNode : public rclcpp::Node {
  message_filters::Subscriber<Image> image_sub_;
  message_filters::Subscriber<LaserScan> scan_sub_;

  // Exact sync
  using ExactSync = message_filters::TimeSynchronizer<Image, LaserScan>;
  std::shared_ptr<ExactSync> exact_sync_;

public:
  SyncNode() : Node("sync_node") {
    image_sub_.subscribe(this, "image");
    scan_sub_.subscribe(this, "scan");

    exact_sync_ = std::make_shared<ExactSync>(image_sub_, scan_sub_, 10);
    exact_sync_->registerCallback(&SyncNode::callback, this);
  }

  void callback(const Image::ConstSharedPtr &img, const LaserScan::ConstSharedPtr &scan) {
    RCLCPP_INFO(get_logger(), "Synced data received");
  }
};
```

## C++ ApproximateTime Synchronizer
```cpp
using ApproxPolicy = message_filters::sync_policies::ApproximateTime<Image, LaserScan>;
using ApproxSync = message_filters::Synchronizer<ApproxPolicy>;

std::shared_ptr<ApproxSync> approx_sync_;

// In constructor:
approx_sync_ = std::make_shared<ApproxSync>(
    ApproxPolicy(10),  // queue_size
    image_sub_, scan_sub_);
approx_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
approx_sync_->registerCallback(&SyncNode::callback, this);
```

## QoS with message_filters
```python
from rclpy.qos import qos_profile_sensor_data

# Python: pass QoS via subscriber
self.image_sub = message_filters.Subscriber(
    self, Image, 'image',
    qos_profile=qos_profile_sensor_data
)
```
```cpp
// C++: pass QoS via subscribe
image_sub_.subscribe(this, "image", rclcpp::SensorDataQoS().get_rmw_qos_profile());
```

## Cache Filter
```python
# Cache stores recent messages for later retrieval
cache = message_filters.Cache(self.image_sub, cache_size=100)

# Get the message closest to a given time
msg = cache.getElemBeforeTime(target_time)
msg = cache.getElemAfterTime(target_time)
```

## Subscriber with Callback Groups
```python
# For use with MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

cb_group = ReentrantCallbackGroup()
self.image_sub = message_filters.Subscriber(
    self, Image, 'image',
    qos_profile=10,
    callback_group=cb_group
)
```

## Tuning Parameters

### slop (ApproximateTimeSynchronizer)
- **Too small** (e.g., 0.001): Messages rarely match, most are dropped.
- **Too large** (e.g., 1.0): Old messages paired with new ones, stale data.
- **Recommended**: Start with `0.1` (100ms) and tune based on sensor rates.

### queue_size
- **Too small** (e.g., 1): If one topic arrives slightly before another, the first message is dropped before the second arrives.
- **Too large** (e.g., 1000): Memory waste, potential latency.
- **Recommended**: `5-20` for sensors publishing at 10-30 Hz.

## Debugging Sync Issues
```python
# Check individual topic rates
# ros2 topic hz /image
# ros2 topic hz /scan

# If callback never fires:
# 1. Check that topics are actually publishing
# 2. Check QoS compatibility (BEST_EFFORT vs RELIABLE)
# 3. Increase slop for ApproximateTimeSynchronizer
# 4. Increase queue_size
# 5. Verify headers have valid timestamps (not zero)
```

## Critical Warnings
- **Header timestamps required**: message_filters uses `msg.header.stamp` for synchronization. Messages WITHOUT headers (e.g., `std_msgs/String`) cannot be time-synchronized. Use headerless alternatives or wrap in a stamped message.
- **Dropped messages with tight slop**: If your slop is 10ms but sensors have 50ms jitter, most messages are silently dropped and the callback rarely fires.
- **Queue overflow**: With small queue_size and asynchronous sensor rates, one slow sensor causes the fast sensor's queue to overflow, dropping messages before they can be paired.
- **Memory**: `KEEP_ALL` + large cache = unbounded memory. Always use bounded queues.
- **Order sensitivity**: `TimeSynchronizer` (exact) requires timestamps to match exactly. This only works when a single sensor driver stamps multiple outputs with the same time (e.g., camera driver publishing image + info).
- **Performance**: Each synchronizer adds latency equal to approximately the slop time. For real-time applications, minimize slop.
- **Python API differences**: In rclpy, `message_filters.Subscriber` wraps `create_subscription` internally. Don't create a separate `create_subscription` for the same topic — you'll get duplicate callbacks.
