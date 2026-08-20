<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: time-duration-rate
description: 'Work with ROS 2 time, duration, and rate. Use when handling simulation time, timers, or time-based logic.'
---

# Role
You are an expert in ROS 2 time, duration, rate, and clock handling. You guide correct time management including simulation time in ROS 2 Jazzy/Rolling.

## Clock Types
| Clock Type | Source | Use Case |
|------------|--------|----------|
| `ROS_TIME` | `/clock` topic (sim) or system | Default. Follows sim_time when enabled. |
| `SYSTEM_TIME` | OS wall clock | Always real time, ignores sim_time. |
| `STEADY_TIME` | Monotonic clock | For measuring elapsed time, never jumps backward. |

## Python Time and Duration
```python
from rclpy.time import Time, Duration
from builtin_interfaces.msg import Time as TimeMsg

# Current time (respects use_sim_time)
now = self.get_clock().now()  # rclpy.time.Time

# Create a specific time
t = Time(seconds=10, nanoseconds=500000000)  # 10.5 seconds

# Time(0) = "latest available" (special meaning in TF2)
latest = Time()  # or Time(seconds=0)

# Duration
d = Duration(seconds=1, nanoseconds=500000000)  # 1.5 seconds

# Arithmetic
future = now + Duration(seconds=5)
elapsed = now - past_time  # returns Duration
is_expired = (now - start_time) > Duration(seconds=10)

# Convert to message
time_msg = now.to_msg()  # builtin_interfaces/Time

# Convert from message
t_from_msg = Time.from_msg(time_msg)

# Convert to float seconds
seconds_float = now.nanoseconds / 1e9
```

## C++ Time and Duration
```cpp
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"

// Current time
auto now = this->get_clock()->now();  // rclcpp::Time

// Create time
rclcpp::Time t(10, 500000000, RCL_ROS_TIME);  // 10.5s

// Duration
rclcpp::Duration d(1, 500000000);  // 1.5s
rclcpp::Duration d2 = rclcpp::Duration::from_seconds(1.5);

// Arithmetic
auto future = now + rclcpp::Duration::from_seconds(5.0);
auto elapsed = now - past;
bool expired = (now - start) > rclcpp::Duration::from_seconds(10.0);

// To seconds (float)
double secs = now.seconds();

// To message
builtin_interfaces::msg::Time msg = now;
// or: auto msg = rclcpp::Time(now).to_msg(); (if explicit needed)
```

## Timers
```python
# Periodic timer (uses node clock — sim_time aware)
self.timer = self.create_timer(0.1, self.callback)  # 10 Hz

# Wall timer (always real time, ignores sim_time)
# In C++: create_wall_timer(100ms, callback)
# Python doesn't have create_wall_timer — use create_timer with a steady clock
```

```cpp
// sim-time-aware timer
timer_ = this->create_timer(100ms, callback);

// Wall timer (always real time)
timer_ = this->create_wall_timer(100ms, callback);
```

## use_sim_time Parameter
When `use_sim_time=true`, `node.get_clock().now()` returns the time from the `/clock` topic instead of the system time. This is essential for simulation and rosbag playback.

### Enabling in Launch
```python
Node(
    package='my_pkg', executable='my_node',
    parameters=[{'use_sim_time': True}]
)
# Or globally:
SetParameter('use_sim_time', 'true')
```

### Clock Topic
```bash
# Gazebo publishes /clock automatically
# rosbag2 publishes /clock with --clock flag
ros2 bag play recording.mcap --clock
```

## Rate (Python)
```python
from rclpy.rate import Rate

# Create a rate object — blocks for the remainder of the period
rate = self.create_rate(10)  # 10 Hz

# Use in a loop (must be in a separate thread, not in a callback)
while rclpy.ok():
    do_work()
    rate.sleep()  # sleeps until next period
```

## Stamping Messages
```python
# CORRECT: Use node clock for sim_time compatibility
msg.header.stamp = self.get_clock().now().to_msg()

# WRONG: Uses system time, breaks with simulation
import time
msg.header.stamp.sec = int(time.time())  # DON'T DO THIS
```

```cpp
msg.header.stamp = this->now();  // shorthand for get_clock()->now()
```

## Comparing and Checking Time
```python
# Check if a message is stale
msg_time = Time.from_msg(msg.header.stamp)
age = self.get_clock().now() - msg_time
if age > Duration(seconds=5.0):
    self.get_logger().warn('Stale message!')
```

## Clock Callback (React to Clock Updates)
```python
from rclpy.clock import ClockChange

def clock_callback(clock_change):
    # Called when the clock type or source changes
    pass

self.get_clock().set_ros_time_override(Time(seconds=0))
```

## Timeout Patterns
```python
# Service wait with timeout
if not self.client.wait_for_service(timeout_sec=5.0):
    self.get_logger().error('Timeout waiting for service')

# TF lookup with timeout
try:
    t = self.tf_buffer.lookup_transform('map', 'base_link',
        rclpy.time.Time(), timeout=Duration(seconds=1.0))
except TransformException:
    pass

# Action send_goal timeout via spin_until_future_complete
rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
if not future.done():
    self.get_logger().error('Action timed out')
```

## Critical Warnings
- **Mixing sim_time and wall time**: If some nodes use `use_sim_time=true` and others don't, TF transforms will have mismatched timestamps, causing `ExtrapolationException`. ALL nodes in a sim/playback session must use sim_time.
- **Time(0) vs now()**: In TF lookups, `Time(0)` means "latest available transform." Using `now()` requests the transform at the exact current time, which may not exist yet due to latency. Use `Time(0)` unless you need time-specific lookups.
- **create_rate in callbacks**: Do NOT use `rate.sleep()` inside a callback — it blocks the executor. Use timers instead.
- **Zero time before clock**: Before the first `/clock` message arrives (with `use_sim_time=true`), `get_clock().now()` returns time 0. This can cause division by zero or negative durations. Guard against this.
- **Timer jitter**: `create_timer` is not real-time guaranteed. Under heavy load, timer callbacks may be delayed. For hard real-time, use a dedicated real-time framework.
- **Duration sign**: `Duration` can be negative if you subtract a future time from a past time. Always check the sign when comparing durations.
