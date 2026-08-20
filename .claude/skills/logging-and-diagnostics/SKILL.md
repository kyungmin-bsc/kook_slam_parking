<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: logging-and-diagnostics
description: 'Configure ROS 2 logging and diagnostics. Use when setting log levels, aggregating diagnostics, or publishing system health.'
---

# Role
You are an expert in ROS 2 logging and diagnostics. You guide correct use of logging APIs, log level configuration, and the diagnostic_updater system in ROS 2 Jazzy/Rolling.

## Python Logging
```python
# Standard logging levels
self.get_logger().debug('Debug detail')
self.get_logger().info('Informational message')
self.get_logger().warn('Warning condition')
self.get_logger().error('Error occurred')
self.get_logger().fatal('Fatal — node cannot continue')

# Throttled logging (interval in nanoseconds for rclpy, seconds for convenience)
self.get_logger().info('Status update', throttle_duration_sec=5.0)

# Once only
self.get_logger().warn('This prints only once', once=True)

# Conditional
self.get_logger().info('Only when enabled', 
    skip_first=True  # skip the first occurrence
)
```

## C++ Logging
```cpp
RCLCPP_DEBUG(this->get_logger(), "Debug: %d", val);
RCLCPP_INFO(this->get_logger(), "Info: %s", str.c_str());
RCLCPP_WARN(this->get_logger(), "Warning: %f", val);
RCLCPP_ERROR(this->get_logger(), "Error occurred");
RCLCPP_FATAL(this->get_logger(), "Fatal failure");

// Throttled (milliseconds)
RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Every 5s");
RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Every 10s");

// Once
RCLCPP_INFO_ONCE(this->get_logger(), "Printed once");

// Conditional
RCLCPP_INFO_EXPRESSION(this->get_logger(), enable_debug_, "Conditional msg");

// Skip first
RCLCPP_WARN_SKIPFIRST(this->get_logger(), "Skips first occurrence");

// Combined: throttle + skip first
RCLCPP_INFO_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "msg");
```

## Named/Child Loggers
```python
# Create a child logger for a subsystem
nav_logger = self.get_logger().get_child('navigation')
nav_logger.info('Path computed')  # Output: [my_node.navigation] Path computed
```
```cpp
auto nav_logger = this->get_logger().get_child("navigation");
RCLCPP_INFO(nav_logger, "Path computed");
```

## Setting Log Levels

### Via CLI
```bash
ros2 run my_pkg my_node --ros-args --log-level debug
ros2 run my_pkg my_node --ros-args --log-level my_node:=debug

# Per-logger
ros2 run my_pkg my_node --ros-args --log-level my_node.navigation:=warn
```

### Via Launch File
```python
Node(
    package='my_pkg', executable='my_node',
    arguments=['--ros-args', '--log-level', 'info'],
)
```

### At Runtime
```bash
ros2 service call /my_node/set_logger_level rcl_interfaces/srv/SetLoggerLevel \
  "{logger_name: 'my_node', level: 10}"
# Levels: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
```

## Environment Variables
```bash
# Enable colored output
export RCUTILS_COLORIZED_OUTPUT=1

# Set default console format
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"

# Enable buffered streaming (better performance)
export RCUTILS_LOGGING_BUFFERED_STREAM=1
```

## Diagnostic Updater (Python)
```python
from diagnostic_updater import Updater, FrequencyStatus, HeaderlessTopicDiagnostic
from diagnostic_msgs.msg import DiagnosticStatus

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.updater = Updater(self)
        self.updater.setHardwareID('motor_controller')

        # Custom diagnostic task
        self.updater.add('Battery', self.check_battery)

        # Frequency monitoring
        self.freq_diag = FrequencyStatus(
            FrequencyStatusParam({'min': 9.0, 'max': 11.0}),
            'sensor_rate'
        )
        self.updater.add(self.freq_diag)

    def check_battery(self, stat):
        voltage = self.read_battery()
        if voltage < 11.0:
            stat.summary(DiagnosticStatus.ERROR, 'Battery critical')
        elif voltage < 12.0:
            stat.summary(DiagnosticStatus.WARN, 'Battery low')
        else:
            stat.summary(DiagnosticStatus.OK, 'Battery normal')
        stat.add('Voltage', f'{voltage:.2f}V')
        stat.add('Percent', f'{self.battery_percent}%')
        return stat

    def sensor_callback(self, msg):
        self.freq_diag.tick()  # Mark received message for frequency tracking
```

## C++ Diagnostic Updater
```cpp
#include "diagnostic_updater/diagnostic_updater.hpp"

class MyNode : public rclcpp::Node {
  diagnostic_updater::Updater updater_;

public:
  MyNode() : Node("my_node"), updater_(this) {
    updater_.setHardwareID("motor_01");
    updater_.add("Motor Status", this, &MyNode::check_motor);
  }

  void check_motor(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    if (motor_temp_ > 80.0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Overheating");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Normal");
    }
    stat.add("Temperature", motor_temp_);
    stat.add("Current", motor_current_);
  }
};
```

## Diagnostic Aggregator
```yaml
# config/analyzers.yaml
diagnostic_aggregator:
  ros__parameters:
    analyzers:
      sensors:
        type: diagnostic_aggregator/AnalyzerGroup
        path: Sensors
        analyzers:
          lidar:
            type: diagnostic_aggregator/GenericAnalyzer
            path: LIDAR
            contains: ['lidar']
          imu:
            type: diagnostic_aggregator/GenericAnalyzer
            path: IMU
            contains: ['imu']
```
```bash
ros2 run diagnostic_aggregator aggregator_node --ros-args --params-file analyzers.yaml
```

## Viewing Diagnostics
```bash
ros2 topic echo /diagnostics
ros2 run rqt_robot_monitor rqt_robot_monitor  # GUI
```

## Critical Warnings
- **Excessive logging in tight loops**: `RCLCPP_INFO` inside a 1kHz callback generates enormous log output and significantly impacts performance. Use `_THROTTLE` or `_ONCE` variants.
- **Log level persistence**: Setting log level via CLI only affects that run. Use launch parameters for persistent configuration.
- **Diagnostic updater timing**: The updater publishes at ~1 Hz by default. Don't expect real-time diagnostic updates.
- **format string injection (C++)**: Never pass user-controlled strings as the format argument to RCLCPP_INFO. Use `%s` with the string as an argument.
- **Python logging vs rclpy logging**: Use `self.get_logger()`, NOT Python's built-in `logging` module. The rclpy logger integrates with ROS logging infrastructure, `rosout`, and log level services.
- **rosout topic**: All logs are also published on `/rosout`. High-frequency logging floods this topic and wastes bandwidth.
