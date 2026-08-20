<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: micro-ros-setup
description: 'Set up micro-ROS on microcontrollers. Use when running ROS 2 publishers/subscribers directly on Teensy, ESP32, or STM32.'
---

# micro-ROS on Microcontrollers for ROS 2 Integration

## What micro-ROS Provides

micro-ROS runs a real ROS 2 node directly on a microcontroller. The MCU participates in the ROS 2 graph—publishing and subscribing to standard message types, discoverable via `ros2 topic list`, with QoS support. It uses the XRCE-DDS (eXtremely Resource Constrained Environments) protocol, which is a lightweight DDS implementation that communicates through an agent running on the host.

## Architecture

```
MCU (micro-ROS client)  ←── serial/UDP/USB ──→  micro_ros_agent (host)  ←── DDS ──→  ROS 2 network
```

The micro-ROS Agent bridges XRCE-DDS to full DDS. It is a standalone executable:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 1000000
# or for UDP (ESP32 WiFi):
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## Supported Platforms

| Board | Transport | Notes |
|-------|-----------|-------|
| Teensy 4.x | USB Serial | Best performance for serial transport |
| ESP32 | WiFi (UDP), Serial | WiFi adds latency (~5-20ms) |
| STM32 (F4, F7, H7) | Serial, USB | Requires STM32CubeMX configuration |
| Arduino Portenta H7 | Serial, WiFi | Dual-core M7+M4 |
| Raspberry Pi Pico | Serial | RP2040, limited RAM |

## Setup with PlatformIO

### platformio.ini

```ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino
lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio
board_microros_transport = serial
board_microros_distro = jazzy
monitor_speed = 1000000
```

After first build, PlatformIO downloads and compiles the micro-ROS libraries (this takes several minutes). Subsequent builds are fast.

### Minimal Publisher Example

```cpp
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    msg.data++;
    rcl_publish(&publisher, &msg, NULL);
}

void setup() {
    Serial.begin(1000000);
    set_microros_serial_transports(Serial);
    delay(2000);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "mcu_node", "", &support);
    rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "mcu/counter");
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback);
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    msg.data = 0;
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
```

## Creating Entities

### Publisher
```cpp
rclc_publisher_init_default(&publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data_raw");
```

### Subscriber
```cpp
void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    float linear = msg->linear.x;
    float angular = msg->angular.z;
    // Set motor speeds
}

rclc_subscription_init_default(&subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
rclc_executor_add_subscription(&executor, &subscriber, &twist_msg,
    &cmd_vel_callback, ON_NEW_DATA);
```

### Service Server
```cpp
rclc_service_init_default(&service, &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "/mcu/enable_motors");
```

## Memory Management

micro-ROS uses static memory allocation by default—no `malloc` at runtime. You pre-configure:

- Maximum number of nodes, publishers, subscribers, timers, services
- Message buffer sizes

For custom messages with dynamic arrays (e.g., `string` fields), you must pre-allocate buffers:
```cpp
// Pre-allocate a string field
char name_buf[64];
msg.name.data = name_buf;
msg.name.capacity = sizeof(name_buf);
msg.name.size = 0;
```

## XRCE-DDS Protocol Details

- Each entity (node, publisher, etc.) gets a unique 16-bit ID on the agent
- Messages are serialized using CDR (Common Data Representation), same as full DDS
- Heartbeat mechanism: agent pings client periodically; if client doesn't respond, agent cleans up entities
- Reconnection: if the agent restarts, the MCU must re-create all entities (handle `RCL_RET_ERROR` from spin)

## Limitations

- **Entity count**: Limited by available RAM. Teensy 4.1 can handle ~10-15 entities comfortably.
- **QoS**: Only `RELIABLE` and `BEST_EFFORT` reliability. No `TRANSIENT_LOCAL` durability for most transports.
- **No actions**: Action servers/clients are not supported. Use services + publishers as a workaround.
- **No parameters**: Parameter server is not supported on the MCU.
- **Message types**: Must be pre-compiled. Adding custom messages requires rebuilding the micro-ROS library.
- **Latency**: Serial transport adds ~1-2ms per message. UDP adds more due to WiFi.

## When to Use micro-ROS

**Use micro-ROS when:**
- You want the MCU to be a first-class ROS 2 citizen (visible in `ros2 node list`, `ros2 topic echo`)
- You publish standard message types (Imu, LaserScan, JointState) and want zero custom protocol work
- Your MCU has sufficient RAM (>256 KB) and you have few entities

**Use a custom serial protocol instead when:**
- You need tight real-time control loops (<1ms jitter)—micro-ROS `spin_some()` adds non-deterministic latency
- You need maximum throughput with minimal overhead
- You want simpler debugging (binary protocol is easier to inspect with a logic analyzer)
- RAM is very constrained (<128 KB)

## Agent Launch File Integration

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/teensy_main', '-b', '1000000'],
            output='screen',
        ),
    ])
```

## Troubleshooting

- **Agent shows "no client connected"**: Check baud rate match, serial port permissions, and that the MCU firmware calls `set_microros_serial_transports()` before any RCL calls.
- **Entity creation fails**: Insufficient memory. Reduce entity count or increase allocator pool.
- **Messages not appearing on `ros2 topic list`**: The agent must be running. Entities are only visible after the MCU creates them and the agent registers them with DDS.
- **Reconnection after agent restart**: The MCU must detect the disconnect (spin returns error) and reinitialize all RCL entities.
