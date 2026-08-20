<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: hardware-watchdog
description: 'Implement hardware safety watchdogs. Use when adding serial timeout, MCU WDT, or multi-layer safety stop systems.'
---

# Hardware Watchdog Timers for Robot Safety

## The Problem

If the ROS 2 host crashes, the navigation stack segfaults, the serial cable gets unplugged, or a node deadlocks—the last motor command sent to the MCU will be executed indefinitely. A robot driving into a wall at full speed because the control loop died is a real failure mode. Watchdog timers at every layer prevent this.

## Layer 1: MCU Software Watchdog

The MCU maintains a timer that resets on every valid command received from the host. If the timer expires, all actuators are set to safe states.

### Implementation with IntervalTimer (Teensy 4.1)

```cpp
#include <Arduino.h>

volatile unsigned long lastCommandTime = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 500;
volatile bool watchdogTripped = false;

IntervalTimer watchdogTimer;

void checkWatchdog() {
    if (millis() - lastCommandTime > WATCHDOG_TIMEOUT_MS) {
        if (!watchdogTripped) {
            // Emergency stop all motors
            setMotorSpeed(0, 0);
            setAllServosNeutral();
            watchdogTripped = true;
        }
    }
}

void setup() {
    Serial.begin(1000000);
    watchdogTimer.begin(checkWatchdog, 50000);  // check every 50ms
    lastCommandTime = millis();
}

void onValidPacketReceived() {
    lastCommandTime = millis();
    watchdogTripped = false;  // clear trip flag on valid comms
}

void loop() {
    // Parse serial packets...
    if (parsePacket()) {
        onValidPacketReceived();
        processCommand();
    }
}
```

Critical design decisions:
- Check interval (50ms) is shorter than timeout (500ms) to catch expiration promptly.
- `watchdogTripped` flag prevents repeatedly calling stop functions.
- `onValidPacketReceived()` resets the timer—only valid, CRC-verified packets count.
- Invalid packets, CRC failures, and partial reads do NOT reset the watchdog.

### What "Safe State" Means

| Actuator | Safe State | Reason |
|----------|-----------|--------|
| Drive motors | Speed = 0 | Prevent runaway |
| Servos | Hold last position OR go to neutral | Depends on application |
| LEDs | Flash warning pattern | Visual indicator |
| Relay outputs | Open (de-energize) | Fail-safe for power switches |

## Layer 2: Hardware Watchdog Timer (WDT)

Many MCUs have a hardware WDT peripheral that resets the entire chip if not "kicked" within a deadline. This catches firmware lockups (infinite loops, hard faults, stack overflows) that a software watchdog cannot detect.

### Teensy 4.1 Watchdog

```cpp
#include "Watchdog_t4.h"

WDT_T4<WDT1> wdt;

void wdtCallback() {
    // Called just before reset—last chance to log
    // Keep this VERY short
}

void setup() {
    WDT_timings_t config;
    config.trigger = 3.0;    // warning callback at 3 seconds
    config.timeout = 5.0;    // hard reset at 5 seconds
    config.callback = wdtCallback;
    wdt.begin(config);
}

void loop() {
    wdt.feed();  // must call within timeout period
    
    // Normal operation...
    parseSerial();
    runControlLoop();
}
```

If `loop()` takes longer than 5 seconds (due to a hang), the WDT resets the Teensy. After reset, the firmware reinitializes with motors stopped—safe state by default.

## Layer 3: ROS 2 Watchdog Node

On the host side, a dedicated node monitors that `/cmd_vel` is being published at the expected rate. If the controller node dies, this watchdog publishes zero velocity.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__('cmd_vel_watchdog')
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_safe')
        
        timeout = self.get_parameter('timeout_sec').value
        
        self.sub = self.create_subscription(
            Twist, self.get_parameter('input_topic').value,
            self.cmd_vel_callback, 10)
        self.pub = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)
        self.estop_pub = self.create_publisher(
            Bool, '/emergency_stop', 10)
        
        self.last_msg_time = self.get_clock().now()
        self.last_twist = Twist()
        self.timed_out = False
        
        self.timer = self.create_timer(0.05, self.check_timeout)  # 20 Hz

    def cmd_vel_callback(self, msg: Twist):
        self.last_msg_time = self.get_clock().now()
        self.last_twist = msg
        self.timed_out = False
        self.pub.publish(msg)

    def check_timeout(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        timeout = self.get_parameter('timeout_sec').value
        
        if elapsed > timeout and not self.timed_out:
            self.get_logger().warn(
                f'cmd_vel timeout ({elapsed:.2f}s), sending zero velocity')
            self.pub.publish(Twist())  # all zeros
            self.estop_pub.publish(Bool(data=True))
            self.timed_out = True
```

## Layer 4: Velocity Multiplexer Priority

Use `twist_mux` or a similar multiplexer to prioritize safety commands over navigation commands:

```yaml
# twist_mux params
twist_mux:
  ros__parameters:
    topics:
      navigation:
        topic: /cmd_vel_nav
        timeout: 0.5
        priority: 10
      teleop:
        topic: /cmd_vel_teleop
        timeout: 0.5
        priority: 20
      safety:
        topic: /cmd_vel_safe
        timeout: 0.5
        priority: 100     # highest priority
    lock_topic: /twist_mux/lock
```

When the safety watchdog publishes zero velocity on the safety topic, it overrides everything else.

## Layer 5: Physical Emergency Stop

A physical e-stop button is the last line of defense. It must:

- **Break the motor power circuit directly** — no software in the loop
- Use a normally-closed (NC) contact: pressing the button opens the circuit
- Be mounted in an accessible location on the robot exterior
- Use a latching button (push to stop, twist to release) so it stays engaged

The e-stop should cut power to motor drivers but NOT to the computer, sensors, or MCU. This allows the system to detect the e-stop state, log it, and recover gracefully after acknowledgment.

## Complete Safety Chain

```
Normal operation:
  Nav2 → /cmd_vel_nav → twist_mux → /cmd_vel → serial → MCU → motors

Watchdog cascade on failure:
  1. Nav2 stops publishing → twist_mux timeout → zero velocity
  2. ROS watchdog detects silence → publishes zero on safety channel
  3. MCU receives zero velocity → motors stop
  4. If serial also fails → MCU watchdog trips (500ms) → motors stop
  5. If MCU firmware hangs → hardware WDT resets MCU → motors stop (init state)
  6. If MCU power fails → motor driver has no signal → motors coast to stop
  7. Physical e-stop → power cut to motor drivers → motors stop immediately
```

## Heartbeat Protocol

Instead of relying on command packets to keep the watchdog alive, implement an explicit heartbeat:

```
Host → MCU: heartbeat packet (type 0xFF, empty payload) every 200ms
MCU: resets watchdog timer on heartbeat receipt
```

Advantages over using motor commands as keepalive:
- Works even when the robot is stationary (no `/cmd_vel` being published)
- Separates "communication is alive" from "robot should be moving"
- Heartbeat can carry status flags (host health, navigation state)

## Testing Watchdogs

1. **Kill the ROS 2 node**: `ros2 lifecycle set /controller_node shutdown` — verify motors stop within timeout
2. **Unplug USB cable**: Verify MCU watchdog trips and motors stop
3. **Infinite loop in firmware**: Inject a `while(1){}` in a test build — verify hardware WDT resets the MCU
4. **Press physical e-stop**: Verify motors lose power immediately, computer stays running
5. **Network partition** (for distributed systems): Disconnect Ethernet — verify all layers activate
