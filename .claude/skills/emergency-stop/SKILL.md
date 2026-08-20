<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: emergency-stop
description: 'Implement emergency stop (e-stop) systems. Use when designing NC hardware e-stop, software safety layers, or BT e-stop integration.'
---

# Implementing Emergency Stop (E-Stop) Systems

## E-Stop Design Principles

An e-stop system must satisfy two requirements: (1) stop the robot immediately when activated, and (2) prevent the robot from moving until the e-stop is explicitly cleared. The system must work even when software has crashed.

## Layer 1: Physical E-Stop (Hardware)

The most critical safety layer. A physical button that breaks the motor power circuit directly.

### Circuit Design

```
Battery+ ─── [E-Stop Button (NC)] ─── Motor Driver V+
                    │
            GPIO input to MCU (optional, for detection)
```

- **Normally Closed (NC)**: The button's contacts are closed during normal operation and open when pressed. This is fail-safe: if the wire breaks or the button connector comes loose, the circuit opens and motors stop.
- **Latching mechanism**: Use a push-pull or twist-release button. Once pressed, it stays engaged until deliberately released. Prevents accidental resumption.
- **Current rating**: The button must handle the motor circuit's peak current. For high-current systems (>10A), use the e-stop button to control a relay or contactor that carries the motor current.

### Relay-Based E-Stop for High Current

```
Battery+ ─── [Main Relay (30A)] ─── Motor Driver V+
                    │
              Relay coil powered through:
                    │
Battery+ ─── [E-Stop Button (NC)] ─── [Relay Coil] ─── GND
```

When the e-stop is pressed, the relay coil de-energizes, opening the main power contacts. The relay must be rated for the motor's peak inrush current.

### What the E-Stop Should NOT Cut

- **Computer power**: The SBC must remain powered to log the e-stop event, report status, and coordinate recovery.
- **Sensor power**: LIDAR, cameras should continue operating for situational awareness.
- **MCU power**: The MCU should detect the e-stop state and prevent re-enabling motors until conditions are met.

## Layer 2: MCU E-Stop Detection

Wire the e-stop button state to a MCU GPIO:

```cpp
const int ESTOP_PIN = 22;       // reads HIGH when e-stop engaged (NC open)
volatile bool estopActive = false;

void setup() {
    pinMode(ESTOP_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, CHANGE);
}

void estopISR() {
    estopActive = digitalRead(ESTOP_PIN);
    if (estopActive) {
        // Immediate motor stop — even if main loop is blocked
        analogWrite(MOTOR_PWM_L, 0);
        analogWrite(MOTOR_PWM_R, 0);
    }
}

void loop() {
    if (estopActive) {
        // Refuse all motor commands
        // Send e-stop status to host
        return;
    }
    // Normal operation
}
```

The MCU serves as a secondary confirmation: even if the relay somehow fails to cut power, the MCU will not send drive signals.

## Layer 3: Software E-Stop (ROS 2)

### E-Stop Topic

```python
# Publisher: any safety node can trigger
from std_msgs.msg import Bool

self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
# latched via transient local QoS so late subscribers see the state
```

Multiple nodes can publish to `/emergency_stop`:
- Physical e-stop detector (via MCU serial data)
- Collision monitor (obstacle too close)
- Battery monitor (critical voltage)
- Communication watchdog (serial timeout)
- Navigation watchdog (recovery behavior failed)

### E-Stop Subscriber in Velocity Multiplexer

```yaml
# twist_mux configuration
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
      estop:
        topic: /cmd_vel_estop
        timeout: 1.0
        priority: 100    # highest priority overrides all others
```

An e-stop handler node subscribes to `/emergency_stop` and publishes zero `Twist` on `/cmd_vel_estop`:

```python
class EStopHandler(Node):
    def __init__(self):
        super().__init__('estop_handler')
        self.estop_active = False
        
        self.estop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.estop_callback, 
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1))
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_estop', 10)
        self.timer = self.create_timer(0.05, self.publish_stop)  # 20 Hz

    def estop_callback(self, msg: Bool):
        if msg.data and not self.estop_active:
            self.get_logger().error('EMERGENCY STOP ACTIVATED')
        self.estop_active = msg.data

    def publish_stop(self):
        if self.estop_active:
            self.cmd_pub.publish(Twist())  # all zeros
```

## Layer 4: Nav2 Collision Monitor

Nav2's `collision_monitor` provides a software safety boundary. It subscribes to sensor data and modifies or blocks `/cmd_vel` based on proximity:

```yaml
collision_monitor:
  ros__parameters:
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "/cmd_vel_nav"
    cmd_vel_out_topic: "/cmd_vel_safe"
    transform_tolerance: 0.3
    source_timeout: 2.0
    stop_pub_timeout: 1.0
    
    polygons: ["stop_zone", "slow_zone"]
    stop_zone:
      type: "polygon"
      points: "[[0.3, 0.25], [0.3, -0.25], [-0.1, -0.25], [-0.1, 0.25]]"
      action_type: "stop"         # immediately zero velocity
    slow_zone:
      type: "polygon"  
      points: "[[0.6, 0.35], [0.6, -0.35], [-0.2, -0.35], [-0.2, 0.35]]"
      action_type: "slowdown"
      max_points: 3
      slowdown_ratio: 0.3        # scale velocity to 30%
    
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "/scan"
```

## Behavior Tree Integration

### ReactiveSequence for E-Stop Checking

```xml
<BehaviorTree ID="MainTree">
  <ReactiveSequence>
    <!-- Check e-stop BEFORE every tick of child nodes -->
    <Condition ID="IsNotEStopActive" topic="/emergency_stop"/>
    
    <Sequence>
      <!-- Normal navigation behavior -->
      <Action ID="NavigateToPose" goal="{goal}"/>
      <Action ID="Wait" wait_duration="5"/>
    </Sequence>
  </ReactiveSequence>
</BehaviorTree>
```

`ReactiveSequence` re-checks `IsNotEStopActive` on every tick. If e-stop activates mid-navigation, the navigation action is immediately halted.

## Recovery from E-Stop

E-stop release must be deliberate, not automatic:

1. Physical e-stop button released (twist/pull)
2. MCU detects button state change, sends status to host
3. ROS 2 node receives "e-stop cleared" but does NOT immediately publish `False` on `/emergency_stop`
4. **Require explicit acknowledgment**: operator must call a service or press a button in the UI

```python
from std_srvs.srv import Trigger

class EStopManager(Node):
    def __init__(self):
        super().__init__('estop_manager')
        self.estop_active = False
        self.hardware_estop = False
        
        self.ack_srv = self.create_service(
            Trigger, '/estop/acknowledge', self.acknowledge_callback)

    def acknowledge_callback(self, request, response):
        if self.hardware_estop:
            response.success = False
            response.message = 'Hardware e-stop still engaged'
        else:
            self.estop_active = False
            self.publish_estop_state(False)
            response.success = True
            response.message = 'E-stop cleared, robot ready'
            self.get_logger().info('E-stop acknowledged and cleared')
        return response
```

## Logging and Diagnostics

Every e-stop event must be logged with full context:

```python
def estop_callback(self, msg: Bool):
    if msg.data and not self.estop_active:
        self.get_logger().error(
            'E-STOP ACTIVATED | source: %s | position: (%.2f, %.2f) | '
            'velocity: (%.2f, %.2f) | battery: %.1fV',
            self.last_trigger_source,
            self.current_x, self.current_y,
            self.current_vx, self.current_omega,
            self.battery_voltage)
```

Publish e-stop status to `/diagnostics` for monitoring tools:

```python
status = DiagnosticStatus()
status.name = 'Safety: E-Stop'
status.level = DiagnosticStatus.ERROR if self.estop_active else DiagnosticStatus.OK
status.message = 'E-STOP ACTIVE' if self.estop_active else 'Normal'
status.values = [
    KeyValue(key='hardware_estop', value=str(self.hardware_estop)),
    KeyValue(key='software_estop', value=str(self.estop_active)),
    KeyValue(key='last_trigger_time', value=str(self.last_trigger_time)),
    KeyValue(key='trigger_source', value=self.last_trigger_source),
]
```

## Complete Safety Chain Summary

```
Priority (highest first):
  1. Physical e-stop button → cuts motor power (hardware, no software)
  2. MCU e-stop detection → zeros PWM outputs (firmware, ISR)
  3. MCU serial watchdog → zeros motors on comms timeout (firmware)
  4. ROS 2 e-stop handler → publishes zero velocity on highest-priority mux channel
  5. Nav2 collision_monitor → modifies/blocks velocity near obstacles
  6. Nav2 recovery behaviors → attempt to escape stuck states before giving up
  7. Navigation watchdog → triggers e-stop if recovery fails repeatedly
```

Each layer is independent. A failure in any single layer is caught by the next.
