<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: servo-control
description: 'Control PWM servos from microcontrollers. Use when driving standard/continuous servos, implementing easing, or gripper control.'
---

# PWM Servo Control from Microcontrollers

## Standard Servo Signal

Standard RC servos expect a 50 Hz PWM signal (20ms period). The pulse width within each period determines the position:

- **1000 μs** (1ms): Fully one direction (e.g., 0°)
- **1500 μs** (1.5ms): Center position (90°)
- **2000 μs** (2ms): Fully other direction (180°)

Some servos accept a wider range (500-2500 μs) for extended travel. Always check the servo's datasheet.

## Teensy 4.1 Servo Control

### Using the Servo Library

```cpp
#include <Servo.h>

Servo gripperServo;
Servo panServo;

void setup() {
    gripperServo.attach(9);     // PWM-capable pin
    panServo.attach(10);
    
    gripperServo.writeMicroseconds(1500);  // center
    panServo.write(90);                     // center (angle mode)
}

void loop() {
    gripperServo.writeMicroseconds(1000);  // open
    delay(1000);
    gripperServo.writeMicroseconds(2000);  // close
    delay(1000);
}
```

`write(angle)` maps 0-180° to the servo's configured min/max microseconds. `writeMicroseconds(us)` gives direct pulse width control—preferred for precision.

### Direct PWM Configuration

For advanced control (non-servo PWM devices, LEDs, etc.):

```cpp
const int SERVO_PIN = 9;

void setup() {
    analogWriteFrequency(SERVO_PIN, 50);     // 50 Hz for servos
    analogWriteResolution(16);                // 16-bit resolution (0-65535)
}

void setServoPulse(int pin, float microseconds) {
    // At 50 Hz, period = 20000 μs
    // duty = (microseconds / 20000) * 65535
    uint16_t duty = (uint16_t)((microseconds / 20000.0) * 65535.0);
    analogWrite(pin, duty);
}
```

The Teensy 4.1's FlexPWM has 16-bit resolution at 50 Hz, giving ~0.3 μs per step—more than sufficient for servo control.

## Continuous Rotation Servos

Continuous rotation servos interpret the pulse width as speed and direction, not position:

| Pulse Width | Behavior |
|------------|----------|
| 1500 μs | Stop |
| < 1500 μs | Rotate one direction (speed increases as pulse decreases) |
| > 1500 μs | Rotate other direction (speed increases as pulse increases) |
| 1000 μs | Full speed direction A |
| 2000 μs | Full speed direction B |

Dead band: Most continuous rotation servos have a dead band around 1500 μs (±10-20 μs) where the motor does not spin. Center calibration may be needed via a trim pot on the servo.

## Gripper Control Pattern

### Open/Close with Position Feedback

```cpp
const int GRIPPER_PIN = 9;
const int GRIP_OPEN_US = 1000;
const int GRIP_CLOSED_US = 2000;
const int GRIP_STEP_US = 5;          // microseconds per step
const int GRIP_STEP_DELAY_MS = 10;   // delay between steps

Servo gripper;
int current_us = 1500;

void moveGripperTo(int target_us) {
    target_us = constrain(target_us, GRIP_OPEN_US, GRIP_CLOSED_US);
    while (current_us != target_us) {
        if (current_us < target_us) current_us += GRIP_STEP_US;
        else current_us -= GRIP_STEP_US;
        
        current_us = constrain(current_us, GRIP_OPEN_US, GRIP_CLOSED_US);
        gripper.writeMicroseconds(current_us);
        delay(GRIP_STEP_DELAY_MS);
    }
}
```

### Stall Detection

When a gripper grabs an object, the motor stalls and current spikes. Detect this to avoid servo burnout:

```cpp
const int CURRENT_PIN = A0;  // Current sense resistor output
const float STALL_THRESHOLD_MA = 800.0;
const unsigned long STALL_TIMEOUT_MS = 2000;

bool moveGripperWithStallDetect(int target_us) {
    unsigned long start = millis();
    while (current_us != target_us) {
        if (current_us < target_us) current_us += GRIP_STEP_US;
        else current_us -= GRIP_STEP_US;
        
        gripper.writeMicroseconds(current_us);
        delay(GRIP_STEP_DELAY_MS);
        
        float current_mA = analogRead(CURRENT_PIN) * (3300.0 / 4095.0) / 0.1;
        
        if (current_mA > STALL_THRESHOLD_MA) {
            return true;  // gripped something
        }
        if (millis() - start > STALL_TIMEOUT_MS) {
            return false;  // timeout, nothing gripped
        }
    }
    return false;
}
```

## Easing Functions for Smooth Motion

Abrupt servo movements cause mechanical shock and imprecise positioning. Use easing:

### Linear Interpolation (LERP)

```cpp
void smoothMove(Servo &servo, int from_us, int to_us, int duration_ms) {
    unsigned long start = millis();
    while (millis() - start < (unsigned long)duration_ms) {
        float t = (float)(millis() - start) / duration_ms;
        t = constrain(t, 0.0f, 1.0f);
        int pos = from_us + (int)((to_us - from_us) * t);
        servo.writeMicroseconds(pos);
        delay(5);
    }
    servo.writeMicroseconds(to_us);
}
```

### S-Curve (Smooth Start and Stop)

```cpp
float sCurve(float t) {
    // Hermite interpolation: 3t² - 2t³
    return t * t * (3.0f - 2.0f * t);
}

void smoothMoveScurve(Servo &servo, int from_us, int to_us, int duration_ms) {
    unsigned long start = millis();
    while (millis() - start < (unsigned long)duration_ms) {
        float t = (float)(millis() - start) / duration_ms;
        t = constrain(t, 0.0f, 1.0f);
        float eased = sCurve(t);
        int pos = from_us + (int)((to_us - from_us) * eased);
        servo.writeMicroseconds(pos);
        delay(5);
    }
    servo.writeMicroseconds(to_us);
}
```

## ROS 2 Integration

### Subscriber-Based Servo Controller

On the MCU, subscribe to a command topic and drive the servo:

```cpp
// MCU serial protocol: receive servo commands
// Packet type 0x03: payload = [channel(1), target_us(2)]
void handleServoCommand(uint8_t *payload, uint8_t len) {
    if (len != 3) return;
    uint8_t channel = payload[0];
    uint16_t target_us = payload[1] | (payload[2] << 8);
    target_us = constrain(target_us, 500, 2500);
    
    if (channel < NUM_SERVOS) {
        servos[channel].writeMicroseconds(target_us);
    }
}
```

On the ROS 2 side, a node subscribes to `sensor_msgs/JointState` and converts angle (radians) to microseconds:

```python
def joint_state_callback(self, msg: JointState):
    for i, name in enumerate(msg.name):
        if name == 'gripper_joint':
            # Map joint angle (0.0 to 1.57 rad) to pulse width (1000 to 2000 us)
            angle = msg.position[i]
            us = int(1000 + (angle / 1.5708) * 1000)
            self.send_servo_command(channel=0, microseconds=us)
```

## Current Limiting and Thermal Protection

- **Current limiting**: If the servo draws more than its rated current for >1 second, detach it (`servo.detach()`) to prevent burnout. Re-attach when the command changes.
- **PWM disable on idle**: If no command received for >5 seconds, detach the servo. This stops the PWM signal and allows the servo to go slack, preventing continuous current draw to hold position.
- **Voltage regulation**: Servos should have their own voltage regulator or BEC (Battery Eliminator Circuit), not share the MCU's 3.3V/5V rail. A stalled servo can draw 1-2A, which would brown out the MCU.
- **Decoupling**: Place 100μF electrolytic + 100nF ceramic capacitors on the servo power rail, as close to the servo connector as possible.
