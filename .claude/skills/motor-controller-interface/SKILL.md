<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: motor-controller-interface
description: 'Interface with motor controllers (RoboClaw, ODrive). Use when configuring packet serial, velocity PID, or encoder feedback.'
---

# Interfacing with Motor Controllers in ROS 2

## Motor Controller Landscape

| Controller | Interface | PID | Encoder Input | Motors | Use Case |
|-----------|-----------|-----|---------------|--------|----------|
| RoboClaw 2x7A/2x15A/2x30A/2x45A | Packet Serial, USB, RC | Yes (velocity + position) | 2x quadrature | 2 DC brushed | Differential drive, dual-motor |
| ODrive v3.6 / S1 / Pro | CAN, USB, UART, SPI | Yes (velocity, position, torque) | Hall, incremental, absolute | 2 BLDC / brushed | High-precision, brushless |
| Pololu G2 / MC33926 | PWM + direction pin | No | No | 1-2 DC brushed | Simple, low-cost |
| Cytron MDD10A / MDD3A | PWM + direction | No | No | 2 DC brushed | Mid-power simple drive |

## RoboClaw: Packet Serial Protocol

### Serial Modes

- **Simple serial**: Single-byte speed commands (0-127 per motor). No feedback. Avoid.
- **Packet serial**: Structured commands with address byte and CRC-16 checksum. Use this.

Packet format:
```
[Address] [Command] [Data bytes...] [CRC16-MSB] [CRC16-LSB]
```

Default address: 0x80 (128). Configurable for multi-controller setups (0x80-0x87).

### CRC-16 Calculation

```python
def roboclaw_crc16(packet: bytes) -> int:
    crc = 0
    for byte in packet:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc
```

### Common Commands

| Command | Byte | Data | Description |
|---------|------|------|-------------|
| Drive M1 Speed | 35 | int32 speed, uint8[2] crc | Set M1 velocity (encoder counts/sec) |
| Drive M2 Speed | 36 | int32 speed, uint8[2] crc | Set M2 velocity (encoder counts/sec) |
| Drive M1/M2 Speed | 37 | int32 m1, int32 m2, uint8[2] crc | Set both simultaneously |
| Read Encoder M1 | 16 | — | Returns int32 count, uint8 status |
| Read Encoder M2 | 17 | — | Returns int32 count, uint8 status |
| Read Main Battery | 24 | — | Returns uint16 (voltage × 10) |
| Read Motor Currents | 49 | — | Returns int16 m1, int16 m2 (×100 mA) |
| Set Velocity PID M1 | 28 | uint32 P, I, D, QPPS | Tune PID for M1 |

### Python RoboClaw Driver

```python
import serial
import struct

class RoboClawDriver:
    def __init__(self, port: str, address: int = 0x80, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.addr = address

    def _send_command(self, cmd: int, data: bytes = b'') -> bool:
        packet = bytes([self.addr, cmd]) + data
        crc = self._crc16(packet)
        self.ser.write(packet + struct.pack('>H', crc))
        return True

    def _read_response(self, size: int) -> bytes:
        data = self.ser.read(size + 2)  # data + 2 byte CRC
        if len(data) < size + 2:
            raise TimeoutError("RoboClaw response timeout")
        payload, recv_crc = data[:size], struct.unpack('>H', data[size:])[0]
        calc_crc = self._crc16(bytes([self.addr]) + payload)  # verify
        if recv_crc != calc_crc:
            raise ValueError("CRC mismatch")
        return payload

    def _crc16(self, data: bytes) -> int:
        crc = 0
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    def set_speeds(self, m1_speed: int, m2_speed: int):
        """Set motor speeds in encoder counts/second."""
        data = struct.pack('>ii', m1_speed, m2_speed)
        self._send_command(37, data)

    def read_encoders(self) -> tuple:
        """Returns (m1_counts, m2_counts)."""
        self._send_command(16)
        m1_data = self._read_response(5)
        m1_counts = struct.unpack('>i', m1_data[:4])[0]

        self._send_command(17)
        m2_data = self._read_response(5)
        m2_counts = struct.unpack('>i', m2_data[:4])[0]
        return m1_counts, m2_counts

    def read_battery_voltage(self) -> float:
        """Returns battery voltage in volts."""
        self._send_command(24)
        data = self._read_response(2)
        raw = struct.unpack('>H', data)[0]
        return raw / 10.0

    def read_motor_currents(self) -> tuple:
        """Returns (m1_amps, m2_amps)."""
        self._send_command(49)
        data = self._read_response(4)
        m1, m2 = struct.unpack('>HH', data)
        return m1 / 100.0, m2 / 100.0
```

## Velocity PID Tuning on RoboClaw

The RoboClaw runs its own PID loop on-board at ~300 Hz. You send target velocity in counts/sec, and the controller tracks it using encoder feedback.

PID parameters: `P`, `I`, `D`, `QPPS` (max speed in quadrature pulses per second).

Tuning procedure:
1. Set `I = 0`, `D = 0`. Start with `P` low (e.g., `P = 1.0`).
2. Command a step velocity (e.g., 1000 counts/sec). Observe motor response.
3. Increase `P` until the motor responds quickly but doesn't oscillate.
4. Add `I` (start at `P/10`) to eliminate steady-state error.
5. Add small `D` only if there is overshoot.
6. Set `QPPS` to the maximum speed the motor can physically achieve (measure it by driving at 100% duty and reading the encoder rate).

## Watchdog/Timeout

The RoboClaw has a configurable serial timeout. If no valid command is received within the timeout period, both motors stop. **This is a critical safety feature.**

```python
# Command 14: Set serial timeout
# Value is in units of 100ms. So 10 = 1 second timeout.
# Set to 0 to disable (dangerous!).
self._send_command(14, struct.pack('>B', 5))  # 500ms timeout
```

Always configure a timeout. If the ROS 2 node crashes or the serial cable disconnects, the motors will stop automatically.

## ODrive Interface (CAN Bus)

ODrive uses CAN bus with a well-defined protocol. Each axis has a CAN node ID. Commands:

```
CAN ID = (node_id << 5) | command_id

Set_Input_Vel:  command_id = 0x00D, data = [float vel, float torque_ff]
Set_Input_Pos:  command_id = 0x00C, data = [float pos, int16 vel_ff, int16 torque_ff]
Get_Encoder_Estimates: command_id = 0x009, returns [float pos, float vel]
```

ODrive advantages: brushless motor support, FOC control, high bandwidth (8 kHz PID loop), integrated encoder interface, web-based configuration tool.

## Pololu Motor Drivers

For simple applications where the MCU handles PID:

```cpp
// PWM + Direction pin control
const int DIR_PIN = 6;
const int PWM_PIN = 7;

void setMotor(int speed) {  // speed: -255 to 255
    digitalWrite(DIR_PIN, speed >= 0 ? HIGH : LOW);
    analogWrite(PWM_PIN, abs(speed));
}
```

No protocol overhead, no CRC, no serial bus contention. The MCU must run its own PID loop using encoder feedback. This gives the most control but requires more firmware development.

## Choosing a Controller

- **RoboClaw**: Best for differential drive robots. Dual motor with built-in PID, encoder inputs, and battery monitoring. One device handles both motors. Packet serial is well-documented.
- **ODrive**: Best when you need brushless motors or very high performance. CAN bus allows long cable runs. More complex setup.
- **Pololu / bare H-bridge**: Best for simple projects, educational robots, or when the MCU already runs PID. Lowest cost and complexity.
