<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: serial-communication-protocol
description: 'Design serial communication protocols. Use when building packet-based UART communication between ROS 2 and microcontrollers.'
---

# Designing Robust Serial Protocols Between ROS 2 and Microcontrollers

## Why a Custom Protocol

Raw serial is a byte stream with no framing. Without a protocol, you cannot tell where one message ends and another begins, detect corruption from electrical noise, or recover from partial reads after a reconnection. A structured packet protocol solves all three.

## Packet Structure

```
┌──────────┬──────────┬───────────┬─────────────┬─────────┬──────────┐
│ Start(2) │ Type(1)  │ Length(1) │ Payload(N)  │ CRC(1)  │
└──────────┴──────────┴───────────┴─────────────┴─────────┴──────────┘
  0xFE 0xFE   cmd_id    N bytes    variable      CRC-8
```

- **Start bytes (0xFE 0xFE)**: Two-byte sync marker. The parser scans the stream for this sequence to find packet boundaries. Two bytes reduces false positives compared to a single byte.
- **Packet type (1 byte)**: Identifies the command or data type (0x01 = motor speeds, 0x02 = encoder data, etc.).
- **Payload length (1 byte)**: Number of bytes in the payload (0-255). Use 2 bytes if you need payloads > 255 bytes.
- **Payload (N bytes)**: The actual data. Use little-endian byte order (matches ARM and x86).
- **CRC-8**: Computed over type + length + payload bytes. Detects single-bit errors and most multi-bit errors.

## CRC-8 Implementation

On the MCU (C++):
```cpp
uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;  // polynomial 0x07
            else
                crc <<= 1;
        }
    }
    return crc;
}
```

For higher reliability, use CRC-16 (CCITT polynomial 0x1021). CRC-8 is sufficient for short packets under 32 bytes.

## Byte Stuffing (COBS Alternative)

If payload bytes can include 0xFE, the parser might falsely detect a start sequence mid-packet. Two solutions:

1. **COBS (Consistent Overhead Byte Stuffing)**: Eliminates a specific byte value (e.g., 0x00) from the payload with at most 1 byte overhead per 254 bytes. Use 0x00 as the frame delimiter instead of 0xFE.
2. **Byte stuffing**: Escape 0xFE in payload as [0xFD, 0x01] and escape 0xFD as [0xFD, 0x00]. Simpler but variable overhead.

For packets under 64 bytes (typical for robotics), byte stuffing adds minimal overhead and is simpler to implement.

## Protocol Definition Example

```python
# protocol.py - Shared constants between ROS 2 node and MCU firmware

SYNC_BYTES = bytes([0xFE, 0xFE])
HEADER_SIZE = 4   # sync(2) + type(1) + length(1)
MAX_PAYLOAD = 128

# Host → MCU commands
CMD_SET_MOTOR_SPEEDS  = 0x01  # payload: int16 left, int16 right (counts/sec)
CMD_SET_LED            = 0x02  # payload: uint8 r, uint8 g, uint8 b
CMD_SET_SERVO          = 0x03  # payload: uint8 channel, uint16 microseconds
CMD_REQUEST_STATUS     = 0x10  # payload: empty (poll request)

# MCU → Host data
DATA_ENCODER_COUNTS   = 0x81  # payload: int32 left, int32 right
DATA_IMU_RAW          = 0x82  # payload: int16 ax,ay,az, int16 gx,gy,gz
DATA_BATTERY          = 0x83  # payload: uint16 voltage_mV, int16 current_mA
DATA_STATUS           = 0x84  # payload: uint8 error_flags, uint16 uptime_sec
```

## Python Parser (ROS 2 Side)

```python
import serial
import struct
from crcmod.predefined import mkCrcFun

crc8_func = mkCrcFun('crc-8')

class PacketParser:
    def __init__(self, port: str, baud: int = 1000000):
        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.buf = bytearray()
        self.stats = {'received': 0, 'crc_errors': 0, 'sync_resets': 0}

    def read_packets(self) -> list:
        """Non-blocking read. Returns list of (packet_type, payload) tuples."""
        data = self.ser.read(512)
        if data:
            self.buf.extend(data)
        
        packets = []
        while len(self.buf) >= 5:  # minimum: sync(2) + type(1) + len(1) + crc(1)
            # Scan for sync bytes
            idx = self.buf.find(b'\xfe\xfe')
            if idx < 0:
                self.buf.clear()
                break
            if idx > 0:
                self.buf = self.buf[idx:]
                self.stats['sync_resets'] += 1

            if len(self.buf) < 4:
                break

            pkt_type = self.buf[2]
            pkt_len = self.buf[3]
            total = 4 + pkt_len + 1  # header + payload + crc

            if len(self.buf) < total:
                break  # wait for more data

            payload = bytes(self.buf[4:4 + pkt_len])
            received_crc = self.buf[4 + pkt_len]
            computed_crc = crc8_func(self.buf[2:4 + pkt_len])

            if received_crc == computed_crc:
                packets.append((pkt_type, payload))
                self.stats['received'] += 1
            else:
                self.stats['crc_errors'] += 1

            self.buf = self.buf[total:]

        return packets

    def send_packet(self, pkt_type: int, payload: bytes = b''):
        header = bytes([0xFE, 0xFE, pkt_type, len(payload)])
        crc = crc8_func(bytes([pkt_type, len(payload)]) + payload)
        self.ser.write(header + payload + bytes([crc]))
```

## MCU-Side Sender (C++)

```cpp
void sendPacket(uint8_t type, const uint8_t *payload, uint8_t len) {
    uint8_t header[4] = {0xFE, 0xFE, type, len};
    uint8_t crcBuf[2 + len];  // type + length + payload
    crcBuf[0] = type;
    crcBuf[1] = len;
    memcpy(&crcBuf[2], payload, len);
    uint8_t crc = crc8(crcBuf, 2 + len);

    Serial.write(header, 4);
    Serial.write(payload, len);
    Serial.write(crc);
    Serial.send_now();
}
```

## Bidirectional Communication Pattern

- **MCU → Host (sensor data)**: MCU sends at a fixed rate (50-100 Hz) driven by a timer interrupt. Each cycle sends encoder counts, IMU data, and battery voltage in separate packets.
- **Host → MCU (commands)**: ROS 2 node sends motor speed commands on every `/cmd_vel` callback. Commands are event-driven, not periodic.
- **Acknowledgment**: Not typically needed for streaming data. For configuration commands (PID gains, servo positions), implement ACK/NACK responses.

## Error Handling Strategy

1. CRC mismatch → discard packet, increment error counter
2. Unexpected packet type → discard, log warning
3. Payload length exceeds MAX_PAYLOAD → discard from sync bytes forward, rescan
4. No data for >500ms → flag communication timeout, trigger safety stop
5. Publish error statistics to `/hardware/serial_stats` for diagnostics
