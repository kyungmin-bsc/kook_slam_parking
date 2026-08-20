<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: teensy-platformio
description: 'Set up Teensy 4.1 with PlatformIO. Use when configuring platformio.ini, IntervalTimer, DMA ADC, or TeensyThreads.'
---

# Teensy 4.1 with PlatformIO for ROS 2 Hardware Integration

## Hardware Specifications

The Teensy 4.1 is built around the NXP i.MX RT1062 ARM Cortex-M7 running at 600 MHz. Key specs relevant to robotics:

- **Flash**: 8 MB (7,936 KB usable), with optional QSPI flash expansion
- **RAM**: 1 MB (512 KB tightly-coupled for zero-wait-state access, 512 KB general purpose)
- **Digital I/O**: 42 pins, 3.3V logic (5V tolerant on most pins)
- **Analog inputs**: 18 pins, 12-bit ADC (configurable to 10/12/16-bit)
- **PWM**: 31 pins across FlexPWM and QuadTimer modules
- **Hardware peripherals**: 3x SPI, 3x I2C, 8x serial (UART), 1x CAN bus (CAN 2.0B via FlexCAN), 1x Ethernet, USB host + device
- **Timers**: 4x general purpose (GPT), 4x periodic interrupt (PIT), plus FlexPWM timers

## PlatformIO Project Setup

### platformio.ini

```ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino
upload_protocol = teensy-gui       ; or teensy-cli for headless
monitor_speed = 1000000

; CPU speed options: 600, 528, 450, 396, 150, 24 MHz
board_build.f_cpu = 600000000L

lib_deps =
    PaulStoffregen/Encoder@^1.4.2
    Wire                            ; built-in, but explicit for clarity
    SPI                             ; built-in
    ftrias/TeensyThreads@^1.1

build_flags =
    -D USB_SERIAL
    -O2
```

### Project Structure

```
project/
├── platformio.ini
├── src/
│   └── main.cpp          ; entry point
├── lib/
│   ├── SerialProtocol/   ; custom serial framing
│   └── MotorControl/     ; PID loops
├── include/
│   └── config.h          ; pin assignments, constants
└── test/
```

## Upload Methods

- **teensy-gui**: Launches the Teensy Loader GUI. Press the physical button on the Teensy to enter bootloader. Works on macOS/Linux/Windows.
- **teensy-cli** (`teensy_loader_cli`): Headless upload. Add to `platformio.ini`: `upload_protocol = teensy-cli`. Requires the `-mmcu=TEENSY41` flag. Some OS require udev rules (Linux) or device permissions.
- **Auto-upload**: Teensy Loader can be set to auto-upload when a new hex file appears. In PlatformIO, `pio run -t upload` triggers this flow.

## Serial Communication at High Baud Rates

Teensy 4.1 USB serial runs at full USB speed regardless of the baud rate setting, so `Serial.begin(1000000)` is fine—USB ignores the baud parameter. For hardware UART (Serial1-Serial8), the baud rate matters:

```cpp
Serial1.begin(1000000);  // UART TX=pin1, RX=pin0
Serial2.begin(115200);   // UART TX=pin8, RX=pin7
```

Hardware UARTs support baud rates up to several Mbps. Use hardware UART for MCU-to-MCU communication or external devices. Use USB Serial for host communication—it is faster and more reliable than UART at high speeds.

## Timer Interrupts with IntervalTimer

`IntervalTimer` uses the ARM PIT (Periodic Interrupt Timer) for microsecond-precision callbacks:

```cpp
#include <Arduino.h>
#include <Encoder.h>

Encoder enc_left(2, 3);
Encoder enc_right(4, 5);
volatile long enc_left_count = 0;
volatile long enc_right_count = 0;

IntervalTimer encoderTimer;
IntervalTimer pidTimer;

void readEncoders() {
    enc_left_count = enc_left.read();
    enc_right_count = enc_right.read();
}

void runPID() {
    // PID computation here—keep it SHORT
    // Read volatile encoder counts, compute output, write to motor PWM
}

void setup() {
    Serial.begin(1000000);
    encoderTimer.begin(readEncoders, 1000);   // 1 kHz
    pidTimer.begin(runPID, 10000);            // 100 Hz
}
```

Up to 4 IntervalTimers can run simultaneously. Each is backed by a separate PIT channel.

## DMA for ADC

For continuous analog sampling without blocking the CPU, use the ADC library with DMA:

```cpp
#include <ADC.h>
ADC *adc = new ADC();

void setup() {
    adc->adc0->setResolution(12);
    adc->adc0->setAveraging(4);
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
}
```

DMA-based ADC reads free the CPU entirely during conversion. This is critical when sampling multiple analog channels at high rates (e.g., battery voltage, motor current, potentiometer feedback).

## TeensyThreads for Multi-tasking

```cpp
#include <TeensyThreads.h>

volatile bool newSerialData = false;

void serialThread() {
    while (1) {
        if (Serial.available()) {
            // Parse incoming packets
            newSerialData = true;
        }
        threads.yield();
    }
}

void setup() {
    Serial.begin(1000000);
    threads.addThread(serialThread, 0, 4096);  // stack size 4096 bytes
}

void loop() {
    // Main thread: handle motor control, sensor reads
}
```

TeensyThreads provides cooperative/preemptive threading. Default stack per thread is 1024 bytes—increase for complex tasks. Use `Threads::Mutex` for shared data access.

## Common Architecture Pattern

```
Main loop (loop())         → Serial protocol: parse commands, send sensor data
IntervalTimer #1 (1 kHz)  → Read encoders (fast, non-blocking)
IntervalTimer #2 (100 Hz) → PID control loop (compute + write motor outputs)
IntervalTimer #3 (50 Hz)  → Read analog sensors (battery, current)
Thread #1                  → Serial RX parsing (if using threaded approach)
```

## Critical Warnings

- **Never use blocking calls in ISRs**: No `Serial.print()`, `delay()`, `Wire` transactions, or `malloc()` inside `IntervalTimer` callbacks. ISRs must complete in microseconds.
- **Volatile keyword**: Any variable shared between ISR and main code must be declared `volatile`. For multi-byte variables (32/64-bit), disable interrupts during read: `noInterrupts(); long val = shared_var; interrupts();`
- **Stack overflow in threads**: TeensyThreads does not detect stack overflow. Allocate generous stacks and keep thread functions lean.
- **USB Serial latency**: `Serial.print()` buffers data. Call `Serial.send_now()` if you need low-latency responses. Consider writing binary packets directly with `Serial.write(buf, len)`.
- **Pin conflicts**: Check the Teensy 4.1 pinout card. Some pins share timer channels or peripheral functions. Using a pin for PWM may conflict with its use as an encoder input.
