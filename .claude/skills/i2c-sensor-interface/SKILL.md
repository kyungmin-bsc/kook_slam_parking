<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: i2c-sensor-interface
description: 'Interface I2C sensors with microcontrollers. Use when reading VL53L0X, INA219, BMP280 via Wire library or I2C multiplexers.'
---

# I2C Sensor Integration Patterns

## I2C Fundamentals for Robotics

I2C (Inter-Integrated Circuit) is a two-wire serial bus: SDA (data) and SCL (clock). A single bus can host multiple devices, each with a unique 7-bit address (0x00-0x7F). The master (MCU) initiates all transactions.

### Teensy 4.1 I2C Buses

| Bus | SDA Pin | SCL Pin | Instance |
|-----|---------|---------|----------|
| Wire | 18 | 19 | I2C0 |
| Wire1 | 17 | 16 | I2C1 |
| Wire2 | 25 | 24 | I2C2 |

Three independent buses — use separate buses for high-throughput sensors (IMU at 400kHz on one bus, slow sensors on another).

### Basic Wire Operations

```cpp
#include <Wire.h>

void setup() {
    Wire.begin();           // join as master
    Wire.setClock(400000);  // 400 kHz fast mode
}

uint8_t readRegister(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    uint8_t error = Wire.endTransmission(false);  // repeated start
    if (error != 0) return 0xFF;  // NACK or timeout
    
    Wire.requestFrom(addr, (uint8_t)1);
    if (Wire.available()) return Wire.read();
    return 0xFF;
}

void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void readRegisters(uint8_t addr, uint8_t startReg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(startReg);
    Wire.endTransmission(false);
    
    Wire.requestFrom(addr, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        buf[i] = Wire.read();
    }
}
```

## I2C Scanner

Always verify device addresses before writing driver code:

```cpp
void scanI2C() {
    Serial.println("Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            Serial.printf("  Device found at 0x%02X\n", addr);
        }
    }
}
```

## Common Robotics I2C Sensors

### VL53L0X / VL53L1X (Time-of-Flight Range Sensor)

Address: 0x29 (default, configurable via software). Range: 30-2000mm (VL53L0X), 40-4000mm (VL53L1X).

```cpp
#include <VL53L0X.h>  // Pololu library

VL53L0X tof;

void setup() {
    Wire.begin();
    Wire.setClock(400000);
    
    tof.setTimeout(500);
    if (!tof.init()) {
        Serial.println("VL53L0X init failed!");
        while (1);
    }
    tof.setMeasurementTimingBudget(33000);  // 33ms for higher accuracy
    tof.startContinuous();
}

void loop() {
    uint16_t range_mm = tof.readRangeContinuousMillimeters();
    if (tof.timeoutOccurred()) {
        Serial.println("VL53L0X timeout");
    } else {
        Serial.printf("Range: %d mm\n", range_mm);
    }
}
```

Multiple VL53L0X on the same bus: each sensor powers up with address 0x29. Use the XSHUT pin to hold all but one in reset, then reprogram addresses one at a time:

```cpp
const uint8_t XSHUT_PINS[] = {6, 7, 8};  // one per sensor
VL53L0X sensors[3];
const uint8_t ADDRESSES[] = {0x30, 0x31, 0x32};

void setup() {
    // Hold all in reset
    for (auto pin : XSHUT_PINS) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    delay(10);
    
    // Enable and configure one at a time
    for (int i = 0; i < 3; i++) {
        digitalWrite(XSHUT_PINS[i], HIGH);
        delay(10);
        sensors[i].setAddress(ADDRESSES[i]);
        sensors[i].init();
        sensors[i].startContinuous();
    }
}
```

### INA219 / INA226 (Current/Voltage Monitor)

Address: 0x40-0x4F (configurable via A0/A1 pins). Measures bus voltage and current via shunt resistor.

```cpp
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219(0x40);

void setup() {
    Wire.begin();
    ina219.begin();
    ina219.setCalibration_16V_400mA();  // or _32V_2A, _32V_1A
}

void loop() {
    float busVoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float power_mW = ina219.getPower_mW();
    Serial.printf("V=%.2f I=%.1fmA P=%.1fmW\n", busVoltage, current_mA, power_mW);
}
```

### BMP280 / BME280 (Barometric Pressure / Humidity)

Address: 0x76 or 0x77 (SDO pin selects). Useful for altitude estimation (relative, not absolute).

### MPU6050 / ICM-20948 (IMU)

Address: 0x68 or 0x69 (AD0 pin). 6-axis (MPU6050) or 9-axis (ICM-20948). Read 14 bytes in burst (accel XYZ + temp + gyro XYZ):

```cpp
void readIMU(int16_t *accel, int16_t *gyro) {
    uint8_t buf[14];
    readRegisters(0x68, 0x3B, buf, 14);
    
    accel[0] = (buf[0] << 8) | buf[1];   // X
    accel[1] = (buf[2] << 8) | buf[3];   // Y
    accel[2] = (buf[4] << 8) | buf[5];   // Z
    // buf[6-7] = temperature
    gyro[0] = (buf[8] << 8) | buf[9];    // X
    gyro[1] = (buf[10] << 8) | buf[11];  // Y
    gyro[2] = (buf[12] << 8) | buf[13];  // Z
}
```

## Pull-Up Resistors

I2C requires pull-up resistors on SDA and SCL. Standard values:

| Clock Speed | Typical Pull-Up |
|------------|----------------|
| 100 kHz (standard) | 4.7 kΩ |
| 400 kHz (fast) | 2.2 kΩ |
| 1 MHz (fast+) | 1 kΩ |

Teensy 4.1 has internal pull-ups (~22 kΩ)—too weak for reliable 400 kHz operation. **Always add external pull-ups** for production robots. One pair per bus, not per device.

## I2C Multiplexer (TCA9548A)

When you need multiple sensors with the same address (e.g., eight VL53L0X at 0x29):

```cpp
const uint8_t TCA_ADDR = 0x70;

void tcaSelect(uint8_t channel) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);  // enable channel 0-7
    Wire.endTransmission();
}

void readAllTOF() {
    for (uint8_t ch = 0; ch < 8; ch++) {
        tcaSelect(ch);
        uint16_t range = tof[ch].readRangeContinuousMillimeters();
        // process range
    }
}
```

The TCA9548A supports up to 8 channels. Multiple TCA9548As can coexist (addresses 0x70-0x77 via A0-A2 pins), giving up to 64 isolated bus segments.

## Error Handling

I2C is susceptible to bus hangs (SDA stuck low). Defensive strategies:

```cpp
uint8_t safeReadRegister(uint8_t addr, uint8_t reg, uint8_t *value, uint8_t retries = 3) {
    for (uint8_t i = 0; i < retries; i++) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        uint8_t err = Wire.endTransmission(false);
        if (err != 0) {
            Wire.end();            // reset bus
            delayMicroseconds(100);
            Wire.begin();
            Wire.setClock(400000);
            continue;
        }
        
        uint8_t count = Wire.requestFrom(addr, (uint8_t)1);
        if (count == 1) {
            *value = Wire.read();
            return 0;  // success
        }
    }
    return 1;  // all retries failed
}
```

## Publishing to ROS 2

Aggregate I2C sensor data into serial packets sent to the host. Example packet layout for sensor board:

```
Packet type 0x82 (sensor data):
  [0-1]  VL53L0X front range (uint16, mm)
  [2-3]  VL53L0X left range  (uint16, mm)
  [4-5]  VL53L0X right range (uint16, mm)
  [6-7]  INA219 voltage (uint16, mV)
  [8-9]  INA219 current (int16, mA)
  [10-11] BMP280 pressure (uint16, hPa × 10)
```

The ROS 2 node unpacks this and publishes `sensor_msgs/Range` for each ToF sensor and `sensor_msgs/BatteryState` for power data. One serial packet per sensor cycle at 20-50 Hz.
