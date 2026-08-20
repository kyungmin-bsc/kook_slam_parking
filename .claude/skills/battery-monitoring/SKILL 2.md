<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: battery-monitoring
description: 'Monitor robot battery state. Use when reading voltage/current via ADC, publishing BatteryState, or estimating state of charge.'
---

# Battery Voltage and Current Monitoring for ROS 2 Robots

## Voltage Measurement via ADC

A LiPo battery pack's maximum voltage exceeds MCU ADC input range. A voltage divider scales it down.

### Voltage Divider Design

For a 4S LiPo (16.8V max, 12.0V empty) with a 3.3V ADC:

$$
V_{\text{out}} = V_{\text{bat}} \times \frac{R_2}{R_1 + R_2}
$$

Target: $V_{\text{out}} \leq 3.0\text{V}$ at $V_{\text{bat}} = 18.0\text{V}$ (margin above 16.8V):

$$
\frac{R_2}{R_1 + R_2} = \frac{3.0}{18.0} = 0.167
$$

Choose standard resistor values: $R_1 = 100\text{k}\Omega$, $R_2 = 20\text{k}\Omega$. Actual ratio: $20/(100+20) = 0.167$. This gives $V_{\text{out}} = 2.80\text{V}$ at 16.8V — safely within ADC range.

Use high-value resistors (100kΩ+) to minimize current drain from the battery through the divider. Add a 100nF capacitor across R2 for noise filtering.

### ADC Reading to Voltage

Teensy 4.1: 12-bit ADC, reference voltage = 3.3V:

```cpp
const float DIVIDER_RATIO = 20.0 / (100.0 + 20.0);  // R2 / (R1 + R2)
const float ADC_MAX = 4095.0;
const float ADC_VREF = 3.3;

float readBatteryVoltage(int pin) {
    int raw = analogRead(pin);
    float adc_voltage = (raw / ADC_MAX) * ADC_VREF;
    float battery_voltage = adc_voltage / DIVIDER_RATIO;
    return battery_voltage;
}
```

### Noise Filtering

Battery voltage is noisy: motor current pulses cause voltage sag, and PWM switching creates EMI. Use multiple filtering strategies:

**Exponential Moving Average (EMA):**
```cpp
float filtered_voltage = 0.0;
const float ALPHA = 0.05;  // lower = smoother, slower response

void updateVoltage() {
    float raw = readBatteryVoltage(A0);
    filtered_voltage = ALPHA * raw + (1.0 - ALPHA) * filtered_voltage;
}
```

**Median Filter** (better for spike rejection):
```cpp
float readings[5];
int idx = 0;

float medianFilter(float new_val) {
    readings[idx] = new_val;
    idx = (idx + 1) % 5;
    float sorted[5];
    memcpy(sorted, readings, sizeof(sorted));
    // Simple insertion sort for 5 elements
    for (int i = 1; i < 5; i++) {
        float key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) { sorted[j+1] = sorted[j]; j--; }
        sorted[j+1] = key;
    }
    return sorted[2];  // median
}
```

Best approach: median filter first (removes spikes), then EMA (smooths remaining noise).

## Current Measurement

Use an INA219 or INA226 I2C current/voltage monitor with a shunt resistor in the power path.

INA219: 12-bit, measures both bus voltage and shunt voltage. Shunt resistor value determines current range:
- 0.1Ω shunt → max 3.2A (higher precision)
- 0.01Ω shunt → max 32A (lower precision, less power loss)

```cpp
#include <Wire.h>
const uint8_t INA219_ADDR = 0x40;

float readCurrent_mA() {
    Wire.beginTransmission(INA219_ADDR);
    Wire.write(0x04);  // Current register
    Wire.endTransmission();
    Wire.requestFrom(INA219_ADDR, (uint8_t)2);
    int16_t raw = (Wire.read() << 8) | Wire.read();
    return raw * 0.1;  // depends on calibration register setting
}
```

## State of Charge Estimation

### Voltage-Based Lookup Table

LiPo cell voltage vs. SoC (at rest, no load):

| Cell Voltage | SoC |
|-------------|-----|
| 4.20V | 100% |
| 4.10V | 90% |
| 3.97V | 80% |
| 3.92V | 70% |
| 3.87V | 60% |
| 3.82V | 50% |
| 3.79V | 40% |
| 3.77V | 30% |
| 3.74V | 20% |
| 3.68V | 10% |
| 3.50V | 5% |
| 3.00V | 0% (CUTOFF) |

For a 4S pack, multiply cell voltage by 4. Linear interpolation between table entries is sufficient.

### Voltage Sag Under Load

When motors draw current, battery voltage drops due to internal resistance. A 4S pack at 50% SoC might read 15.28V at rest but 14.2V under 10A load. **Do not use loaded voltage for SoC estimation directly.** Strategies:

1. **No-load sampling**: Only update SoC when motor current is below a threshold (e.g., <0.5A)
2. **IR compensation**: Measure current, estimate internal resistance, compensate: $V_{\text{oc}} = V_{\text{measured}} + I \times R_{\text{internal}}$
3. **Coulomb counting**: Integrate current over time. Requires known initial SoC and good current measurement.

## Publishing sensor_msgs/BatteryState

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.pub = self.create_publisher(BatteryState, 'battery/state', 10)
        self.timer = self.create_timer(1.0, self.publish_battery)

    def publish_battery(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = 15.2                  # from serial protocol
        msg.current = -2.5                  # negative = discharging (convention)
        msg.charge = float('nan')           # unknown unless coulomb counting
        msg.capacity = float('nan')
        msg.design_capacity = 5.0           # 5000mAh = 5.0 Ah
        msg.percentage = 0.65               # 0.0 to 1.0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True
        # Per-cell voltages (if available)
        msg.cell_voltage = [3.80, 3.80, 3.81, 3.79]
        msg.cell_temperature = []
        self.pub.publish(msg)
```

## Alert Thresholds and Integration

Define thresholds for behavior tree integration:

```yaml
# battery_params.yaml
battery_monitor:
  ros__parameters:
    warn_percentage: 0.20        # 20% - start heading to charger
    critical_percentage: 0.10    # 10% - abort all tasks, dock immediately
    cutoff_voltage: 12.0         # 3.0V/cell absolute minimum
    cells: 4
    chemistry: "lipo"
```

Behavior tree condition node `IsBatteryLow` subscribes to `/battery/state` and checks `percentage < warn_percentage`. This triggers a transition from patrol mode to dock-seeking mode. At critical level, override all behaviors with emergency docking.

## Hardware Safety

- **Always use a fuse** between battery and electronics (rated slightly above peak expected current)
- **Low-voltage alarm buzzer**: Hardware buzzer that screams when any cell drops below 3.3V. Independent of software.
- **Never discharge LiPo below 3.0V/cell**: Causes permanent cell damage, swelling, and fire risk
- **Temperature monitoring**: LiPo should not exceed 60°C during discharge. Add a thermistor on the pack and publish temperature in `BatteryState.cell_temperature`.
