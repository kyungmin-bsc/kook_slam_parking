<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: power-management
description: 'Design robot power distribution. Use when sizing batteries, planning voltage rails, monitoring power rails, or charging systems.'
---

# Robot Power Management and Distribution

## Power Architecture Overview

A mobile robot's power system distributes energy from the battery to multiple subsystems at different voltages:

```
Battery (e.g., 4S LiPo 14.8V nominal)
  │
  ├── Main Fuse (20-30A)
  │
  ├──→ Buck Converter → 12V rail (motor drivers, high-power actuators)
  │
  ├──→ Buck Converter → 5V rail (SBC: Raspberry Pi, Jetson, NUC)
  │
  ├──→ Buck Converter → 3.3V rail (MCU, low-power sensors)
  │
  └──→ Direct → Motor Controller input (accepts battery voltage directly)
```

## Voltage Rails and Regulation

### Buck Converters

Step-down (buck) converters are >90% efficient. Use these, not linear regulators (LM7805 wastes energy as heat).

| Rail | Typical Source | Consumers | Notes |
|------|---------------|-----------|-------|
| Battery (14.8V) | Direct | Motor controller, high-power relay | No regulation needed |
| 12V | LM2596 / MP1584 buck | Cooling fans, some actuators | 3A typical |
| 5V | D24V50F5 (Pololu) / MP1584 | SBC, servos, USB devices | 5A+ for SBC with peripherals |
| 3.3V | AMS1117 LDO from 5V | Teensy, I2C sensors | 1A sufficient, LDO is fine here |

### Critical: SBC Power Requirements

- **Raspberry Pi 5**: 5V, 5A recommended (27W USB-C PD). Under-voltage causes throttling and SD card corruption.
- **Jetson Nano**: 5V, 4A (through barrel jack or header pins).
- **Intel NUC**: 12-19V barrel jack, 3-5A depending on model.

Use a dedicated, high-quality buck converter for the SBC. Do NOT share its 5V rail with servos or motors—inrush current from servos causes voltage dips that reset the SBC.

## Power Monitoring with INA219/INA226

Place current/voltage monitors on each power rail for real-time monitoring.

### INA219 Wiring

```
Battery+ ─── [Shunt Resistor] ─── Load+
                  │         │
                 VIN+      VIN-    (INA219 sense inputs)
                  
Battery+ ──── VBUS ─── GND        (INA219 bus voltage input)
```

### Multi-Rail Monitoring Code

```cpp
#include <Adafruit_INA219.h>

Adafruit_INA219 ina_motor(0x40);    // A0=GND, A1=GND
Adafruit_INA219 ina_compute(0x41);  // A0=VCC, A1=GND
Adafruit_INA219 ina_sensor(0x44);   // A0=GND, A1=VCC

struct PowerRail {
    const char *name;
    Adafruit_INA219 *sensor;
    float voltage;
    float current_mA;
    float power_mW;
};

PowerRail rails[] = {
    {"motor",   &ina_motor,   0, 0, 0},
    {"compute", &ina_compute, 0, 0, 0},
    {"sensor",  &ina_sensor,  0, 0, 0},
};

void readAllPower() {
    for (auto &rail : rails) {
        rail.voltage = rail.sensor->getBusVoltage_V();
        rail.current_mA = rail.sensor->getCurrent_mA();
        rail.power_mW = rail.sensor->getPower_mW();
    }
}
```

## Power Budget Calculation

List every component and its current draw at each operating state:

| Component | Idle Current | Active Current | Peak Current | Voltage |
|-----------|-------------|----------------|-------------|---------|
| Raspberry Pi 5 | 0.6A | 1.5A | 5.0A | 5V |
| Teensy 4.1 | 0.1A | 0.15A | 0.2A | 3.3V (USB powered) |
| Motor driver (RoboClaw) | 0.05A | 2.0A | 15A | 14.8V |
| LIDAR (LD19) | 0.35A | 0.35A | 0.5A | 5V |
| IMU (ICM-20948) | 0.003A | 0.003A | 0.003A | 3.3V |
| Servo × 2 | 0.01A | 0.5A | 1.5A | 5V |
| **Total from battery** | **~1.5A** | **~5A** | **~20A** | **14.8V** |

Total average power: ~5A × 14.8V ≈ 74W

## Battery Sizing

$$
\text{Battery capacity (Wh)} = \text{Runtime (hours)} \times \text{Average power (W)}
$$

For 2 hours of runtime at 74W average: 148 Wh needed.

A 4S 10,000 mAh LiPo: $14.8\text{V} \times 10\text{Ah} = 148\text{Wh}$. Marginally sufficient — in practice, plan for 70% usable capacity (never discharge below 20% SoC for battery health):

$$
\text{Effective capacity} = 148 \times 0.70 = 103.6\text{Wh} → \text{Runtime} \approx 1.4\text{hours}
$$

## LiPo Safety

- **Minimum cell voltage**: 3.0V per cell (12.0V for 4S). Below this causes permanent damage, swelling, and potential thermal runaway.
- **Storage voltage**: 3.8V per cell for long-term storage (not fully charged, not depleted).
- **Charging**: Always use a balance charger. Charge at 1C maximum (10A for a 10Ah pack). Never charge unattended.
- **Physical protection**: LiPo bags or fire-resistant enclosure on the robot. Inspect for swelling or damage after impacts.
- **Wiring**: Use properly rated connectors (XT60 for up to 60A, XT90 for higher). Solder all connections—crimps and screw terminals add resistance and fire risk at high currents.

## Power Sequencing

Incorrect startup order can cause issues:

1. **MCU first**: Powers up, initializes I/O pins to safe states (motors OFF, servos neutral)
2. **Sensors next**: LIDAR, IMU, cameras initialize and begin publishing
3. **SBC last**: Boots, starts ROS 2, begins sending commands

If the SBC powers up before the MCU, the MCU's I/O pins may float, causing unpredictable motor behavior. Design the firmware so that **powers-up in safe state by default** — all outputs LOW/disabled until explicitly enabled by a command.

## Emergency Power Cutoff

A physical kill switch must be accessible without tools:

- **Location**: Rear of robot, top surface, marked with red
- **Type**: Latching push-button (push to cut, twist to release) or key switch
- **Circuit**: In series with battery positive line, before all electronics
- **Rating**: Must handle peak battery current (e.g., 30A rated for a 20A peak system)

For advanced setups, use a relay/MOSFET that the MCU can also trigger for software-initiated shutdown (e.g., critical battery level).

## Publishing Power Data as ROS 2 Diagnostics

```python
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class PowerDiagnostics(Node):
    def __init__(self):
        super().__init__('power_diagnostics')
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(1.0, self.publish_diagnostics)

    def publish_diagnostics(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for rail_name, voltage, current in self.get_power_data():
            status = DiagnosticStatus()
            status.name = f'Power: {rail_name}'
            status.hardware_id = f'ina219_{rail_name}'
            
            if voltage < self.min_voltages[rail_name]:
                status.level = DiagnosticStatus.ERROR
                status.message = f'{rail_name} voltage LOW: {voltage:.2f}V'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'Normal'
            
            status.values = [
                KeyValue(key='voltage_V', value=f'{voltage:.2f}'),
                KeyValue(key='current_mA', value=f'{current:.1f}'),
                KeyValue(key='power_W', value=f'{voltage * current / 1000:.1f}'),
            ]
            msg.status.append(status)
        
        self.diag_pub.publish(msg)
```

View diagnostics with:
```bash
ros2 run rqt_runtime_monitor rqt_runtime_monitor
# or
ros2 topic echo /diagnostics
```

## Thermal Considerations

- Buck converters generate heat proportional to current. Mount with heatsinks and airflow.
- Motor drivers at sustained high current need heatsinking. RoboClaw 2x30A can thermal-throttle without airflow.
- SBC (especially Jetson) needs active cooling under load. Monitor CPU temperature and throttle navigation if overheating.
- In enclosed robot bodies, consider a small exhaust fan controlled by the MCU based on internal temperature readings.
