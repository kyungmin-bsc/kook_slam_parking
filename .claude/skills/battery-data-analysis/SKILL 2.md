<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: battery-data-analysis
description: 'Analyze robot battery performance data. Use when plotting discharge curves, current histograms, or estimating state of health.'
---

# Battery Performance Analysis for Robots

## Collecting Battery Data

```bash
# Record battery state to a rosbag
ros2 bag record --storage mcap /battery_state /cmd_vel /odom

# BatteryState message fields (sensor_msgs/msg/BatteryState):
#   float32 voltage          — pack voltage (V)
#   float32 current          — current draw, negative = discharging (A)
#   float32 charge           — remaining charge (Ah)
#   float32 capacity         — full capacity (Ah)
#   float32 percentage       — 0.0 to 1.0
#   float32 temperature      — battery temp (°C)
#   uint8   power_supply_status  — CHARGING/DISCHARGING/NOT_CHARGING/FULL
#   float32[] cell_voltage   — per-cell voltages if available
```

## Key Metrics

| Metric | Formula | Significance |
|--------|---------|-------------|
| Discharge curve | voltage vs time | Shows remaining capacity nonlinearly |
| Current draw profile | current histogram | idle vs nav vs motor stall |
| Energy per mission | ∫ P dt = ∫ V·I dt | Total Wh consumed per patrol |
| Remaining runtime | (charge / avg_current) | Estimated time to cutoff |
| State of Health | measured_capacity / nominal_capacity | Battery degradation tracking |
| Voltage sag | V_load - V_rest | Internal resistance indicator |

## Voltage Thresholds by Chemistry

| Chemistry | Nominal V/cell | Min V/cell | Cutoff Action |
|-----------|---------------|------------|---------------|
| LiPo (3S) | 3.7V (11.1V pack) | 3.0V (9.0V) | Emergency dock |
| LiFePO4 (4S) | 3.2V (12.8V pack) | 2.5V (10.0V) | Emergency dock |
| Lead-acid (12V) | 2.1V/cell | 1.75V/cell (10.5V) | Emergency dock |

## Analysis Script

```python
#!/usr/bin/env python3
"""Battery performance analysis: discharge curve, current profile, power vs velocity."""

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def read_topic(bag_path: str, topic: str):
    """Yield (msg, timestamp_ns) for a single topic."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=""),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    msg_type = get_message(type_map[topic])
    while reader.has_next():
        _, data, ts = reader.read_next()
        yield deserialize_message(data, msg_type), ts


def load_battery_data(bag_path: str) -> pd.DataFrame:
    rows = []
    for msg, ts in read_topic(bag_path, "/battery_state"):
        rows.append({
            "time_ns": ts,
            "voltage": msg.voltage,
            "current": abs(msg.current),  # make positive for discharge
            "percentage": msg.percentage * 100,
            "temperature": msg.temperature if msg.temperature > -50 else np.nan,
            "charge": msg.charge,
            "capacity": msg.capacity,
        })
    df = pd.DataFrame(rows)
    df["time_s"] = (df["time_ns"] - df["time_ns"].iloc[0]) / 1e9
    df["time_min"] = df["time_s"] / 60
    df["power_w"] = df["voltage"] * df["current"]
    return df


def load_velocity_data(bag_path: str) -> pd.DataFrame:
    rows = []
    for msg, ts in read_topic(bag_path, "/cmd_vel"):
        rows.append({
            "time_ns": ts,
            "vx": msg.linear.x,
            "wz": msg.angular.z,
        })
    df = pd.DataFrame(rows)
    df["time_s"] = (df["time_ns"] - df["time_ns"].iloc[0]) / 1e9
    df["speed"] = np.sqrt(df["vx"]**2 + (df["wz"] * 0.2)**2)  # approximate
    return df


def analyze_battery(bag_path: str):
    bat = load_battery_data(bag_path)

    # Compute statistics
    total_energy_wh = np.trapz(bat["power_w"], bat["time_s"]) / 3600
    mean_current = bat["current"].mean()
    peak_current = bat["current"].max()
    p95_current = bat["current"].quantile(0.95)
    voltage_drop = bat["voltage"].iloc[0] - bat["voltage"].iloc[-1]
    duration_min = bat["time_min"].iloc[-1]

    print(f"Duration:        {duration_min:.1f} min")
    print(f"Energy consumed: {total_energy_wh:.2f} Wh")
    print(f"Mean current:    {mean_current:.2f} A")
    print(f"Peak current:    {peak_current:.2f} A (P95: {p95_current:.2f} A)")
    print(f"Voltage drop:    {voltage_drop:.2f} V ({bat['voltage'].iloc[0]:.1f}V → {bat['voltage'].iloc[-1]:.1f}V)")

    if bat["capacity"].iloc[0] > 0:
        soh = bat["capacity"].iloc[0] / bat["capacity"].max() * 100
        print(f"State of Health: {soh:.1f}%")

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 1. Discharge curve
    ax = axes[0, 0]
    ax.plot(bat["time_min"], bat["voltage"], "b-", linewidth=1)
    ax.axhline(y=10.0, color="r", linestyle="--", alpha=0.7, label="Min cutoff (LiFePO4)")
    ax.set_xlabel("Time (min)")
    ax.set_ylabel("Voltage (V)")
    ax.set_title("Discharge Curve")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 2. Current histogram
    ax = axes[0, 1]
    ax.hist(bat["current"], bins=80, edgecolor="black", alpha=0.7, color="steelblue")
    ax.axvline(mean_current, color="r", linestyle="--", label=f"Mean: {mean_current:.2f}A")
    ax.axvline(p95_current, color="orange", linestyle="--", label=f"P95: {p95_current:.2f}A")
    ax.set_xlabel("Current (A)")
    ax.set_ylabel("Count")
    ax.set_title("Current Draw Distribution")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 3. Power and voltage over time (voltage sag visible)
    ax = axes[1, 0]
    ax.plot(bat["time_min"], bat["power_w"], "g-", linewidth=0.5, alpha=0.7, label="Power (W)")
    ax2 = ax.twinx()
    ax2.plot(bat["time_min"], bat["voltage"], "b-", linewidth=0.8, alpha=0.5, label="Voltage (V)")
    ax.set_xlabel("Time (min)")
    ax.set_ylabel("Power (W)", color="g")
    ax2.set_ylabel("Voltage (V)", color="b")
    ax.set_title("Power Draw & Voltage Sag")
    ax.grid(True, alpha=0.3)

    # 4. Power vs velocity correlation
    try:
        vel = load_velocity_data(bag_path)
        # Resample velocity to battery timestamps
        bat_resampled = bat.set_index("time_s")
        vel_resampled = vel.set_index("time_s").reindex(
            bat_resampled.index, method="nearest", tolerance=1.0
        )
        mask = vel_resampled["speed"].notna()
        ax = axes[1, 1]
        ax.scatter(vel_resampled.loc[mask, "speed"], bat_resampled.loc[mask, "power_w"],
                   alpha=0.3, s=5)
        ax.set_xlabel("Robot Speed (m/s approx)")
        ax.set_ylabel("Power (W)")
        ax.set_title("Power vs Velocity Correlation")
        ax.grid(True, alpha=0.3)
    except Exception:
        axes[1, 1].text(0.5, 0.5, "No /cmd_vel data", transform=axes[1, 1].transAxes,
                        ha="center", va="center", fontsize=14)

    plt.suptitle(f"Battery Analysis — {total_energy_wh:.1f} Wh over {duration_min:.0f} min", fontsize=14)
    plt.tight_layout()
    plt.savefig("battery_analysis.png", dpi=150)
    plt.show()


if __name__ == "__main__":
    import sys
    bag_path = sys.argv[1] if len(sys.argv) > 1 else "rosbag2_recording"
    analyze_battery(bag_path)
```

## Voltage Sag Analysis

Voltage sag reveals internal resistance. Under load, voltage drops; at rest, it recovers:

```python
# Detect sag events: voltage drops > 0.3V within 2 seconds
voltage_diff = bat["voltage"].diff()
sag_events = bat[voltage_diff < -0.3]
for _, event in sag_events.iterrows():
    print(f"Sag at t={event['time_min']:.1f}min: ΔV={voltage_diff.loc[event.name]:.2f}V, "
          f"I={event['current']:.2f}A → R_int≈{abs(voltage_diff.loc[event.name])/event['current']:.3f}Ω")
```

## Automated Alerts

```python
# Runtime alert thresholds
ALERTS = {
    "low_voltage": {"threshold": 10.5, "field": "voltage", "op": "lt"},
    "high_current": {"threshold": 15.0, "field": "current", "op": "gt"},
    "high_temp": {"threshold": 45.0, "field": "temperature", "op": "gt"},
    "low_soh": {"threshold": 70.0, "field": "capacity", "op": "lt_pct"},  # % of nominal
}
```

## Tracking Degradation Over Charge Cycles

Log each full discharge cycle's total energy and capacity. Plot capacity vs cycle number to predict replacement schedule. A LiFePO4 pack typically retains 80% capacity after 2000+ cycles; LiPo after ~300-500 cycles.
