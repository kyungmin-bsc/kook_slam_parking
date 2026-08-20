<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: rosbag2-analysis
description: 'Analyze rosbag2 data offline. Use when reading bag files in Python, plotting trajectories, timing analysis, or mcap format.'
---

# Analyzing rosbag2 Data Offline

## Recording Bags

```bash
# Record all topics
ros2 bag record -a

# Record selected topics (preferred for disk space)
ros2 bag record /scan /odom /cmd_vel /battery_state /tf /tf_static

# Use mcap storage (better compression, random access, Foxglove-compatible)
ros2 bag record --storage mcap /scan /odom /cmd_vel

# Default sqlite3 storage (works everywhere)
ros2 bag record --storage sqlite3 /odom /cmd_vel

# Set max bag duration/size
ros2 bag record -a --max-bag-duration 300 --max-bag-size 1073741824
```

## Inspecting Bags

```bash
# Summary info: topics, message counts, duration
ros2 bag info bag_directory/

# Example output:
# Files:             bag_directory/bag_directory_0.mcap
# Bag size:          142.6 MiB
# Duration:          312.4s
# Messages:          48732
# Topic information:
#   /odom          12500 msgs : nav_msgs/msg/Odometry
#   /scan           6240 msgs : sensor_msgs/msg/LaserScan
#   /cmd_vel        6250 msgs : geometry_msgs/msg/Twist
#   /battery_state   312 msgs : sensor_msgs/msg/BatteryState
```

## Playback

```bash
# Basic playback (publishes /clock for use_sim_time nodes)
ros2 bag play bag_directory/ --clock

# Half-speed replay for debugging
ros2 bag play bag_directory/ --clock --rate 0.5

# Double-speed for fast review
ros2 bag play bag_directory/ --clock --rate 2.0

# Filter to specific topics on playback
ros2 bag play bag_directory/ --topics /scan /odom

# Loop continuously
ros2 bag play bag_directory/ --clock --loop

# Start from offset (seconds)
ros2 bag play bag_directory/ --clock --start-offset 60.0
```

## Python Analysis with rosbag2_py

```python
#!/usr/bin/env python3
"""Read /odom from a rosbag and plot the robot trajectory."""

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def read_messages(bag_path: str, topics: list[str]):
    """Yield (topic, msg, timestamp_ns) from a rosbag."""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    # Build type map
    type_map = {}
    for topic_info in reader.get_all_topics_and_types():
        type_map[topic_info.name] = topic_info.type

    # Filter to requested topics
    storage_filter = rosbag2_py.StorageFilter(topics=topics)
    reader.set_filter(storage_filter)

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp


def odom_to_dataframe(bag_path: str) -> pd.DataFrame:
    """Extract /odom into a pandas DataFrame."""
    rows = []
    for topic, msg, ts in read_messages(bag_path, ["/odom"]):
        rows.append({
            "time_ns": ts,
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "vx": msg.twist.twist.linear.x,
            "wz": msg.twist.twist.angular.z,
        })
    df = pd.DataFrame(rows)
    df["time_s"] = (df["time_ns"] - df["time_ns"].iloc[0]) / 1e9
    return df


def plot_trajectory(df: pd.DataFrame):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    # Trajectory (x vs y)
    axes[0].plot(df["x"], df["y"], linewidth=0.8)
    axes[0].plot(df["x"].iloc[0], df["y"].iloc[0], "go", markersize=10, label="Start")
    axes[0].plot(df["x"].iloc[-1], df["y"].iloc[-1], "rs", markersize=10, label="End")
    axes[0].set_xlabel("X (m)")
    axes[0].set_ylabel("Y (m)")
    axes[0].set_title("Robot Trajectory")
    axes[0].set_aspect("equal")
    axes[0].legend()
    axes[0].grid(True)

    # Velocity profiles
    axes[1].plot(df["time_s"], df["vx"], label="Linear (m/s)", linewidth=0.8)
    axes[1].plot(df["time_s"], df["wz"], label="Angular (rad/s)", linewidth=0.8)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_title("Velocity Profile")
    axes[1].legend()
    axes[1].grid(True)

    # Timing jitter (message interval vs expected 50Hz)
    dt = np.diff(df["time_s"])
    expected_dt = 0.02  # 50 Hz
    axes[2].hist(dt * 1000, bins=100, edgecolor="black", alpha=0.7)
    axes[2].axvline(expected_dt * 1000, color="r", linestyle="--", label=f"Expected {expected_dt*1000:.0f}ms")
    axes[2].set_xlabel("Message Interval (ms)")
    axes[2].set_title("Odom Timing Jitter")
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig("rosbag_analysis.png", dpi=150)
    plt.show()


if __name__ == "__main__":
    import sys
    bag_path = sys.argv[1] if len(sys.argv) > 1 else "rosbag2_recording"
    df = odom_to_dataframe(bag_path)
    print(f"Loaded {len(df)} odom messages over {df['time_s'].iloc[-1]:.1f}s")
    print(f"Distance traveled: {np.sqrt(np.diff(df['x'])**2 + np.diff(df['y'])**2).sum():.2f}m")
    plot_trajectory(df)
```

## Common Analyses

| Analysis | What to Plot | Topics Needed |
|----------|-------------|---------------|
| Trajectory | odom x vs y | `/odom` |
| Velocity profile | vx, wz over time | `/cmd_vel` or `/odom` |
| Sensor timing jitter | histogram of message intervals | any high-rate topic |
| TF latency | time between tf broadcasts | `/tf` |
| Scan coverage | polar plot of ranges | `/scan` |
| Battery discharge | voltage over time | `/battery_state` |

## mcap vs sqlite3

| Feature | sqlite3 | mcap |
|---------|---------|------|
| Default in Jazzy | Yes | No (use `--storage mcap`) |
| Compression | None | LZ4/Zstd built-in |
| Random access | Slow (sequential) | Fast (indexed) |
| Foxglove Studio | Needs conversion | Native support |
| File size | Larger | 30-50% smaller |

## Foxglove Studio

Open mcap bags directly in [Foxglove Studio](https://foxglove.dev/) for interactive visualization: timeline scrubbing, 3D scene rendering, synchronized multi-panel plots, and image+pointcloud overlay. Export to mcap format for best compatibility:

```bash
# Convert sqlite3 bag to mcap
ros2 bag convert -i input_bag/ -o output_bag/ --output-storage mcap
```
