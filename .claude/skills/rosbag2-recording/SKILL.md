<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: rosbag2-recording
description: 'Record and play back ROS 2 bag files. Use when configuring rosbag2 recording, storage backends, or playback options.'
---

# Role
You are an expert in ROS 2 rosbag2 recording and playback. You guide correct bag recording, playback, storage format selection, and programmatic bag access in ROS 2 Jazzy/Rolling.

## Recording
```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /scan /odom /cmd_vel -o my_recording

# Record with topic regex
ros2 bag record --regex "/camera/.*" -o camera_data

# Exclude topics
ros2 bag record -a --exclude "/camera/image_raw|/pointcloud"

# Record with compression (zstd)
ros2 bag record -a --compression-mode file --compression-format zstd

# Record with MCAP storage (recommended for Jazzy+)
ros2 bag record -a -s mcap -o my_recording

# Record with max bag size (split into multiple files)
ros2 bag record -a --max-bag-size 1073741824  # 1 GB

# Record with max duration
ros2 bag record -a --max-bag-duration 300  # 5 minutes
```

## Playback
```bash
# Basic playback
ros2 bag play my_recording

# Publish clock for sim_time nodes
ros2 bag play my_recording --clock

# Playback at different speeds
ros2 bag play my_recording --rate 2.0    # 2x speed
ros2 bag play my_recording --rate 0.5    # half speed

# Loop playback
ros2 bag play my_recording --loop

# Play specific topics only
ros2 bag play my_recording --topics /scan /odom

# Remap topics during playback
ros2 bag play my_recording --remap /cmd_vel:=/robot/cmd_vel

# Start from a specific time offset
ros2 bag play my_recording --start-offset 30.0  # skip first 30s

# Play until a specific duration
ros2 bag play my_recording --playback-duration 60.0  # play 60 seconds

# Pause/resume (press space in terminal)
ros2 bag play my_recording --start-paused
```

## Bag Information
```bash
ros2 bag info my_recording

# Output includes:
# - Duration
# - Start/end time
# - Message count per topic
# - Topic types
# - Storage format (sqlite3/mcap)
# - File sizes
```

## Storage Backends

### SQLite3 (Default in Humble)
- Single `.db3` file + `metadata.yaml`
- Mature, widely supported
- Slower for large bags, poor seek performance

### MCAP (Preferred for Jazzy/Rolling)
- Single `.mcap` file
- Much better read/write performance
- Excellent seek performance (indexed)
- Better compression support
- Compatible with Foxglove Studio
- Smaller file sizes

```bash
# Set MCAP as default
ros2 bag record -s mcap /scan /odom -o my_recording

# Convert old SQLite3 bags to MCAP
ros2 bag convert -i old_recording -o new_recording -s mcap
```

## Python API for Reading Bags
```python
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from rosidl_runtime_py.utilities import get_message

def read_bag(bag_path):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    # Get topic metadata
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if topic == '/scan':
            print(f'[{timestamp}] Scan ranges: {len(msg.ranges)}')
        elif topic == '/odom':
            print(f'[{timestamp}] Odom x={msg.pose.pose.position.x:.2f}')
```

## Python API for Writing Bags
```python
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message
from std_msgs.msg import String
import time

writer = SequentialWriter()
storage_options = StorageOptions(uri='output_bag', storage_id='mcap')
converter_options = ConverterOptions(
    input_serialization_format='cdr',
    output_serialization_format='cdr')
writer.open(storage_options, converter_options)

topic = TopicMetadata(
    name='/my_topic',
    type='std_msgs/msg/String',
    serialization_format='cdr')
writer.create_topic(topic)

msg = String()
msg.data = 'hello'
writer.write('/my_topic', serialize_message(msg), int(time.time() * 1e9))
```

## QoS Override During Playback
```yaml
# qos_overrides.yaml
/scan:
  reliability: best_effort
  durability: volatile
/map:
  reliability: reliable
  durability: transient_local
```
```bash
ros2 bag play my_recording --qos-profile-overrides-path qos_overrides.yaml
```

## Filtering by Time
```bash
# Record with time trigger (start after delay)
ros2 bag record -a --start-paused
# Then resume via service: ros2 service call /rosbag2_recorder/resume ...

# Programmatic time filtering during read
while reader.has_next():
    topic, data, timestamp = reader.read_next()
    time_sec = timestamp / 1e9
    if time_sec < start_time or time_sec > end_time:
        continue
    # process message
```

## Critical Warnings
- **Disk space with cameras**: An uncompressed 1080p camera at 30fps generates ~1.5 GB/min. Always use compressed image topics (`/image/compressed`) or enable bag compression. PointCloud2 at 10Hz can exceed 500 MB/min.
- **Clock for sim_time**: When playing back bags for nodes with `use_sim_time=true`, you MUST use `--clock`. Without it, nodes using sim_time will be stuck at time 0.
- **Topic QoS mismatch on playback**: rosbag2 replays with the recorded QoS. If subscribers expect different QoS, use `--qos-profile-overrides-path`.
- **MCAP for new projects**: Always use MCAP (`-s mcap`) for new recordings. SQLite3 is legacy and has poor performance for large bags.
- **Bag corruption**: SQLite3 bags can corrupt on unclean shutdown. MCAP is more resilient. Always stop recording cleanly (Ctrl+C).
- **Namespace changes**: If topics were recorded under a namespace (e.g., `/robot/scan`) but your nodes expect `/scan`, use `--remap` during playback.
- **TF and playback**: TF transforms in the bag may conflict with a running robot. Use namespacing or topic remapping to avoid conflicts.
