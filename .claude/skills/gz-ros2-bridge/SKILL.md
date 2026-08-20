<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: gz-ros2-bridge
description: 'Configure ros_gz_bridge. Use when bridging Gazebo and ROS 2 topics, setting up cmd_vel, odom, sensor, or TF bridges.'
---

# Gazebo–ROS 2 Bridge (ros_gz_bridge)

## Overview

The `ros_gz_bridge` package provides bidirectional message translation between Gazebo Transport and ROS 2 DDS. Gazebo uses its own transport layer (`gz::transport`) with its own message types (`gz.msgs.*`). The bridge subscribes on one side and re-publishes on the other.

## Bridge Syntax

```
TOPIC@ROS_TYPE@GZ_TYPE     # Bidirectional
TOPIC@ROS_TYPE[GZ_TYPE     # GZ → ROS only (subscribe GZ, publish ROS)
TOPIC@ROS_TYPE]GZ_TYPE     # ROS → GZ only (subscribe ROS, publish GZ)
```

The `[` means "from Gazebo" (Gazebo publishes, ROS subscribes). The `]` means "to Gazebo" (ROS publishes, Gazebo subscribes). `@` in the middle means bidirectional.

## Common Bridge Mappings

```bash
# Clock (GZ → ROS) — REQUIRED for use_sim_time
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

# Velocity commands (ROS → GZ)
/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist

# Odometry (GZ → ROS)
/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry

# Lidar scan (GZ → ROS)
/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan

# IMU (GZ → ROS)
/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU

# Camera image (GZ → ROS)
/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image

# Camera info (GZ → ROS)
/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo

# Depth image (GZ → ROS)
/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image

# Point cloud (GZ → ROS)
/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked

# Joint states (GZ → ROS)
/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model

# TF (GZ → ROS)
/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V
```

## Launch File with Bridge

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Gazebo sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': '-r house.sdf',
        }.items(),
    )

    # Bridge with multiple topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[
            # Remap if Gazebo topic names differ from expected ROS names
            # ('/model/my_robot/cmd_vel', '/cmd_vel'),
        ],
        output='screen',
    )

    return LaunchDescription([gz_sim, bridge])
```

## YAML Configuration (Preferred for Many Topics)

For complex setups, use a YAML config file instead of command-line arguments:

```yaml
# bridge_config.yaml
---
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/model/my_robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/odom"
  gz_topic_name: "/model/my_robot/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "/scan"
  gz_topic_name: "/lidar"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu/data"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/camera_info"
  gz_topic_name: "/camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS

- ros_topic_name: "/joint_states"
  gz_topic_name: "/world/house_patrol/model/my_robot/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
```

Launch with YAML:

```python
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['--ros-args', '-p',
               'config_file:=/path/to/bridge_config.yaml'],
    parameters=[{'use_sim_time': True}],
    output='screen',
)
```

## Gazebo Topic Naming

Gazebo internally namespaces topics by world and model:
- World clock: `/world/<world_name>/clock`
- Model cmd_vel: `/model/<model_name>/cmd_vel`
- Model odometry: `/model/<model_name>/odometry`
- Sensor data: the `<topic>` element from the sensor plugin definition

List all Gazebo topics:
```bash
gz topic -l
```

The bridge handles the mapping between these Gazebo-namespaced topics and flat ROS 2 topic names.

## Image Bridge

For camera topics, `ros_gz_image` is more efficient than the generic bridge because it avoids serialization overhead:

```python
image_bridge = Node(
    package='ros_gz_image',
    executable='image_bridge',
    arguments=['/camera/image_raw'],
    parameters=[{'use_sim_time': True}],
    output='screen',
)
```

## TF Bridge

The `ros_gz_bridge` can publish model poses as TF transforms. Alternatively, use the diff-drive plugin's built-in `<tf_topic>tf</tf_topic>` which directly publishes odom→base_link via the bridge.

For the model pose (world→base_link for ground truth):

```python
# Publish model pose as TF
pose_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/model/my_robot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    ],
    parameters=[{'use_sim_time': True}],
)
```

## Debugging Bridge Issues

```bash
# Check ROS 2 side
ros2 topic list
ros2 topic hz /scan

# Check Gazebo side
gz topic -l
gz topic -e -t /lidar

# Verify message types match
ros2 topic info /scan --verbose
gz topic -i -t /lidar

# Common issue: wrong direction symbol ([ vs ])
# If cmd_vel doesn't work: ensure ] (ROS→GZ), not [ (GZ→ROS)
```

## Supported Message Pairs

Not all message types are bridgeable. Check supported pairs:
```bash
ros2 run ros_gz_bridge parameter_bridge --help
```

Key supported pairs include Twist, Odometry, LaserScan, Image, Imu, PointCloud2, JointState, Clock, Pose, TFMessage, CameraInfo, NavSat, and Boolean.
