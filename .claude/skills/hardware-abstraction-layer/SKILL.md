<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: hardware-abstraction-layer
description: 'Abstract hardware behind ROS 2 interfaces. Use when designing driver nodes, using ros2_control, or swapping hardware transparently.'
---

# Hardware Abstraction Layer in ROS 2

## Core Principle

Every hardware-interfacing node publishes and subscribes to standard ROS 2 message types. No other node in the system knows or cares what hardware is actually connected. The motor controller could be a RoboClaw, ODrive, or a simulated model — the navigation stack sees the same `Twist` command and `Odometry` feedback regardless.

## The Abstraction Pattern

```
┌─────────────────────┐   /cmd_vel    ┌───────────────────┐
│  Navigation Stack   │ ────────────→ │  Motor Driver Node │ ──→ Hardware
│  (Nav2, BT, etc.)   │ ←──────────── │  (publishes Odom)  │ ←── Hardware
└─────────────────────┘   /odom       └───────────────────┘

┌─────────────────────┐   /scan       ┌───────────────────┐
│  SLAM / Costmap     │ ←──────────── │  LIDAR Driver Node │ ←── Hardware
└─────────────────────┘               └───────────────────┘

┌─────────────────────┐   /imu/data   ┌───────────────────┐
│  EKF Localization   │ ←──────────── │  IMU Driver Node   │ ←── Hardware
└─────────────────────┘               └───────────────────┘
```

Each driver node is a **thin adapter**: it translates between hardware-specific protocols (serial packets, I2C registers, CAN frames) and standard ROS 2 interfaces.

## Standard Interface Contracts

### Differential Drive

| Direction | Topic | Type | Content |
|-----------|-------|------|---------|
| In | `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` (m/s), `angular.z` (rad/s) |
| Out | `/odom` | `nav_msgs/Odometry` | Pose + twist with covariances |
| Out | TF: `odom → base_link` | Transform | Same pose as odometry |

### LIDAR

| Direction | Topic | Type |
|-----------|-------|------|
| Out | `/scan` | `sensor_msgs/LaserScan` |

### IMU

| Direction | Topic | Type |
|-----------|-------|------|
| Out | `/imu/data_raw` | `sensor_msgs/Imu` (no orientation) |
| Out | `/imu/data` | `sensor_msgs/Imu` (with orientation, post-filter) |
| Out | `/imu/mag` | `sensor_msgs/MagneticField` |

### Camera

| Direction | Topic | Type |
|-----------|-------|------|
| Out | `/camera/image_raw` | `sensor_msgs/Image` |
| Out | `/camera/camera_info` | `sensor_msgs/CameraInfo` |
| Out | `/camera/depth/image_raw` | `sensor_msgs/Image` (depth cameras) |

### Battery

| Direction | Topic | Type |
|-----------|-------|------|
| Out | `/battery/state` | `sensor_msgs/BatteryState` |

## Benefits of Abstraction

### 1. Hardware Swapping

Replace a RoboClaw with an ODrive: only the motor driver node changes. Navigation params, behavior trees, SLAM — all untouched.

```python
# Before: RoboClaw driver
Node(package='roboclaw_driver', executable='roboclaw_node', ...)

# After: ODrive driver — publishes the same /odom, subscribes to the same /cmd_vel
Node(package='odrive_driver', executable='odrive_node', ...)
```

### 2. Simulation Transparency

Gazebo publishes the same topics (`/odom`, `/scan`, `/imu/data`, `/camera/image_raw`) as real hardware. Navigation code runs identically in simulation and on the real robot:

```python
# Launch condition: use real drivers or simulated
if use_sim_time:
    # Gazebo provides all sensor/actuator topics
    pass
else:
    # Launch real hardware drivers
    motor_driver = Node(package='motor_driver', ...)
    lidar_driver = Node(package='ldlidar_stl_ros2', ...)
```

### 3. Independent Testing

Each driver can be tested in isolation:

```bash
# Test motor driver with manual Twist commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 topic echo /odom

# Test LIDAR driver
ros2 topic echo /scan --once
ros2 run rviz2 rviz2  # visualize scan
```

## Example: Motor Driver Abstraction Node

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

class MotorDriverNode(Node):
    """Abstract motor driver. Receives Twist, publishes Odometry.
    Subclass and override send_wheel_speeds() and read_encoders()
    for specific hardware."""
    
    def __init__(self):
        super().__init__('motor_driver')
        
        self.declare_parameter('wheel_separation', 0.3)
        self.declare_parameter('wheel_diameter', 0.09)
        self.declare_parameter('serial_port', '/dev/motor_controller')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_dia = self.get_parameter('wheel_diameter').value
        
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.odom_timer = self.create_timer(0.02, self.update_odometry)  # 50 Hz
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z
        
        # Inverse kinematics: Twist → wheel velocities
        v_left = v - (omega * self.wheel_sep / 2.0)
        v_right = v + (omega * self.wheel_sep / 2.0)
        
        self.send_wheel_speeds(v_left, v_right)

    def send_wheel_speeds(self, v_left: float, v_right: float):
        """Override in hardware-specific subclass."""
        raise NotImplementedError

    def read_encoders(self) -> tuple:
        """Override: return (delta_left_m, delta_right_m) since last call."""
        raise NotImplementedError

    def update_odometry(self):
        dl, dr = self.read_encoders()
        
        d_center = (dl + dr) / 2.0
        d_theta = (dr - dl) / self.wheel_sep
        
        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        
        now = self.get_clock().now().to_msg()
        
        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.get_parameter('odom_frame').value
        odom.child_frame_id = self.get_parameter('base_frame').value
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.odom_pub.publish(odom)
        
        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = odom.header.frame_id
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
```

## ros2_control Framework

For robots with complex actuator configurations (multi-joint arms, legged robots, or when you need position/velocity/effort interfaces), ros2_control provides a formal hardware abstraction.

### Architecture

```
Controller Manager
  ├── diff_drive_controller (plugin) ← subscribes /cmd_vel, publishes /odom
  ├── joint_state_broadcaster (plugin) ← publishes /joint_states
  └── ...

Hardware Interface (SystemInterface)
  ├── read()  → reads encoder counts, IMU, etc. from hardware
  └── write() → sends motor commands to hardware
```

### When to Use ros2_control

- Multi-actuator robots (arm + mobile base)
- You need standardized position/velocity/effort control interfaces
- You want to hot-swap controllers (e.g., switch from velocity to position mode)
- The controller plugins (diff_drive_controller, joint_trajectory_controller) fit your needs

### When NOT to Use ros2_control

- Simple differential drive with a motor controller that does its own PID (RoboClaw)
- The controller overhead isn't justified (adds complexity, latency, configuration)
- You need tight integration with custom safety logic that doesn't fit the controller architecture
- Prototyping — a bare publisher/subscriber driver is faster to develop

### Minimal ros2_control Setup

URDF hardware interface definition:
```xml
<ros2_control name="RobotSystem" type="system">
  <hardware>
    <plugin>my_robot_hardware/MyRobotHardware</plugin>
    <param name="serial_port">/dev/motor_controller</param>
  </hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

Controller config:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 50
    
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController
    
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.30
    wheel_radius: 0.045
    publish_rate: 50.0
    odom_frame_id: "odom"
    base_frame_id: "base_link"
    use_stamped_vel: false
```

## Pattern Summary

| Complexity | Approach | Example |
|-----------|----------|---------|
| Simple | Custom node: subscribe Twist, publish Odometry | Single motor controller, prototyping |
| Medium | Custom node + parameter-driven config | Production differential drive |
| Complex | ros2_control SystemInterface + controller plugins | Multi-joint, needs controller switching |

The key insight: regardless of which approach you choose, the **external interface** to the rest of the ROS 2 system remains the same standard topics. That is the abstraction.
