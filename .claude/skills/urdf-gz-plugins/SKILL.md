<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: urdf-gz-plugins
description: 'Add Gazebo plugins to URDF/Xacro. Use when configuring gz-sim-diff-drive, joint-state-publisher, IMU, lidar, or camera simulation plugins.'
---

# Adding Gazebo Plugins to URDF/Xacro

## Plugin Architecture

Gazebo Harmonic plugins are loaded via `<plugin>` elements inside `<gazebo>` tags in URDF/Xacro. The `<gazebo>` tag either targets a specific link/joint (`<gazebo reference="link_name">`) or is global (no reference). Plugin filenames follow the pattern `gz-sim-<name>-system` and the fully-qualified class name goes in `name=`.

## Differential Drive

The core mobility plugin for differential drive robots. Subscribes to `cmd_vel`, publishes odometry and TF.

```xml
<xacro:macro name="diff_drive_plugin" params="left_joint right_joint">
  <gazebo>
    <plugin filename="gz-sim-diff-drive-system"
            name="gz::sim::systems::DiffDrive">
      <!-- Joint names must match URDF joint definitions -->
      <left_joint>${left_joint}</left_joint>
      <right_joint>${right_joint}</right_joint>

      <!-- Kinematic parameters - must match physical robot -->
      <wheel_separation>0.34</wheel_separation>
      <wheel_radius>0.05</wheel_radius>
      <wheel_diameter>0.10</wheel_diameter>

      <!-- Velocity/acceleration limits -->
      <max_linear_acceleration>1.0</max_linear_acceleration>
      <max_angular_acceleration>2.0</max_angular_acceleration>
      <max_linear_velocity>0.5</max_linear_velocity>
      <max_angular_velocity>1.5</max_angular_velocity>

      <!-- Odometry -->
      <odom_publish_frequency>50</odom_publish_frequency>
      <odom_topic>odom</odom_topic>
      <tf_topic>tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>

      <!-- Input -->
      <topic>cmd_vel</topic>
    </plugin>
  </gazebo>
</xacro:macro>
```

## Joint State Publisher

Publishes all movable joint positions to `/world/<world>/model/<model>/joint_state`. Bridge this to ROS 2 `/joint_states`.

```xml
<gazebo>
  <plugin filename="gz-sim-joint-state-publisher-system"
          name="gz::sim::systems::JointStatePublisher">
    <joint_name>left_wheel_joint</joint_name>
    <joint_name>right_wheel_joint</joint_name>
    <topic>joint_states</topic>
    <update_rate>50</update_rate>
  </plugin>
</gazebo>
```

## IMU Sensor

Attach to a link. Requires `gz-sim-imu-system` at the world level and `gz-sim-sensors-system`.

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <topic>imu</topic>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0002</stddev>
          <bias_mean>0.0000075</bias_mean>
          <bias_stddev>0.0000008</bias_stddev>
        </noise></x>
        <!-- Repeat for y, z -->
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise></x>
        <!-- Repeat for y, z -->
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>

<!-- World-level IMU system plugin (in SDF or robot gazebo block) -->
<gazebo>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
</gazebo>
```

## Lidar (GPU-accelerated)

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_lidar">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <topic>scan</topic>
    <lidar>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>1</samples>
          <resolution>1</resolution>
          <min_angle>0</min_angle>
          <max_angle>0</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.12</min>
        <max>12.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise type="gaussian">
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </lidar>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

## Camera

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <topic>camera/image_raw</topic>
    <camera>
      <horizontal_fov>1.2</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
      <noise type="gaussian">
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
  </sensor>
</gazebo>
```

## Depth Camera (RGBD)

For OAK-D or RealSense simulation:

```xml
<gazebo reference="depth_camera_link">
  <sensor name="rgbd_camera" type="rgbd_camera">
    <always_on>true</always_on>
    <update_rate>15</update_rate>
    <topic>depth_camera</topic>
    <camera>
      <horizontal_fov>1.20428</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.2</near>
        <far>10.0</far>
      </clip>
      <depth_camera>
        <clip>
          <near>0.2</near>
          <far>10.0</far>
        </clip>
      </depth_camera>
    </camera>
  </sensor>
</gazebo>
```

## Complete Xacro Snippet — Diff-Drive Robot

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot_sim">
  <xacro:include filename="$(find description)/urdf/my_robot.urdf.xacro"/>

  <!-- World-level systems -->
  <gazebo>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  </gazebo>

  <!-- Diff drive (references joints from base URDF) -->
  <xacro:diff_drive_plugin left_joint="left_wheel_joint" right_joint="right_wheel_joint"/>

  <!-- Wheel friction (critical for accurate driving) -->
  <gazebo reference="left_wheel_link">
    <collision>
      <surface>
        <friction>
          <ode><mu>1.0</mu><mu2>1.0</mu2></ode>
        </friction>
      </surface>
    </collision>
  </gazebo>

  <!-- IMU, lidar, camera sensors as shown above -->
</robot>
```

## Key Gotchas

- Plugin `filename` uses hyphens (`gz-sim-diff-drive-system`), not underscores
- `name` uses C++ namespace (`gz::sim::systems::DiffDrive`)
- Sensor plugins require the parent `gz-sim-sensors-system` plugin at world level
- Topic names are Gazebo transport topics—they need bridging to ROS 2 via `ros_gz_bridge`
- Collision geometry should be simpler than visual geometry for performance
