<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: sensor-frame-mounting
description: 'Mount sensor frames in URDF. Use when adding camera, lidar, IMU, or GPS frame links and transforms to the robot description.'
---

# Sensor Frame Mounting in URDF

## Principle

Every sensor on the robot needs a frame in the TF tree. This frame is defined by a fixed joint in the URDF from the mounting link to the sensor link. The frame's name **must match** the `frame_id` parameter in the sensor's ROS driver.

```xml
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.10 0 0.15" rpy="0 0 0"/>
</joint>
<link name="lidar_link"/>
```

The `origin` encodes the physical position and orientation of the sensor relative to the parent link. **Measure these offsets carefully**—errors here directly corrupt sensor data in the robot's coordinate frame.

## Camera Frames

Cameras use an **optical frame** convention that differs from the ROS body convention:

| Convention | X | Y | Z |
|---|---|---|---|
| ROS body (REP 103) | forward | left | up |
| Optical frame | right | down | forward |

Use an intermediate link to handle the rotation:

```xml
<!-- Camera body frame: x-forward, z-up (ROS convention) -->
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.15 0 0.10" rpy="0 0 0"/>
</joint>
<link name="camera_link"/>

<!-- Optical frame: z-forward, x-right, y-down -->
<joint name="camera_optical_joint" type="fixed">
  <parent link="camera_link"/>
  <child link="camera_optical_link"/>
  <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
</joint>
<link name="camera_optical_link"/>
```

The rotation `rpy="${-pi/2} 0 ${-pi/2}"` converts from ROS body to optical frame:
1. Rotate -π/2 around x → z now points down, y points forward
2. Rotate -π/2 around z → x now points right, y points down, z points forward

Set the camera driver's `frame_id` to `camera_optical_link`.

## OAK-D Camera (Stereo + IMU)

The OAK-D has multiple sensors, each needing its own frame:

```xml
<!-- Main camera body -->
<joint name="oakd_joint" type="fixed">
  <parent link="base_link"/>
  <child link="oakd_link"/>
  <origin xyz="0.12 0 0.08" rpy="0 0 0"/>
</joint>
<link name="oakd_link"/>

<!-- RGB camera optical frame -->
<joint name="oakd_rgb_optical_joint" type="fixed">
  <parent link="oakd_link"/>
  <child link="oakd_rgb_camera_optical_frame"/>
  <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
</joint>
<link name="oakd_rgb_camera_optical_frame"/>

<!-- Left stereo camera (offset from center) -->
<joint name="oakd_left_joint" type="fixed">
  <parent link="oakd_link"/>
  <child link="oakd_left_camera_frame"/>
  <origin xyz="0 0.0375 0" rpy="0 0 0"/>
</joint>
<link name="oakd_left_camera_frame"/>

<!-- Left stereo optical frame -->
<joint name="oakd_left_optical_joint" type="fixed">
  <parent link="oakd_left_camera_frame"/>
  <child link="oakd_left_camera_optical_frame"/>
  <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
</joint>
<link name="oakd_left_camera_optical_frame"/>
```

## Lidar Frame

Lidars typically follow ROS convention (x-forward, z-up). The critical parameter is the **height** (z offset):

```xml
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <!-- Lidar mounted 15cm above base_link origin, 10cm forward -->
  <origin xyz="0.10 0 0.15" rpy="0 0 0"/>
</joint>
<link name="lidar_link"/>
```

If the lidar is mounted upside-down, add a π rotation around x:

```xml
<origin xyz="0.10 0 0.15" rpy="${pi} 0 0"/>
```

**Wrong lidar height** is the most common cause of the costmap seeing the ground as an obstacle. If the scan hits the floor in simulation, the z-offset is too low.

## IMU Frame

The IMU frame orientation determines how roll/pitch/yaw are interpreted by the EKF:

```xml
<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
<link name="imu_link"/>
```

If the physical IMU is mounted rotated (e.g., 90° yaw), encode that in the origin:

```xml
<origin xyz="0 0 0.05" rpy="0 0 ${pi/2}"/>
```

Alternatively, handle the rotation in the EKF configuration or the IMU driver. Choose ONE place to apply the correction—never both.

For `robot_localization` EKF, the IMU frame_id and the URDF link name must match, and the transform between `base_link` and `imu_link` must be correct. The EKF uses this transform to rotate IMU data into the robot's body frame.

## GPS Frame

For outdoor robots, the GPS antenna position matters:

```xml
<joint name="gps_joint" type="fixed">
  <parent link="base_link"/>
  <child link="gps_link"/>
  <origin xyz="-0.05 0 0.25" rpy="0 0 0"/>
</joint>
<link name="gps_link"/>
```

The GPS frame position affects the `navsat_transform_node` in `robot_localization`—it accounts for the lever arm between GPS antenna and the robot's base frame.

## Sensor Mounting Xacro Macro

Reusable macro for simple sensor mounts:

```xml
<xacro:macro name="sensor_mount" params="name parent xyz rpy">
  <joint name="${name}_joint" type="fixed">
    <parent link="${parent}"/>
    <child link="${name}_link"/>
    <origin xyz="${xyz}" rpy="${rpy}"/>
  </joint>
  <link name="${name}_link">
    <visual>
      <geometry><box size="0.02 0.02 0.02"/></geometry>
      <material name="dark_gray"><color rgba="0.3 0.3 0.3 1"/></material>
    </visual>
  </link>
</xacro:macro>

<!-- Usage -->
<xacro:sensor_mount name="front_sonar" parent="base_link"
                    xyz="0.18 0 0.05" rpy="0 0 0"/>
<xacro:sensor_mount name="rear_sonar" parent="base_link"
                    xyz="-0.18 0 0.05" rpy="0 0 ${pi}"/>
```

## Verification

```bash
# Check that sensor frames exist in the TF tree
ros2 run tf2_tools view_frames

# Verify transform between base_link and sensor
ros2 run tf2_ros tf2_echo base_link camera_optical_link
```

Common pitfalls:
- Driver's `frame_id` doesn't match URDF link name → data appears at wrong location in RViz
- Optical frame rotation wrong → image appears flipped or rotated
- Sensor height off by a few cm → lidar sees ground, camera FOV is wrong
