<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: simulated-sensors
description: 'Simulate sensors in Gazebo. Use when configuring simulated IMU, lidar, camera, depth camera, GPS, or contact sensor noise models.'
---

# Simulated Sensors in Gazebo Harmonic

## Sensor System Architecture

All sensors in Gazebo Harmonic require the `gz-sim-sensors-system` world plugin. Individual sensor types may need additional system plugins (e.g., `gz-sim-imu-system` for IMU). Sensors attach to links via `<sensor>` elements inside `<gazebo reference="link_name">` blocks.

**Critical principle**: simulated sensor parameters must match real hardware—same FOV, resolution, update rate, and realistic noise levels. Mismatched sensor characteristics cause sim-to-real transfer failures where algorithms that work in simulation fail on the physical robot.

## IMU — Inertial Measurement Unit

Requires: `gz-sim-imu-system` + `gz-sim-sensors-system`

Realistic noise parameters for a typical MEMS IMU (e.g., BNO055):

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <topic>imu/data</topic>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0002</stddev>        <!-- rad/s, ~0.01 deg/s -->
          <bias_mean>0.0000075</bias_mean>  <!-- Slow gyro drift -->
          <bias_stddev>0.0000008</bias_stddev>
          <dynamic_bias_stddev>0.00000002</dynamic_bias_stddev>
          <dynamic_bias_correlation_time>400</dynamic_bias_correlation_time>
        </noise>
      </x>
      <y><noise type="gaussian"><mean>0.0</mean><stddev>0.0002</stddev>
        <bias_mean>0.0000075</bias_mean><bias_stddev>0.0000008</bias_stddev>
      </noise></y>
      <z><noise type="gaussian"><mean>0.0</mean><stddev>0.0002</stddev>
        <bias_mean>0.0000075</bias_mean><bias_stddev>0.0000008</bias_stddev>
      </noise></z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>           <!-- m/s², typical accelerometer -->
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
          <dynamic_bias_stddev>0.0001</dynamic_bias_stddev>
          <dynamic_bias_correlation_time>175</dynamic_bias_correlation_time>
        </noise>
      </x>
      <y><noise type="gaussian"><mean>0.0</mean><stddev>0.017</stddev>
        <bias_mean>0.1</bias_mean><bias_stddev>0.001</bias_stddev>
      </noise></y>
      <z><noise type="gaussian"><mean>0.0</mean><stddev>0.017</stddev>
        <bias_mean>0.1</bias_mean><bias_stddev>0.001</bias_stddev>
      </noise></z>
    </linear_acceleration>
    <!-- Enable gravity reference for orientation -->
    <enable_orientation>true</enable_orientation>
  </imu>
</sensor>
```

The noise model is Gaussian with additive bias drift. `dynamic_bias_stddev` and `dynamic_bias_correlation_time` model slowly-varying bias using a first-order Gauss-Markov process. These parameters should come from your IMU's datasheet (noise density × √bandwidth).

## Lidar

Requires: `gz-sim-sensors-system` (the sensors system handles rendering-based sensors including gpu_lidar)

Use `gpu_lidar` for performance; `lidar` uses CPU raycasting and is much slower.

```xml
<sensor name="lidar" type="gpu_lidar">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <topic>scan</topic>
  <visualize>true</visualize>
  <lidar>
    <scan>
      <horizontal>
        <samples>720</samples>          <!-- Match real lidar: LD19 = 160-320/rev -->
        <resolution>1</resolution>      <!-- 1 = use all samples -->
        <min_angle>-3.14159</min_angle>  <!-- -180 deg -->
        <max_angle>3.14159</max_angle>   <!--  180 deg -->
      </horizontal>
      <vertical>
        <samples>1</samples>            <!-- 2D lidar = 1 vertical sample -->
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.12</min>                   <!-- Match real minimum range -->
      <max>12.0</max>                   <!-- Match real maximum range -->
      <resolution>0.01</resolution>     <!-- Range resolution in meters -->
    </range>
    <noise type="gaussian">
      <mean>0.0</mean>
      <stddev>0.01</stddev>             <!-- 1cm stddev, typical for budget lidars -->
    </noise>
  </lidar>
</sensor>
```

For 3D lidars (e.g., simulating Velodyne), increase vertical samples and set appropriate vertical angles.

## Camera

```xml
<sensor name="camera" type="camera">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <topic>camera/image_raw</topic>
  <camera>
    <horizontal_fov>1.2</horizontal_fov>  <!-- ~69 degrees, typical webcam -->
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
    <distortion>
      <k1>0.0</k1><k2>0.0</k2><k3>0.0</k3>
      <p1>0.0</p1><p2>0.0</p2>
      <center>0.5 0.5</center>
    </distortion>
  </camera>
</sensor>
```

## Depth Camera / RGBD

For OAK-D simulation, use `rgbd_camera` which publishes both color and depth:

```xml
<sensor name="oakd" type="rgbd_camera">
  <always_on>true</always_on>
  <update_rate>15</update_rate>
  <topic>oakd</topic>
  <camera>
    <horizontal_fov>1.20428</horizontal_fov>  <!-- OAK-D: ~69 deg -->
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip><near>0.2</near><far>10.0</far></clip>
    <depth_camera>
      <clip><near>0.2</near><far>10.0</far></clip>
    </depth_camera>
    <noise type="gaussian">
      <mean>0.0</mean>
      <stddev>0.005</stddev>
    </noise>
  </camera>
</sensor>
```

This produces topics: `oakd/image` (color), `oakd/depth_image` (depth), `oakd/points` (point cloud), `oakd/camera_info`.

## Contact Sensor

Detects physical collisions—useful for bumper simulation:

```xml
<gazebo reference="bumper_link">
  <sensor name="bumper_contact" type="contact">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <topic>bumper/contact</topic>
    <contact>
      <collision>bumper_link_collision</collision>
    </contact>
  </sensor>
</gazebo>
<gazebo>
  <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
</gazebo>
```

## NavSat (GPS)

```xml
<sensor name="gps" type="navsat">
  <always_on>true</always_on>
  <update_rate>1</update_rate>
  <topic>gps/fix</topic>
  <navsat>
    <position_sensing>
      <horizontal><noise type="gaussian"><mean>0</mean><stddev>1.5</stddev></noise></horizontal>
      <vertical><noise type="gaussian"><mean>0</mean><stddev>3.0</stddev></noise></vertical>
    </position_sensing>
    <velocity_sensing>
      <horizontal><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></horizontal>
      <vertical><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></vertical>
    </velocity_sensing>
  </navsat>
</sensor>
```

Requires `<spherical_coordinates>` in the world SDF to define the reference lat/lon.

## Sim-to-Real Checklist

| Parameter | Must Match | Why |
|-----------|-----------|-----|
| FOV | Within 5% | Costmap coverage, object visibility |
| Resolution | Exact or scaled | Processing pipeline assumptions |
| Update rate | Same Hz | Filter tuning, timing assumptions |
| Min/max range | Exact | Costmap clearing, obstacle detection bounds |
| Noise stddev | Same order of magnitude | Filter convergence, false positive rates |
| Frame ID / TF | Exact | Entire processing pipeline depends on frames |
