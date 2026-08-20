<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: imu-integration
description: 'Integrate IMU data for localization. Use when configuring IMU input to EKF/UKF, setting covariance, or handling orientation drift.'
---

# IMU Integration for Navigation

## The sensor_msgs/Imu Message

```
Header header
geometry_msgs/Quaternion orientation           # Absolute orientation (if available)
float64[9] orientation_covariance              # Row-major 3x3 (roll, pitch, yaw)
geometry_msgs/Vector3 angular_velocity         # rad/s about each axis
float64[9] angular_velocity_covariance         # Row-major 3x3
geometry_msgs/Vector3 linear_acceleration      # m/s² along each axis
float64[9] linear_acceleration_covariance      # Row-major 3x3
```

If the IMU does not provide orientation, set `orientation_covariance[0] = -1` to indicate unavailability. Consumers (like robot_localization) will skip that field.

---

## Frame Conventions (REP 103)

The IMU frame in your URDF must follow REP 103:
- **X** → forward
- **Y** → left
- **Z** → up
- Right-hand rule for rotations

### Common Axis Issues

Many IMU breakout boards use a different convention (e.g., NED: North-East-Down). Symptoms of misaligned axes:
- Robot turns left but IMU reports right rotation
- Yaw drifts when driving straight (axes swapped)
- Gravity appears on X or Y instead of Z

Fix: Add a `static_transform_publisher` in your launch file to rotate the raw IMU frame to REP 103 convention:

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
)
```

Adjust the rotation (quaternion or RPY) to match your physical IMU mounting. Verify with `ros2 topic echo /imu/data` — gravity should appear as ~9.81 on `linear_acceleration.z` when the robot is at rest and level.

---

## IMU Orientation Filters

Raw IMU typically provides only gyroscope and accelerometer data. An orientation filter fuses these into a quaternion orientation estimate.

### imu_filter_madgwick

```yaml
imu_filter_madgwick_node:
  ros__parameters:
    use_mag: false              # Disable magnetometer if not available or unreliable indoors
    publish_tf: false           # Let robot_localization handle TF
    world_frame: "enu"          # East-North-Up (REP 103 compatible)
    gain: 0.1                   # Filter gain (lower = smoother but slower response)
    zgyro_bias: 0.0             # Static gyro bias correction (calibrate at rest)
    frequency: 100.0            # Filter update rate — match IMU publish rate
```

### imu_complementary_filter

```yaml
complementary_filter_node:
  ros__parameters:
    do_bias_estimation: true
    do_adaptive_gain: true
    use_mag: false
    publish_tf: false
    gain_acc: 0.01              # Accelerometer gain (lower = trust gyro more)
    gain_mag: 0.01              # Magnetometer gain (irrelevant if use_mag: false)
```

Both filters subscribe to raw `/imu/data_raw` (without orientation) and publish to `/imu/data` (with orientation filled in).

---

## Feeding IMU to robot_localization EKF

### What to Fuse

For a 2D differential-drive robot:

| IMU Field | Fuse? | Rationale |
|-----------|-------|-----------|
| orientation.yaw | **Yes** | Absolute heading reference, corrects gyro drift |
| orientation.roll | No | Locked by `two_d_mode` |
| orientation.pitch | No | Locked by `two_d_mode` |
| angular_velocity.z | **Yes** | Direct yaw rate measurement |
| angular_velocity.x, y | No | Not relevant in 2D |
| linear_acceleration.x | **Yes** | Forward acceleration aids velocity estimation |
| linear_acceleration.y, z | No | Lateral and vertical not useful in 2D mode |

### EKF Configuration

```yaml
imu0: "imu/data"
imu0_config: [false, false, false,    # No position from IMU
              false, false, true,     # Yaw only
              false, false, false,    # No velocity from IMU
              false, false, true,     # vyaw
              true,  false, false]    # ax only
imu0_differential: false
imu0_relative: false
imu0_remove_gravitational_acceleration: true
```

---

## Gravity Removal

The accelerometer always reads gravitational acceleration (~9.81 m/s² on Z). If the IMU driver does **not** subtract gravity, you must set:

```yaml
imu0_remove_gravitational_acceleration: true
```

robot_localization will use the current orientation estimate to compute and subtract the gravity vector. If the IMU driver already reports acceleration without gravity (common with filtered IMU drivers), set this to `false` to avoid double-subtracting.

**Verify**: With the robot at rest on a level surface:
- **With gravity**: `linear_acceleration.z ≈ 9.81`
- **Without gravity**: `linear_acceleration.z ≈ 0.0`

---

## Static Calibration

Record IMU data while the robot is perfectly still for 30–60 seconds:

```bash
ros2 bag record /imu/data -o imu_calibration --duration 60
```

Compute bias offsets:

```python
import numpy as np
# Load bag data, extract angular_velocity and linear_acceleration
# At rest, all angular_velocity should be 0, accel should be [0, 0, 9.81]
gyro_bias_x = np.mean(gyro_x_samples)
gyro_bias_y = np.mean(gyro_y_samples)
gyro_bias_z = np.mean(gyro_z_samples)
accel_bias_x = np.mean(accel_x_samples)  # Should be ~0
accel_bias_y = np.mean(accel_y_samples)  # Should be ~0
```

Apply these biases either in the IMU driver configuration or in the Madgwick filter's bias parameters.

---

## Magnetic Declination

If using the magnetometer for absolute yaw (outdoors):

```yaml
magnetic_declination_radians: 0.0  # Look up for your location at ngdc.noaa.gov
```

Indoors, magnetometers are unreliable due to metal structures, motors, and wiring. Set `use_mag: false`.

---

## Full Pipeline

```
Raw IMU (gyro + accel)
        │
        ▼
  Madgwick / Complementary Filter
  (fuses gyro + accel → orientation quaternion)
        │
        ▼
  /imu/data (full sensor_msgs/Imu with orientation)
        │
        ▼
  robot_localization EKF
  (fuses yaw, vyaw, ax with wheel odometry)
        │
        ▼
  /odometry/filtered (fused nav_msgs/Odometry)
  odom → base_link TF
```

---

## Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| Yaw drifts continuously | Gyro bias not calibrated, or no absolute heading reference | Calibrate at rest, ensure orientation filter provides yaw |
| Yaw snaps when turning | IMU axes inverted | Check URDF frame, add static TF to correct orientation |
| EKF ignores IMU entirely | Topic name mismatch or covariance set to -1 | Verify `ros2 topic echo /imu/data`, check covariance fields |
| Robot tilts in RViz but is level | Roll/pitch fused when `two_d_mode` should be `true` | Ensure `two_d_mode: true` in EKF config |
| Acceleration causes position overshoot | IMU acceleration too noisy or gravity not removed | Increase acceleration covariance or remove ax from fusion |
