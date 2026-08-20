<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: imu-driver-config
description: 'Configure IMU drivers for ROS 2. Use when setting up imu_filter_madgwick, complementary filter, or publishing sensor_msgs/Imu.'
---

# IMU Driver Configuration for ROS 2

## The sensor_msgs/Imu Message

```
Header header
geometry_msgs/Quaternion orientation          # (x, y, z, w)
float64[9] orientation_covariance             # row-major 3x3

geometry_msgs/Vector3 angular_velocity        # rad/s
float64[9] angular_velocity_covariance

geometry_msgs/Vector3 linear_acceleration     # m/s²
float64[9] linear_acceleration_covariance
```

Conventions (REP 103/145):
- Frame: x-forward, y-left, z-up (right-hand rule)
- `orientation`: rotation from the IMU frame to a world-aligned frame (gravity = +z, north = +x if magnetometer used)
- Set the first element of a covariance matrix to `-1` if that measurement is not available

## Raw IMU Data from MCU

If using a custom serial protocol (not micro-ROS), the MCU sends raw accelerometer and gyroscope readings. The ROS 2 node converts them:

```python
def unpack_imu_data(payload: bytes) -> Imu:
    ax, ay, az, gx, gy, gz = struct.unpack('<hhhhhh', payload)
    
    msg = Imu()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'imu_link'
    
    # Accelerometer: convert from raw counts to m/s²
    # MPU6050 at ±2g: sensitivity = 16384 LSB/g
    ACCEL_SCALE = 9.80665 / 16384.0
    msg.linear_acceleration.x = ax * ACCEL_SCALE
    msg.linear_acceleration.y = ay * ACCEL_SCALE
    msg.linear_acceleration.z = az * ACCEL_SCALE
    
    # Gyroscope: convert from raw counts to rad/s
    # MPU6050 at ±250°/s: sensitivity = 131 LSB/(°/s)
    GYRO_SCALE = (1.0 / 131.0) * (math.pi / 180.0)
    msg.angular_velocity.x = gx * GYRO_SCALE
    msg.angular_velocity.y = gy * GYRO_SCALE
    msg.angular_velocity.z = gz * GYRO_SCALE
    
    # No orientation from raw data—set covariance[0] = -1
    msg.orientation_covariance[0] = -1.0
    
    return msg
```

## Static Covariance Configuration

Set covariance based on the IMU's datasheet noise specifications:

```python
# MPU6050 typical noise densities:
# Accelerometer: 400 μg/√Hz at 1 kHz bandwidth → σ² ≈ (400e-6 * 9.81)² * bandwidth
# Gyroscope: 0.005 °/s/√Hz → σ² ≈ (0.005 * π/180)² * bandwidth

accel_variance = 0.01      # m/s², conservative estimate
gyro_variance = 0.001      # rad/s, conservative estimate
orient_variance = 0.05     # rad, from filter output

msg.linear_acceleration_covariance = [
    accel_variance, 0.0, 0.0,
    0.0, accel_variance, 0.0,
    0.0, 0.0, accel_variance,
]

msg.angular_velocity_covariance = [
    gyro_variance, 0.0, 0.0,
    0.0, gyro_variance, 0.0,
    0.0, 0.0, gyro_variance,
]
```

## IMU Filter Packages

Raw accelerometer and gyroscope data do not provide orientation. A filter fuses them (and optionally magnetometer) to estimate orientation.

### imu_filter_madgwick

The most widely used ROS 2 IMU filter. Implements Madgwick's gradient-descent AHRS algorithm.

```yaml
# imu_filter_madgwick params
imu_filter_madgwick_node:
  ros__parameters:
    use_mag: false              # true if magnetometer available and calibrated
    publish_tf: false           # set true if no other node publishes imu transform
    world_frame: "enu"          # ENU (east-north-up) per REP 103
    fixed_frame: "odom"
    gain: 0.1                   # filter gain (higher = faster convergence, more noise)
    zeta: 0.0                   # gyro drift compensation (0 = disabled)
    frequency: 100.0            # expected IMU data rate in Hz
    
    # Remappings (set in launch file)
    # /imu/data_raw → raw accel + gyro
    # /imu/mag → magnetometer (if use_mag)
    # /imu/data → output with orientation
```

Launch file:
```python
Node(
    package='imu_filter_madgwick',
    executable='imu_filter_madgwick_node',
    name='imu_filter',
    parameters=[imu_filter_params],
    remappings=[
        ('/imu/data_raw', '/imu/data_raw'),
        ('/imu/data', '/imu/data'),
    ],
)
```

### imu_complementary_filter

Simpler filter, works well for many applications:

```yaml
complementary_filter_gain_node:
  ros__parameters:
    do_bias_estimation: true
    bias_alpha: 0.01
    do_adaptive_gain: true
    gain_acc: 0.01
    gain_mag: 0.01              # only if use_mag
    use_mag: false
    publish_tf: false
    fixed_frame: "odom"
```

## Gyro Bias Calibration

Gyroscope readings drift over time. At startup, the robot must be stationary for bias estimation:

```cpp
// MCU-side: average 500 readings at startup
int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
for (int i = 0; i < 500; i++) {
    readIMU(accel, gyro);
    gx_sum += gyro[0];
    gy_sum += gyro[1];
    gz_sum += gyro[2];
    delay(2);
}
int16_t gx_bias = gx_sum / 500;
int16_t gy_bias = gy_sum / 500;
int16_t gz_bias = gz_sum / 500;
```

Subtract the bias from all subsequent readings. Some filters (Madgwick with `zeta > 0`, complementary with `do_bias_estimation`) estimate bias online, but a startup calibration gives a better initial value.

## Magnetometer Calibration

If using a magnetometer for absolute heading (yaw), it must be calibrated for:

### Hard Iron Distortion

Constant magnetic field offsets from nearby ferromagnetic materials (motors, steel frames, batteries). Corrected by subtracting an offset vector:

$$
\vec{m}_{\text{cal}} = \vec{m}_{\text{raw}} - \vec{b}_{\text{hard}}
$$

### Soft Iron Distortion

Non-uniform scaling caused by nearby metals that distort the field shape. Corrected by a 3x3 matrix multiplication:

$$
\vec{m}_{\text{cal}} = A \cdot (\vec{m}_{\text{raw}} - \vec{b}_{\text{hard}})
$$

### Calibration Procedure

1. Use `rosrun imu_tools magnetometer_calibration` or the MotionCal tool
2. Rotate the robot slowly in all orientations (figure-8 pattern)
3. Collect data for 60+ seconds, covering all orientations
4. The tool outputs hard-iron offsets ($\vec{b}$) and soft-iron matrix ($A$)
5. Apply corrections either on the MCU or in the ROS driver

## Frame Alignment

The IMU's physical mounting orientation must match ROS conventions. If the IMU is mounted with its X-axis pointing backward:

```yaml
# In URDF: static transform from base_link to imu_link
<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0.0 0.0 0.1" rpy="0 0 3.14159"/>  # rotated 180° yaw
</joint>
```

Alternatively, apply a rotation in the driver node to remap axes before publishing.

## EKF Integration

The filtered IMU data feeds into `robot_localization`'s EKF:

```yaml
# ekf.yaml
ekf_filter_node:
  ros__parameters:
    imu0: /imu/data
    imu0_config: [false, false, false,     # x, y, z position: ignore
                  true,  true,  true,      # roll, pitch, yaw: use
                  false, false, false,     # vx, vy, vz: ignore
                  true,  true,  true,      # vroll, vpitch, vyaw: use
                  true,  true,  true]      # ax, ay, az: use (optional)
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true
    imu0_queue_size: 10
```

Key: `imu0_remove_gravitational_acceleration: true` tells the EKF to subtract gravity from the accelerometer readings before integrating for velocity. Without this, the robot thinks it's constantly accelerating upward at 9.81 m/s².

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Orientation drifts in yaw | No magnetometer and no yaw constraint | Use `use_mag: true` with calibrated mag, or fuse with wheel odometry via EKF |
| Roll/pitch offset at rest | IMU not level or axis misalignment | Check URDF transform, verify IMU mounting |
| Noisy angular velocity | Vibration coupling from motors | Mount IMU on vibration dampeners (rubber standoffs), increase filter gain |
| Filter diverges at startup | Large initial bias | Ensure robot is stationary for first 2 seconds, enable bias estimation |
| Heading jumps near motors | Magnetic interference | Move IMU away from motors/batteries, recalibrate magnetometer on the robot |
