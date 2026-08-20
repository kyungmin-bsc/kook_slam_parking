<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: visual-odometry
description: 'Set up visual odometry for ROS 2. Use when using camera-based ego-motion estimation with RTAB-Map, ORB-SLAM, or libviso2.'
---

# Visual Odometry for Robot Localization

## Overview

Visual odometry (VO) estimates robot motion by tracking visual features across consecutive camera frames. It provides a `nav_msgs/Odometry` message that can be fused with other sources in the robot_localization EKF. Particularly valuable when wheels slip (smooth floors, carpets, outdoor terrain).

---

## Available Packages (ROS 2)

| Package | Input | Method | Notes |
|---------|-------|--------|-------|
| `rtabmap_odom` | Stereo or RGB-D | Feature-based or ICP | Part of RTAB-Map, well-maintained |
| `viso2_ros` | Stereo | Libviso2 feature matching | Lightweight, stereo only |
| `ORB-SLAM3` | Mono/stereo/IMU | ORB features + bundle adjustment | State-of-the-art accuracy, heavy |
| DepthAI (OAK-D) | Stereo + IMU | On-chip VIO | Hardware-accelerated, low CPU |

---

## RTAB-Map Visual Odometry

### Stereo Configuration

```yaml
rtabmap_odom_node:
  ros__parameters:
    frame_id: "base_link"
    odom_frame_id: "vo_odom"
    publish_tf: false                  # EKF handles TF
    wait_for_transform_duration: 0.2
    approx_sync: true
    subscribe_stereo: true
    # Feature detection
    Odom/Strategy: "0"                 # 0=Frame-to-Map, 1=Frame-to-Frame
    Odom/EstimationType: "1"          # 0=3D, 1=2D (for ground robots)
    OdomF2M/MaxSize: 2000             # Max features in local map
    Vis/FeatureType: "8"              # 0=SURF, 8=ORB
    Vis/MaxFeatures: 1000
    Vis/MinInliers: 20                # Minimum inliers to accept motion estimate
```

### Subscribed Topics

```
/camera/left/image_rect   (sensor_msgs/Image)
/camera/right/image_rect  (sensor_msgs/Image)
/camera/left/camera_info  (sensor_msgs/CameraInfo)
/camera/right/camera_info (sensor_msgs/CameraInfo)
```

---

## OAK-D On-Chip Visual-Inertial Odometry

The OAK-D camera can compute visual-inertial odometry (VIO) entirely on its VPU, offloading the CPU:

```python
# In your DepthAI pipeline (depthai-ros)
Node(
    package='depthai_ros_driver',
    executable='camera.launch.py',
    parameters=[{
        'camera.i_enable_imu': True,
        'camera.i_publish_tf_from_calibration': False,
    }],
)
```

The DepthAI driver publishes odometry on `/stereo_inertial_publisher/odometry`. This can be fed directly into the EKF.

---

## Fusing Visual Odometry with EKF

### Key: Use Differential Mode

Visual odometry pose accumulates drift over time. If you fuse the raw absolute pose, the EKF will eventually see conflicting positions from VO and wheel odom. Use `differential: true` to convert absolute poses into incremental (velocity-like) measurements:

```yaml
ekf_filter_node:
  ros__parameters:
    # Primary: wheel odometry
    odom0: "odom/unfiltered"
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  false, false,
                   false, false, true,
                   false, false, false]

    # Secondary: visual odometry (differential mode)
    odom1: "vo/odometry"
    odom1_config: [true,  true,  false,   # Fuse x, y from VO (as differential)
                   false, false, true,    # Fuse yaw from VO (as differential)
                   false, false, false,
                   false, false, false,
                   false, false, false]
    odom1_differential: true              # Converts absolute pose to velocity
    odom1_relative: false
    odom1_queue_size: 5

    # IMU
    imu0: "imu/data"
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  true,  false, false]
    imu0_remove_gravitational_acceleration: true
```

### Why Differential?

When `odom1_differential: true`:
1. EKF stores each absolute pose from VO
2. On the next measurement, it computes the delta (difference)
3. This delta is treated as a velocity measurement
4. Absolute drift cancels out — only frame-to-frame motion is used

---

## Advantages and Disadvantages

### Advantages
- Works on slippery surfaces where wheel encoders fail
- Independent of wheel contact — useful for ramps, thresholds
- Provides lateral motion detection (vy) that wheel odom cannot

### Disadvantages
- **Lighting sensitivity**: performance degrades in dark or rapidly changing illumination
- **Feature-poor environments**: blank walls, ceilings, uniform carpets provide no trackable features
- **Computational cost**: 50–200ms per frame for CPU-based VO (OAK-D VPU mitigates this)
- **Motion blur**: fast rotation or acceleration blurs features, causing tracking loss
- **Scale drift**: monocular VO has scale ambiguity (stereo and VIO do not)

---

## Failure Detection

VO methods report inlier counts and confidence. Monitor these to detect failures:

```python
class VOMonitor(Node):
    def __init__(self):
        super().__init__('vo_monitor')
        self.create_subscription(Odometry, 'vo/odometry', self.vo_cb, 10)
        self.last_stamp = None

    def vo_cb(self, msg):
        # Check for stale data (VO failed to compute)
        if self.last_stamp and msg.header.stamp == self.last_stamp:
            self.get_logger().warn('VO output stale — possible tracking loss')

        # Check covariance spike (VO uncertain)
        if msg.pose.covariance[0] > 1.0:
            self.get_logger().warn('VO covariance high — degraded estimate')

        self.last_stamp = msg.header.stamp
```

When VO fails, the EKF gracefully falls back to wheel odom + IMU — no action needed if covariances are set properly. The EKF will simply weight the missing VO source as zero.

---

## Practical Tips

1. **Camera exposure**: Use auto-exposure with limits to prevent motion blur. On OAK-D: `camera.i_max_exposure_usec: 10000`
2. **Publish rate**: VO at 10–30 Hz is typical. Higher rates increase CPU load without proportional accuracy gain.
3. **Calibration**: Stereo calibration quality directly impacts VO accuracy. Recalibrate if the camera is bumped.
4. **Timestamps**: Ensure camera and wheel odom timestamps are synchronized. robot_localization interpolates based on timestamps.
