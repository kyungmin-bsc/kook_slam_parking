<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: multi-sensor-fusion
description: 'Fuse multiple sensor sources for localization. Use when combining wheel odom + IMU + lidar + visual odometry in a single EKF/UKF.'
---

# Multi-Sensor Fusion — Advanced Patterns

## The Cardinal Rule

**Never fuse the same physical measurement twice.** Every dimension in the EKF state vector should be informed by at most one absolute source. Multiple velocity sources for the same dimension are acceptable (the EKF weights by covariance), but fusing the same absolute pose dimension from two sources causes the filter to double-count information, leading to overconfident and potentially oscillating estimates.

### Example Violation

```yaml
# BAD: Double-fusing absolute yaw
odom0: "wheel_odom"
odom0_config: [false, false, false,
               false, false, true,    # Absolute yaw from wheel odometry
               true,  false, false,
               false, false, true,
               false, false, false]

imu0: "imu/data"
imu0_config: [false, false, false,
              false, false, true,     # Absolute yaw from IMU — CONFLICT
              false, false, false,
              false, false, true,
              true,  false, false]
```

**Fix**: Remove absolute yaw from wheel odometry (it's poor quality anyway). Let the IMU be the sole absolute yaw source:

```yaml
odom0_config: [false, false, false,
               false, false, false,   # No absolute yaw from wheel odom
               true,  false, false,
               false, false, true,    # vyaw is fine — it's a velocity, not absolute
               false, false, false]
```

---

## Diagnosing Double-Fusion

| Symptom | Likely Cause |
|---------|-------------|
| EKF output oscillates at high frequency | Two absolute sources fighting for the same dimension |
| State estimate is overconfident (very low covariance) but inaccurate | Information double-counted, covariance artificially shrunk |
| Adding a new sensor makes the estimate WORSE | New sensor overlaps with existing source dimensions |
| State diverges slowly then snaps back | Absolute sources drifting apart, periodic correction |

**Debug approach**: Disable sensors one at a time. If removing a sensor improves accuracy, it was conflicting with another source.

---

## The Two-EKF Pattern

For complex robots, use two separate EKF instances:

### Local EKF (odom frame)

Fuses high-rate, low-latency sensors for smooth motion estimation:
- Wheel odometry (vx, vyaw)
- IMU (yaw, vyaw, ax)
- Optionally: lidar odom, visual odom (differential mode)

```yaml
ekf_local:
  ros__parameters:
    world_frame: "odom"
    publish_tf: true            # Publishes odom → base_link
    frequency: 50.0

    odom0: "odom/unfiltered"
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  false, false,
                   false, false, true,
                   false, false, false]

    imu0: "imu/data"
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  true,  false, false]
    imu0_remove_gravitational_acceleration: true
```

### Global EKF or AMCL (map frame)

Corrects accumulated drift using global references:
- AMCL publishes map → odom directly (most common)
- OR: a second EKF in map frame fuses AMCL pose + local EKF output

The typical Nav2 setup uses AMCL for the map → odom correction, **not** a second EKF. The two-EKF pattern is mainly for outdoor robots with GPS, where the global EKF fuses GPS + local EKF output.

---

## Complete Multi-Source Configuration

Four sensors: wheel odometry, IMU, visual odometry, lidar odometry.

```yaml
ekf_filter_node:
  ros__parameters:
    use_sim_time: false
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: true
    publish_tf: true
    predict_to_current_time: true

    map_frame: "map"
    odom_frame: "odom"
    base_link_frame: "base_link"
    world_frame: "odom"

    # ─── Source 0: Wheel Odometry ─────────────────────────────
    # Primary motion source. Fuse forward velocity and yaw rate.
    # These are the most reliable dimensions from a diff-drive.
    odom0: "odom/unfiltered"
    odom0_config: [false, false, false,    # No position (EKF integrates internally)
                   false, false, false,    # No orientation from wheel odom
                   true,  false, false,    # vx — YES
                   false, false, true,     # vyaw — YES
                   false, false, false]    # No acceleration from wheel odom
    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 10

    # ─── Source 1: IMU ────────────────────────────────────────
    # Absolute yaw heading, angular velocity, forward acceleration.
    # The IMU is the authority on absolute yaw orientation.
    imu0: "imu/data"
    imu0_config: [false, false, false,    # No position from IMU
                  false, false, true,     # yaw — YES (absolute heading)
                  false, false, false,    # No velocity from IMU
                  false, false, true,     # vyaw — YES (angular velocity)
                  true,  false, false]    # ax — YES (forward acceleration)
    imu0_differential: false
    imu0_relative: false
    imu0_queue_size: 10
    imu0_remove_gravitational_acceleration: true

    # ─── Source 2: Visual Odometry ────────────────────────────
    # Fuse x, y, yaw as DIFFERENTIAL (incremental motion only).
    # VO pose drifts absolutely, but frame-to-frame delta is good.
    # Do NOT fuse vyaw — already covered by wheel odom and IMU.
    odom1: "vo/odometry"
    odom1_config: [true,  true,  false,   # x, y as differential
                   false, false, true,    # yaw as differential
                   false, false, false,   # No velocity (use pose differential instead)
                   false, false, false,   # No angular velocity (avoid triple-counting vyaw)
                   false, false, false]
    odom1_differential: true              # CRITICAL: converts absolute to incremental
    odom1_relative: false
    odom1_queue_size: 5
    odom1_rejection_threshold: 2.0        # Reject outlier measurements (Mahalanobis distance)

    # ─── Source 3: Lidar Odometry ─────────────────────────────
    # Fuse x, y as DIFFERENTIAL only. Do NOT fuse yaw or vyaw
    # (already well-covered by IMU + wheel odom).
    odom2: "odom_rf2o"
    odom2_config: [true,  true,  false,   # x, y as differential
                   false, false, false,   # No yaw (would triple-count with IMU + VO)
                   false, false, false,
                   false, false, false,   # No vyaw (would quadruple-count!)
                   false, false, false]
    odom2_differential: true
    odom2_relative: false
    odom2_queue_size: 5
    odom2_rejection_threshold: 2.0

    # ─── Process Noise ────────────────────────────────────────
    # Increase values for dimensions where the robot accelerates aggressively.
    # These values assume a modest indoor robot.
    process_noise_covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.015]
```

---

## Dimension Fusion Map

Summary of which source provides which dimensions:

| Dimension | Wheel Odom | IMU | Visual Odom | Lidar Odom |
|-----------|-----------|-----|-------------|------------|
| x (abs) | — | — | differential | differential |
| y (abs) | — | — | differential | differential |
| yaw (abs) | — | **YES** | differential | — |
| vx | **YES** | — | — | — |
| vyaw | **YES** | **YES** | — | — |
| ax | — | **YES** | — | — |

Note that `vyaw` is fused from **two** sources (wheel odom + IMU). This is acceptable — they are independent measurements of the same quantity. The EKF weights each by its reported covariance. This is different from fusing the same absolute yaw from two sources, which would double-count.

---

## Covariance Tuning

If one sensor is much noisier than another, increase its covariance so the EKF trusts it less:

```python
# In your odometry publisher:
msg.twist.covariance[0] = 0.1    # vx variance — high if wheels slip
msg.twist.covariance[35] = 0.05  # vyaw variance

# In your IMU driver or filter:
msg.angular_velocity_covariance[8] = 0.001  # vyaw from IMU — usually very precise
```

The EKF uses a Kalman gain derived from the ratio of process noise to measurement noise. If sensor covariance is very low, the EKF trusts that sensor strongly. If it is high, the EKF relies more on its internal prediction.

### Process Noise (Q Matrix)

Increase process noise if:
- Robot accelerates aggressively (state changes faster than model predicts)
- Environment causes frequent velocity changes (bumps, ramps)
- You want the filter to be more responsive to sensor updates

Decrease process noise if:
- Robot moves slowly and smoothly
- You want a smoother output trajectory

---

## Rejection Thresholds

The `rejection_threshold` parameter uses the Mahalanobis distance to detect and reject outlier measurements:

```yaml
odom1_rejection_threshold: 2.0
odom2_rejection_threshold: 2.0
```

A measurement is rejected if its Mahalanobis distance from the current state estimate exceeds this threshold. Start with 2.0 (roughly 2 standard deviations) and increase if legitimate measurements are being rejected.

Check diagnostic output for rejected measurements:

```bash
ros2 topic echo /diagnostics
```

---

## Incremental Integration Strategy

When adding sensors to an existing setup:

1. **Start with wheel odom + IMU alone** — validate the base estimate works
2. **Add visual odom** — use differential mode, verify the estimate improves (compare with ground truth if available)
3. **Add lidar odom** — same approach, differential mode, verify improvement
4. **At each step**: record a bag, compare filtered output with and without the new sensor
5. **If accuracy decreases**: the new sensor is likely overlapping with existing sources — remove conflicting dimensions

```bash
# Record for comparison
ros2 bag record /odometry/filtered /odom/unfiltered /vo/odometry /odom_rf2o /imu/data -o multi_sensor_test
```
