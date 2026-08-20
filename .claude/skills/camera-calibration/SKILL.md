<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: camera-calibration
description: 'Calibrate cameras for ROS 2. Use when computing intrinsics, generating calibration YAML, or stereo calibration.'
---

# Camera Calibration

## Why Calibrate

Camera lenses introduce distortion (barrel, pincushion) and the intrinsic parameters (focal length, principal point) vary per unit. Without calibration:
- 3D reconstruction from depth is inaccurate
- Costmap projections are misaligned
- Object localization has systematic bias
- Stereo depth computation fails entirely

Recalibrate whenever the lens, mounting, or focus changes.

## The Calibration Target

A **checkerboard** pattern is standard. The `--size` argument to the calibrator specifies **inner corner count**, not squares:

```
--size 8x6  →  9 columns × 7 rows of squares, 8×6 inner corners
```

`--square 0.025` means each square side is 25mm. Measure precisely — this sets the metric scale for all calibration.

Print the checkerboard on rigid, flat material (foamboard or aluminum composite). Inkjet prints on paper curl and invalidate calibration.

## Monocular Calibration

### Run the Calibrator

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  --ros-args \
  --remap image:=/camera/image_raw \
  --remap camera:=/camera
```

### The Calibration GUI

The GUI shows four progress bars:
- **X**: Move the board left-right
- **Y**: Move the board up-down
- **Size**: Move the board closer and farther
- **Skew**: Tilt the board at various angles

Collect at least 30-40 images covering all four axes until all bars are green. Click **CALIBRATE** and wait. Then **SAVE** to write `ost.yaml`, and **COMMIT** to write it to the camera driver's calibration URL.

### Tips for Good Calibration

- Move the board slowly to avoid motion blur
- Cover the entire image, including corners and edges
- Include images with the board at ~45° tilt in both axes
- Ensure consistent, diffuse lighting (no reflections on the board)
- Hold the board still for each capture

## Output: camera_info.yaml

```yaml
image_width: 640
image_height: 480
camera_name: camera
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [fx', 0, cx', 0, 0, fy', cy', 0, 0, 0, 1, 0]
```

**Camera matrix** (`K`): `fx`, `fy` = focal lengths in pixels. `cx`, `cy` = principal point (usually near image center).

**Distortion coefficients**: `k1`, `k2`, `k3` = radial distortion. `p1`, `p2` = tangential distortion. For low-distortion lenses, values are small (~0.01-0.1).

**Reprojection error**: Reported after calibration. Below 0.3 pixels is excellent, 0.5 is acceptable, above 1.0 suggests a problem.

## Using the Calibration File

### Camera Driver Parameter

Most ROS 2 camera drivers accept `camera_info_url`:

```yaml
camera_node:
  ros__parameters:
    camera_info_url: "file:///home/robot/.ros/camera_info/camera.yaml"
```

The driver publishes `sensor_msgs/msg/CameraInfo` alongside each image on the `camera_info` topic. Downstream nodes (`depth_image_proc`, `image_proc`, stereo processing) consume this for undistortion and projection.

### Image Rectification

`image_proc` uses the calibration to undistort images:

```python
ComposableNode(
    package='image_proc',
    plugin='image_proc::RectifyNode',
    name='rectify',
    remappings=[
        ('image', '/camera/image_raw'),
        ('camera_info', '/camera/camera_info'),
        ('image_rect', '/camera/image_rect'),
    ],
),
```

## Stereo Calibration

For stereo cameras (including OAK-D), calibrate both cameras simultaneously to compute the **baseline** (distance between cameras) and **rectification matrices**.

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  --approximate 0.1 \
  --ros-args \
  --remap left:=/left/image_raw \
  --remap right:=/right/image_raw \
  --remap left_camera:=/left \
  --remap right_camera:=/right
```

`--approximate 0.1` allows a 100ms timestamp mismatch between left and right images (necessary for unsynchronized cameras).

Stereo calibration output includes:
- Individual camera intrinsics (left and right)
- Rotation matrix `R` between cameras
- Translation vector `T` (baseline)
- Rectification matrices `R1`, `R2`
- Projection matrices `P1`, `P2`

## The CameraInfo Message

Published alongside every image frame:

```
sensor_msgs/msg/CameraInfo:
  header:              # same stamp as the image
  height: 480
  width: 640
  distortion_model: "plumb_bob"
  d: [k1, k2, p1, p2, k3]
  k: [fx, 0, cx, 0, fy, cy, 0, 0, 1]      # 3x3 intrinsic
  r: [1, 0, 0, 0, 1, 0, 0, 0, 1]           # 3x3 rectification
  p: [fx', 0, cx', 0, 0, fy', cy', 0, 0, 0, 1, 0]  # 3x4 projection
```

Any node that back-projects pixels to 3D (detection→localization, depth→pointcloud) uses `K` from this message.

## Verification

After calibration, visually verify by viewing the rectified image in RViz. Straight lines in the real world should appear straight in the rectified image. If barrel distortion remains visible, recalibrate with more board positions covering the image edges.

Test numerically:
```bash
ros2 topic echo /camera/camera_info --once
```

Confirm `d` coefficients are reasonable (absolute values typically < 0.5 for standard lenses).
