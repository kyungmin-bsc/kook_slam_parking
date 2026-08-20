<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: object-detection-pipeline
description: 'Build an object detection pipeline in ROS 2. Use when processing camera images, running neural network inference, and publishing detections.'
---

# Object Detection Pipeline

## End-to-End Architecture

```
Camera ──→ Image Topic ──→ Detection Model ──→ 2D Bounding Boxes
                                                       │
Depth Image ──────────────────────────────────────→ 3D Localization
                                                       │
                                              ┌────────┴────────┐
                                              ▼                  ▼
                                      Costmap Layer      Behavior Tree
                                      (obstacles)        (condition node)
                                              │                  │
                                              ▼                  ▼
                                         Nav2 Planner      Alert / React
```

## 2D Detection to 3D Position

Given a bounding box from YOLO or another detector and a registered depth image, compute the 3D position of the detected object.

### Using Camera Intrinsics

The `sensor_msgs/msg/CameraInfo` message provides the intrinsic matrix `K`:

```
K = [fx  0  cx]
    [ 0 fy  cy]
    [ 0  0   1]
```

Where `fx`, `fy` are focal lengths in pixels and `cx`, `cy` is the principal point.

### Back-Projection

```python
import numpy as np
from cv_bridge import CvBridge

def detection_to_3d(bbox_center_x, bbox_center_y, depth_image, camera_info):
    """Convert 2D detection center + depth to 3D point in camera optical frame."""
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx = camera_info.k[2]
    cy = camera_info.k[5]

    u = int(bbox_center_x)
    v = int(bbox_center_y)

    # Sample a small patch around the center to get robust depth
    patch = depth_image[max(0, v-3):v+3, max(0, u-3):u+3]
    valid = patch[patch > 0]
    if len(valid) == 0:
        return None
    z = float(np.median(valid)) / 1000.0  # mm to meters for 16UC1

    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return (x, y, z)  # in camera_depth_optical_frame
```

Use a small patch (e.g., 7x7) around the detection center and take the median depth to handle noisy readings and object edges where depth is unreliable.

### Transform to Map Frame

Use TF2 to transform the 3D point from `camera_depth_optical_frame` to `map`:

```python
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs

point = PointStamped()
point.header = depth_msg.header
point.point.x, point.point.y, point.point.z = x, y, z

map_point = tf_buffer.transform(point, 'map', timeout=rclpy.duration.Duration(seconds=0.1))
```

## RViz Visualization

Publish detections as `visualization_msgs/msg/MarkerArray` for 3D visualization:

```python
from visualization_msgs.msg import Marker, MarkerArray

def create_detection_marker(det_id, position, class_name, confidence, frame_id, stamp):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'detections'
    marker.id = det_id
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position = position
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 1.0
    marker.color.r = 1.0
    marker.color.a = 0.7
    marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

    # Text label above the marker
    text = Marker()
    text.header = marker.header
    text.ns = 'detection_labels'
    text.id = det_id
    text.type = Marker.TEXT_VIEW_FACING
    text.action = Marker.ADD
    text.pose.position = position
    text.pose.position.z += 1.2
    text.scale.z = 0.2
    text.color.r = text.color.g = text.color.b = 1.0
    text.color.a = 1.0
    text.text = f'{class_name}: {confidence:.0%}'
    text.lifetime = marker.lifetime

    return [marker, text]
```

## Filtering Detections

### Confidence Threshold

Set per-class minimum confidence. Person detection at 0.5, other objects at 0.6+.

### Minimum Bounding Box Size

Discard detections smaller than a minimum pixel area. Very small bounding boxes are unreliable:

```python
min_area = 1500  # pixels²
area = bbox.size_x * bbox.size_y
if area < min_area:
    continue
```

### Temporal Smoothing

Require an object to be detected in N out of M consecutive frames before reporting:

```python
class TemporalFilter:
    def __init__(self, n_required=3, window=5):
        self.n_required = n_required
        self.window = window
        self.history = {}  # class_name -> deque of bools

    def update(self, class_name, detected: bool) -> bool:
        if class_name not in self.history:
            self.history[class_name] = collections.deque(maxlen=self.window)
        self.history[class_name].append(detected)
        return sum(self.history[class_name]) >= self.n_required
```

## Behavior Tree Integration

Publish detections to a topic. Create a BT condition node that subscribes:

```xml
<ReactiveSequence>
  <Condition ID="PersonDetected" topic="/detections/persons" min_confidence="0.6"/>
  <Action ID="SendAlert" message="Person detected at {detection_x}, {detection_y}"/>
</ReactiveSequence>
```

The `PersonDetected` condition node subscribes to the filtered detections topic and checks for recent person detections within a time window. It sets blackboard variables `detection_x` and `detection_y` for downstream action nodes.

```cpp
class PersonDetected : public BT::ConditionNode {
public:
    BT::NodeStatus tick() override {
        auto det = last_detection_;  // from subscription callback
        if (!det || (now() - det->header.stamp) > timeout_) {
            return BT::NodeStatus::FAILURE;
        }
        for (const auto& d : det->detections) {
            if (d.results[0].hypothesis.class_id == "person" &&
                d.results[0].hypothesis.score >= min_confidence_) {
                setOutput("detection_x", d.bbox.center.position.x);
                setOutput("detection_y", d.bbox.center.position.y);
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::FAILURE;
    }
};
```

## Performance Considerations

- Run detection at a lower rate than the camera FPS (e.g., 5 Hz detection on a 30 Hz camera) by decimating or using a timer callback
- Use `image_transport` compressed topics to reduce bandwidth if the detection node runs on a different machine
- Depth lookup is essentially free once the depth image is in memory — the expensive step is always the neural network inference
