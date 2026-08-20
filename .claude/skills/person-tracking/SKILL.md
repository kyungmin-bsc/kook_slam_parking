<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: person-tracking
description: 'Track people in ROS 2. Use when following detected persons, maintaining track IDs, or estimating person positions in 3D.'
---

# Person Tracking

## Detection Sources

For a patrol robot, person detection can come from:

- **YOLO (YOLOv8n/v11n)**: General-purpose; detects class `person` (COCO class 0). Best accuracy, needs GPU or on-device VPU.
- **MobileNet-SSD**: Lighter model, runs on CPU at ~15 FPS. Lower accuracy but sufficient for indoor detection within 5m.
- **OAK-D Built-in Person Detection**: Uses `MobileNetSpatialDetectionNetwork` on the Myriad X. Returns 3D position directly with no CPU inference. Best for RPi-class hardware.

## Tracking: Assigning IDs Across Frames

Raw detections have no identity — each frame produces a new list of bounding boxes. A tracker maintains identity across frames.

### Nearest-Neighbor Matching

Simplest approach: match each new detection to the closest existing track using Euclidean distance on 3D position (or 2D image center).

```python
import numpy as np
from scipy.optimize import linear_sum_assignment

def match_detections_to_tracks(detections, tracks, max_distance=1.0):
    """Hungarian algorithm matching on 3D position distance."""
    if not detections or not tracks:
        return [], list(range(len(detections))), list(range(len(tracks)))

    cost = np.zeros((len(detections), len(tracks)))
    for i, det in enumerate(detections):
        for j, trk in enumerate(tracks):
            cost[i, j] = np.linalg.norm(
                np.array(det.position) - np.array(trk.predicted_position)
            )

    row_idx, col_idx = linear_sum_assignment(cost)
    matches, unmatched_dets, unmatched_trks = [], [], []

    for i, j in zip(row_idx, col_idx):
        if cost[i, j] > max_distance:
            unmatched_dets.append(i)
            unmatched_trks.append(j)
        else:
            matches.append((i, j))

    unmatched_dets += [i for i in range(len(detections)) if i not in row_idx]
    unmatched_trks += [j for j in range(len(tracks)) if j not in col_idx]

    return matches, unmatched_dets, unmatched_trks
```

### SORT Tracker

Simple Online and Realtime Tracking. Uses a Kalman filter to predict each track's next position, then matches predictions to detections via IoU (2D) or Euclidean distance (3D).

Track lifecycle:
1. New detection with no match → create new track (tentative)
2. Tentative track matched for N consecutive frames → promote to confirmed
3. Confirmed track unmatched for M frames → delete

### DeepSORT

Extends SORT with appearance features (a small CNN extracts a 128-d embedding per bounding box). Matching uses a weighted combination of motion distance (Kalman) and appearance distance (cosine similarity). Much better at re-identifying persons after occlusion.

## Spatial Tracking (3D)

Using depth data, track persons in world coordinates (`map` frame) rather than image coordinates:

```python
from dataclasses import dataclass, field
from geometry_msgs.msg import Point
import time

@dataclass
class TrackedPerson:
    person_id: int
    position: Point           # in map frame
    velocity: Point           # m/s in map frame
    last_seen: float          # timestamp
    hit_count: int = 0
    miss_count: int = 0
    confirmed: bool = False

    def update(self, new_position: Point, dt: float):
        if dt > 0:
            self.velocity.x = (new_position.x - self.position.x) / dt
            self.velocity.y = (new_position.y - self.position.y) / dt
        self.position = new_position
        self.last_seen = time.time()
        self.hit_count += 1
        self.miss_count = 0
        if self.hit_count >= 3:
            self.confirmed = True

    def predict(self, dt: float) -> Point:
        return Point(
            x=self.position.x + self.velocity.x * dt,
            y=self.position.y + self.velocity.y * dt,
            z=0.0,
        )
```

## Custom Tracked Person Message

```
# msg/TrackedPerson.msg
uint32 person_id
geometry_msgs/Point position      # map frame
geometry_msgs/Vector3 velocity    # m/s
float64 confidence
float64 last_seen                 # epoch timestamp
bool is_confirmed

# msg/TrackedPersonArray.msg
std_msgs/Header header
TrackedPerson[] persons
```

## Tracking Node Skeleton

```python
class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')
        self.declare_parameter('max_match_distance', 1.0)
        self.declare_parameter('max_coast_time', 3.0)  # seconds before deleting lost track
        self.declare_parameter('confirm_hits', 3)

        self.tracks: list[TrackedPerson] = []
        self.next_id = 0

        self.sub = self.create_subscription(
            Detection2DArray, '/detections', self.detection_cb, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_cb, 10)
        self.pub = self.create_publisher(TrackedPersonArray, '/tracked_persons', 10)

        self.timer = self.create_timer(0.1, self.publish_tracks)

    def detection_cb(self, msg):
        persons_3d = []
        for det in msg.detections:
            if det.results[0].hypothesis.class_id != 'person':
                continue
            pos_3d = self.project_to_3d(det)  # uses depth + TF
            if pos_3d:
                persons_3d.append(pos_3d)

        matches, new_dets, lost_trks = match_detections_to_tracks(
            persons_3d, self.tracks, self.max_match_distance)

        now = self.get_clock().now().nanoseconds / 1e9
        for det_i, trk_i in matches:
            dt = now - self.tracks[trk_i].last_seen
            self.tracks[trk_i].update(persons_3d[det_i], dt)

        for det_i in new_dets:
            self.tracks.append(TrackedPerson(
                person_id=self.next_id, position=persons_3d[det_i],
                velocity=Point(), last_seen=now))
            self.next_id += 1

        for trk_i in lost_trks:
            self.tracks[trk_i].miss_count += 1

        # Prune tracks lost longer than max_coast_time
        self.tracks = [t for t in self.tracks
                       if (now - t.last_seen) < self.max_coast_time]
```

## Patrol Integration

**Alert on unknown person**: When a confirmed track exists and the robot is in patrol mode, publish an alert. Optionally capture a snapshot image.

**Ignore known household members**: Face recognition (e.g., dlib face embeddings) can classify detected persons. Alternatively, use a time schedule — persons detected during expected hours are ignored; detections outside the schedule trigger alerts.

**Behavior Tree integration**:

```xml
<ReactiveSequence>
  <Condition ID="ConfirmedPersonNearby" topic="/tracked_persons" max_distance="5.0"/>
  <Action ID="ApproachPerson" target="{person_position}"/>
  <Action ID="CaptureSnapshot"/>
  <Action ID="SendNotification" message="Person detected"/>
</ReactiveSequence>
```

## Privacy Considerations

- Process images and detections on-device; do not stream raw camera feeds off-robot unless explicitly enabled
- Store snapshots with encryption, auto-delete after a configurable retention period
- Provide a hardware kill switch for the camera
- Face recognition embeddings are one-way (cannot reconstruct the face) — store embeddings, not face images
- Comply with local privacy laws regarding in-home cameras and recording
