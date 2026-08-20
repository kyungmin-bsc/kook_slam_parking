<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: anomaly-detection-vision
description: 'Implement visual anomaly detection. Use when detecting changes in a scene, monitoring for unusual objects, or security surveillance.'
---

# Visual Anomaly Detection for Patrol Robots

## Concept

A patrol robot visits the same waypoints repeatedly. By comparing the current camera view to a stored "known good" baseline, the robot can detect environmental changes: moved furniture, open doors that should be closed, new objects, missing objects, or intruders.

## Approaches

### 1. Pixel-Level Difference

Compute the absolute difference between the current image and the reference image, pixel by pixel.

```python
import cv2
import numpy as np

def pixel_difference(current, reference, threshold=30):
    diff = cv2.absdiff(current, reference)
    gray_diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_diff, threshold, 255, cv2.THRESH_BINARY)
    change_ratio = np.count_nonzero(mask) / mask.size
    return change_ratio, mask
```

**Pros**: Simple, fast, no dependencies.
**Cons**: Extremely sensitive to lighting changes, camera position drift, and lens vignetting. Practical only in controlled-lighting environments.

### 2. Structural Similarity (SSIM)

SSIM compares luminance, contrast, and structure between images. Much more robust to uniform brightness changes than pixel difference.

```python
from skimage.metrics import structural_similarity as ssim

def compute_ssim(current_gray, reference_gray):
    score, diff_image = ssim(reference_gray, current_gray, full=True)
    # score: 1.0 = identical, 0.0 = completely different
    # diff_image: per-pixel similarity map
    return score, diff_image
```

For anomaly detection, flag when `score < threshold` (e.g., 0.85). The `diff_image` highlights which regions changed.

### 3. Feature-Based Comparison

Extract keypoints and descriptors (ORB, SIFT) from both images, match features, and quantify how many features are consistent vs. inconsistent.

```python
import cv2

def feature_comparison(current_gray, reference_gray, min_match_ratio=0.3):
    orb = cv2.ORB_create(nfeatures=500)
    kp1, des1 = orb.detectAndCompute(reference_gray, None)
    kp2, des2 = orb.detectAndCompute(current_gray, None)

    if des1 is None or des2 is None:
        return 0.0

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    good = [m for m in matches if m.distance < 50]

    match_ratio = len(good) / max(len(kp1), 1)
    return match_ratio  # high = similar, low = changed
```

**Pros**: Robust to moderate viewpoint changes and lighting shifts.
**Cons**: Fails if the scene has few texture features (blank walls). ORB is fast; SIFT is more accurate but slower.

### 4. Learned Embeddings

Use a pretrained CNN (ResNet-18, EfficientNet-B0) to encode each image as a fixed-length vector. Compare vectors using cosine similarity.

```python
import torch
import torchvision.transforms as T
from torchvision.models import resnet18

class SceneEncoder:
    def __init__(self):
        self.model = resnet18(pretrained=True)
        self.model.fc = torch.nn.Identity()  # remove classification head
        self.model.eval()
        self.transform = T.Compose([
            T.ToPILImage(),
            T.Resize((224, 224)),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])

    @torch.no_grad()
    def encode(self, image_bgr):
        image_rgb = image_bgr[:, :, ::-1]  # BGR → RGB
        tensor = self.transform(image_rgb).unsqueeze(0)
        return self.model(tensor).squeeze().numpy()

    def similarity(self, emb1, emb2):
        cos = np.dot(emb1, emb2) / (np.linalg.norm(emb1) * np.linalg.norm(emb2))
        return float(cos)
```

**Pros**: Most robust to lighting, viewpoint, and minor scene changes. Captures semantic similarity.
**Cons**: Requires a neural network on the host (or pre-compute embeddings). Heavier computation.

## Handling Lighting Variations

Lighting changes are the primary source of false positives. Mitigation strategies:

1. **Grayscale conversion**: Removes color temperature shifts
2. **Histogram equalization**: `cv2.equalizeHist()` normalizes brightness distribution
3. **CLAHE** (Contrast Limited Adaptive Histogram Equalization): Local equalization, better than global
4. **Illumination-invariant color spaces**: Convert to LAB and use only the A and B channels
5. **Time-of-day matching**: Compare against reference images captured at the same time of day

```python
def preprocess(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return clahe.apply(gray)
```

## Reference Image Management

At each patrol waypoint, capture and store a reference image:

```python
import os, json

class ReferenceStore:
    def __init__(self, store_dir='/home/robot/patrol_references'):
        self.store_dir = store_dir
        os.makedirs(store_dir, exist_ok=True)

    def save_reference(self, waypoint_id: str, image, embedding=None):
        path = os.path.join(self.store_dir, f'{waypoint_id}.png')
        cv2.imwrite(path, image)
        if embedding is not None:
            np.save(os.path.join(self.store_dir, f'{waypoint_id}.npy'), embedding)

    def load_reference(self, waypoint_id: str):
        path = os.path.join(self.store_dir, f'{waypoint_id}.png')
        if not os.path.exists(path):
            return None, None
        image = cv2.imread(path)
        emb_path = os.path.join(self.store_dir, f'{waypoint_id}.npy')
        embedding = np.load(emb_path) if os.path.exists(emb_path) else None
        return image, embedding
```

Update references periodically (e.g., weekly) or on command to account for legitimate changes.

## SSIM-Based Comparison Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from skimage.metrics import structural_similarity as ssim
import cv2

class AnomalyDetector(Node):
    def __init__(self):
        super().__init__('anomaly_detector')
        self.declare_parameter('ssim_threshold', 0.80)
        self.declare_parameter('waypoint_id', '')
        self.declare_parameter('reference_dir', '/home/robot/patrol_references')

        self.threshold = self.get_parameter('ssim_threshold').value
        self.bridge = CvBridge()
        self.reference = None
        self.store = ReferenceStore(self.get_parameter('reference_dir').value)

        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.score_pub = self.create_publisher(Float32, '/anomaly/score', 10)
        self.alert_pub = self.create_publisher(Image, '/anomaly/diff_image', 10)

    def set_waypoint(self, waypoint_id: str):
        ref_img, _ = self.store.load_reference(waypoint_id)
        if ref_img is not None:
            self.reference = cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY)
            self.reference = cv2.resize(self.reference, (320, 240))

    def image_cb(self, msg):
        if self.reference is None:
            return
        current = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        current = cv2.resize(current, (320, 240))

        score, diff = ssim(self.reference, current, full=True)
        self.score_pub.publish(Float32(data=score))

        if score < self.threshold:
            self.get_logger().warn(f'Anomaly detected: SSIM={score:.3f}')
            diff_uint8 = (255 * (1.0 - diff)).astype('uint8')
            diff_msg = self.bridge.cv2_to_imgmsg(diff_uint8, encoding='mono8')
            self.alert_pub.publish(diff_msg)
```

## Temporal Filtering

Reduce false positives by requiring the anomaly to persist across multiple patrol visits:

```python
class TemporalAnomalyFilter:
    def __init__(self, trigger_count=2, window=5):
        self.trigger_count = trigger_count
        self.window = window
        self.history = {}  # waypoint_id -> deque of bools

    def update(self, waypoint_id: str, is_anomaly: bool) -> bool:
        import collections
        if waypoint_id not in self.history:
            self.history[waypoint_id] = collections.deque(maxlen=self.window)
        self.history[waypoint_id].append(is_anomaly)
        return sum(self.history[waypoint_id]) >= self.trigger_count
```

## Behavior Tree Integration

```xml
<Sequence>
  <Action ID="NavigateToWaypoint" waypoint="{current_waypoint}"/>
  <Action ID="SetAnomalyWaypoint" waypoint="{current_waypoint}"/>
  <Action ID="WaitForStable" duration="1.0"/>
  <Condition ID="AnomalyDetected" topic="/anomaly/score" threshold="0.80"/>
  <Action ID="CaptureSnapshot"/>
  <Action ID="SendAnomalyAlert" waypoint="{current_waypoint}" score="{anomaly_score}"/>
</Sequence>
```

The `AnomalyDetected` condition reads the SSIM score from the topic and returns SUCCESS if the score is below the threshold (indicating a change). The `WaitForStable` action pauses briefly after arriving at the waypoint to let vibration settle before capturing.
