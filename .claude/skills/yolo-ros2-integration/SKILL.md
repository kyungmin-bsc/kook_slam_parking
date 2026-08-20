<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: yolo-ros2-integration
description: 'Integrate YOLO object detection with ROS 2. Use when running YOLOv8/v5 inference, publishing Detection2D messages, or real-time detection.'
---

# YOLO ROS 2 Integration

## Overview

YOLO (You Only Look Once) provides real-time object detection. For ROS 2 Jazzy, the main integration paths are:

1. **Direct ultralytics Python API** — simplest, run inference in a ROS 2 node
2. **yolo_ros2** — community ROS 2 wrapper with lifecycle management
3. **OAK-D on-device inference** — compile YOLO to a `.blob`, run on the Myriad X VPU at 15-30 FPS with zero CPU load

## Model Selection

| Model | Params | mAP (COCO) | Speed (CPU) | Speed (GPU) | Use Case |
|---|---|---|---|---|---|
| YOLOv8n | 3.2M | 37.3 | ~100ms | ~6ms | Embedded/real-time on CPU |
| YOLOv8s | 11.2M | 44.9 | ~200ms | ~8ms | Balanced accuracy/speed |
| YOLOv8m | 25.9M | 50.2 | ~400ms | ~12ms | Best accuracy when GPU available |
| YOLOv11n | 2.6M | 39.5 | ~80ms | ~5ms | Latest nano model |

For a patrol robot running on a Jetson or RPi with OAK-D, use `yolov8n` or `yolov11n` on-device. For a system with a CUDA GPU, `yolov8s` or `yolov8m` gives better accuracy.

## Output Messages

Standard output: `vision_msgs/msg/Detection2DArray`

```
Detection2DArray:
  header: ...
  detections[]:
    - bbox:
        center: {x: 320.0, y: 240.0}
        size_x: 85.0
        size_y: 200.0
      results[]:
        - hypothesis:
            class_id: "person"
            score: 0.92
```

## Minimal Detection Node (ultralytics)

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cpu')  # 'cpu', 'cuda:0', 'mps'

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        device = self.get_parameter('device').value

        self.model = YOLO(model_path)
        self.model.to(device)
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.pub = self.create_publisher(Detection2DArray, '/detections', 10)

    def image_cb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image, conf=self.conf_thresh, verbose=False)

        det_array = Detection2DArray()
        det_array.header = msg.header

        for result in results:
            for box in result.boxes:
                det = Detection2D()
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                det.bbox.center.position.x = (x1 + x2) / 2.0
                det.bbox.center.position.y = (y1 + y2) / 2.0
                det.bbox.size_x = x2 - x1
                det.bbox.size_y = y2 - y1

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = result.names[int(box.cls[0])]
                hyp.hypothesis.score = float(box.conf[0])
                det.results.append(hyp)
                det_array.detections.append(det)

        self.pub.publish(det_array)


def main():
    rclpy.init()
    rclpy.spin(YoloDetector())
```

## Inference Backends

**CPU**: Default. Suitable for `yolov8n` at ~10 FPS on modern x86. On ARM (RPi 5), expect 2-3 FPS.

**CUDA**: Install `ultralytics` with CUDA support. Set `device='cuda:0'`. Achieves 30+ FPS on Jetson Orin with `yolov8s`.

**OAK-D (Myriad X)**: Compile the model to OpenVINO IR then to `.blob`:

```bash
# Export to ONNX
yolo export model=yolov8n.pt format=onnx imgsz=416

# Convert to blob (use blobconverter or OpenVINO toolkit)
pip install blobconverter
python -c "
import blobconverter
blob_path = blobconverter.from_onnx(
    model='yolov8n.onnx',
    data_type='FP16',
    shaves=6,
    output_dir='.',
)
print(f'Blob: {blob_path}')
"
```

On-device inference runs the entire detection pipeline on the OAK-D's VPU. The host receives detection results only — no image transfer needed for detection. Achieves 15-30 FPS with zero CPU utilization.

## NMS and Confidence Tuning

- **Confidence threshold** (`conf`): Minimum score to keep a detection. Default 0.25; raise to 0.5+ for patrol use to reduce false positives.
- **NMS IoU threshold** (`iou`): When overlapping boxes exist, suppress those with IoU above this threshold. Default 0.7; lower to 0.45 to be more aggressive.

```python
results = self.model(frame, conf=0.5, iou=0.45, classes=[0])  # class 0 = person
```

## ONNX Export for Cross-Platform

```bash
yolo export model=yolov8n.pt format=onnx imgsz=640 simplify=True
```

ONNX models run via `onnxruntime` on CPU, CUDA, TensorRT, or OpenVINO. This decouples the model from the PyTorch dependency:

```python
import onnxruntime as ort
session = ort.InferenceSession('yolov8n.onnx', providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
```

## Launch Configuration

```python
Node(
    package='my_perception',
    executable='yolo_detector',
    name='yolo_detector',
    parameters=[{
        'model_path': '/opt/models/yolov8n.pt',
        'confidence_threshold': 0.5,
        'device': 'cpu',
    }],
    remappings=[
        ('/camera/image_raw', '/oakd/rgb/image'),
    ],
),
```
