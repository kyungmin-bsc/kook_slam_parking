<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: spatial-ai-depthai
description: 'Use DepthAI/OAK-D spatial AI cameras. Use when running on-device neural inference, stereo depth, or spatial object detection.'
---

# Spatial AI with DepthAI and OAK-D

## Hardware Overview

OAK-D cameras combine a stereo depth system with an Intel Myriad X Vision Processing Unit (VPU) capable of 4 TOPS of neural inference. This enables on-device object detection, segmentation, and pose estimation with zero CPU load on the host.

Key models:
- **OAK-D**: Stereo + RGB, USB connection, standard baseline
- **OAK-D Lite**: Smaller form factor, fixed focus, lower cost
- **OAK-D Pro**: Adds IR dot projector for active stereo in low light
- **OAK-D S2**: Wide-angle stereo, ideal for robotics

## DepthAI Pipeline Architecture

DepthAI programs are directed acyclic graphs (DAGs) of processing nodes that execute entirely on the VPU. The host only receives output data.

```
ColorCamera ──→ YoloDetectionNetwork ──→ XLinkOut (detections)
     │                    │
     └─── preview ────────┘
MonoLeft ──┐
           ├──→ StereoDepth ──→ SpatialDetectionNetwork ──→ XLinkOut (spatial detections)
MonoRight ─┘          │
                      └──→ XLinkOut (depth)
```

### Node Types

| Node | Purpose |
|---|---|
| `ColorCamera` | RGB sensor, configurable resolution/FPS |
| `MonoCamera` | Grayscale sensor (left/right for stereo) |
| `StereoDepth` | Computes disparity/depth from stereo pair |
| `MobileNetDetectionNetwork` | MobileNet-SSD inference on-device |
| `YoloDetectionNetwork` | YOLO inference on-device |
| `SpatialDetectionNetwork` | 2D detection + depth → 3D positions |
| `NeuralNetwork` | Generic neural network inference |
| `ImageManip` | Resize, crop, color convert on-device |
| `XLinkOut` | Send data from VPU to host |
| `XLinkIn` | Send data from host to VPU |

## depthai-ros Package

The `depthai_ros_driver` package provides a ROS 2 component node that configures and runs a DepthAI pipeline, publishing standard ROS topics.

### Installation

```bash
sudo apt install ros-jazzy-depthai-ros
```

### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/oakd/rgb/image` | `sensor_msgs/Image` | Color camera image |
| `/oakd/rgb/camera_info` | `sensor_msgs/CameraInfo` | Color camera intrinsics |
| `/oakd/stereo/depth` | `sensor_msgs/Image` | Depth image (16UC1, mm) |
| `/oakd/stereo/camera_info` | `sensor_msgs/CameraInfo` | Depth camera intrinsics |
| `/oakd/stereo/points` | `sensor_msgs/PointCloud2` | 3D point cloud |
| `/oakd/nn/detections` | `vision_msgs/Detection2DArray` | 2D detections (if NN enabled) |
| `/oakd/nn/spatial_detections` | `vision_msgs/Detection3DArray` | 3D spatial detections |

## Converting Models to .blob

DepthAI requires models in OpenVINO `.blob` format. Conversion pipeline:

```
PyTorch (.pt) ──→ ONNX (.onnx) ──→ OpenVINO IR (.xml/.bin) ──→ blob
```

### Using blobconverter

```bash
pip install blobconverter

python3 -c "
import blobconverter
blob_path = blobconverter.from_onnx(
    model='yolov8n.onnx',
    data_type='FP16',
    shaves=6,             # Myriad X SHAVE cores (6 is default, max 16)
    version='2022.1',     # OpenVINO version
    output_dir='./models',
)
print(f'Model blob: {blob_path}')
"
```

More SHAVE cores = faster inference but less available for other pipeline nodes. For a stereo + YOLO pipeline, 6 shaves for YOLO is a good balance.

### Using the OpenVINO Toolkit Directly

```bash
# ONNX → OpenVINO IR
mo --input_model yolov8n.onnx --data_type FP16

# IR → blob (using compile_tool from OpenVINO)
compile_tool -d MYRIAD -m yolov8n.xml -o yolov8n.blob
```

## SpatialDetectionNetwork

The key feature for robotics: combines 2D bounding box detection with depth lookup to produce 3D object positions without any host computation.

For each detected bounding box, the node:
1. Computes the average depth within the central region of the bounding box
2. Back-projects to 3D using the stereo calibration
3. Outputs `(x, y, z)` position in the camera frame

Output includes confidence, class label, bounding box, and spatial coordinates.

## Launch Configuration: Stereo + YOLO Spatial Detection

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthai_ros_driver',
            executable='camera.launch.py',
            name='oakd',
            parameters=[{
                'camera.i_nn_type': 'spatial',
                'camera.i_pipeline_type': 'RGBD',

                # RGB camera
                'rgb.i_resolution': '1080P',
                'rgb.i_fps': 30,
                'rgb.i_publish_topic': True,

                # Stereo depth
                'stereo.i_depth_preset': 'HIGH_ACCURACY',
                'stereo.i_output_depth': True,
                'stereo.i_publish_topic': True,
                'stereo.i_max_depth': 10000,        # mm

                # Neural network
                'nn.i_nn_config_path': '/opt/models/yolov8n_coco.json',
                'nn.i_enable_passthrough': True,

                # Spatial detection
                'spatial.i_lower_threshold': 200,     # mm — min depth
                'spatial.i_upper_threshold': 8000,    # mm — max depth
            }],
        ),
    ])
```

### Neural Network Config JSON

```json
{
    "nn_config": {
        "output_format": "detection",
        "NN_family": "YOLO",
        "input_size": "416x416",
        "NN_specific_metadata": {
            "classes": 80,
            "coordinates": 4,
            "anchors": [],
            "anchor_masks": {},
            "iou_threshold": 0.5,
            "confidence_threshold": 0.5
        }
    },
    "mappings": {
        "labels": ["person", "bicycle", "car", "..."]
    }
}
```

## Performance Characteristics

| Pipeline | FPS | CPU Load | Notes |
|---|---|---|---|
| RGB + Stereo only | 30 | ~5% | Depth compute on VPU |
| RGB + Stereo + MobileNet-SSD | 30 | ~5% | Detection on VPU |
| RGB + Stereo + YOLOv8n (416) | 15-20 | ~5% | Heavier model, still on VPU |
| RGB + Stereo + YOLOv8n (320) | 25-30 | ~5% | Smaller input = faster |

Host CPU load is dominated by data deserialization and ROS publishing, not inference.

## Integration with Nav2

Spatial detections provide obstacle positions in the camera frame. Transform to `map` frame and inject into a costmap layer, or use the depth image/point cloud as a standard observation source:

```yaml
observation_sources: lidar oakd_depth
oakd_depth:
  topic: /oakd/stereo/depth
  data_type: PointCloud2
  marking: true
  clearing: true
  min_obstacle_height: 0.1
  max_obstacle_height: 1.5
```

For detection-specific costmap integration (e.g., marking only detected persons as obstacles), implement a custom costmap layer that subscribes to `/oakd/nn/spatial_detections`.
