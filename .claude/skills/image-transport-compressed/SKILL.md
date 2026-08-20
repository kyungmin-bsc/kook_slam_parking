<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: image-transport-compressed
description: 'Use ROS 2 image_transport for compressed images. Use when reducing camera bandwidth, configuring JPEG/PNG/H264 compression.'
---

# image_transport and Compressed Image Topics

## The Problem

A raw 640×480 RGB image is `640 × 480 × 3 = 921,600 bytes` per frame. At 30 FPS that is **27.6 MB/s** per camera. Over WiFi this saturates bandwidth quickly, causing frame drops and latency in all ROS 2 topics sharing the network.

## image_transport Package

`image_transport` provides transparent plugin-based compression for image publishing and subscribing. It replaces direct use of `rclcpp::Publisher<sensor_msgs::msg::Image>`.

### Available Transport Plugins

| Plugin | Encoding | Typical Compression | Use Case |
|---|---|---|---|
| `raw` | None | 1:1 | Local, same machine |
| `compressed` | JPEG or PNG | 10-20:1 (JPEG) | WiFi/network, color images |
| `compressedDepth` | 16-bit PNG (optionally with RVL) | 2-4:1 | Depth images |
| `theora` | Theora video codec | 20-50:1 | Streaming video |

### Bandwidth Comparison (640×480 @ 30 FPS)

| Transport | Per Frame | Bandwidth |
|---|---|---|
| `raw` | ~922 KB | ~27.6 MB/s |
| `compressed` (JPEG q=80) | ~40-60 KB | ~1.5 MB/s |
| `compressed` (JPEG q=50) | ~20-30 KB | ~0.8 MB/s |
| `theora` | ~10-20 KB | ~0.5 MB/s |

## Publishing with image_transport (C++)

```cpp
#include <image_transport/image_transport.hpp>

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node") {
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        pub_ = it_->advertise("camera/image", 1);
    }

    void publish_frame(const sensor_msgs::msg::Image& msg) {
        pub_.publish(msg);
        // Automatically publishes to:
        //   camera/image              (raw)
        //   camera/image/compressed   (if compressed plugin loaded)
        //   camera/image/theora       (if theora plugin loaded)
    }

private:
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher pub_;
};
```

## Subscribing with Transport Hint (C++)

```cpp
image_transport::Subscriber sub = it_->subscribe(
    "camera/image",       // base topic
    1,                    // queue size
    &MyNode::image_cb,    // callback
    this,
    "compressed"          // transport hint — request compressed
);
```

In Python (using `cv_bridge` with compressed):

```python
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

def compressed_cb(msg: CompressedImage):
    np_arr = np.frombuffer(msg.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
```

## Configuration Parameters

### JPEG Compression

Set via parameter on the publisher node:

```yaml
camera_node:
  ros__parameters:
    image_transport: "compressed"
    compressed:
      format: "jpeg"
      jpeg_quality: 80       # 0-100; 80 is good balance
      png_level: 6           # 0-9; only used if format is "png"
```

Lower `jpeg_quality` reduces bandwidth but introduces artifacts. For detection pipelines, 60-70 is usually acceptable. For human viewing or recording, 80-90.

### Depth Compression

Depth images (`16UC1` or `32FC1`) cannot use JPEG (lossy destroys depth precision). Use `compressedDepth`:

```yaml
depth_camera_node:
  ros__parameters:
    depth.image_transport: "compressedDepth"
    depth.compressedDepth:
      format: "png"           # lossless 16-bit PNG
      png_level: 3            # compression level (0=none, 9=max)
```

## Republishing Between Formats

The `image_transport` package provides a `republish` node to convert between transports:

```bash
# Decompress a compressed topic back to raw (e.g., for a node that only accepts raw)
ros2 run image_transport republish compressed raw \
  --ros-args \
  --remap in/compressed:=/camera/image/compressed \
  --remap out:=/camera/image_raw
```

Launch file version:

```python
Node(
    package='image_transport',
    executable='republish',
    name='image_republisher',
    arguments=['compressed', 'raw'],
    remappings=[
        ('in/compressed', '/camera/image/compressed'),
        ('out', '/camera/image_decompressed'),
    ],
),
```

## Topic Name Structure

When `image_transport` publishes, it creates sub-topics under the base:

```
/camera/image                  ← raw
/camera/image/compressed       ← CompressedImage
/camera/image/compressedDepth  ← CompressedImage (depth)
/camera/image/theora           ← theora packets
```

**Common issue**: Subscribing to `/camera/image/compressed` directly with a `sensor_msgs/Image` subscriber won't work. Use `image_transport::Subscriber` with the transport hint, or subscribe to the `CompressedImage` message type directly.

## Launch Configuration for Network-Optimized Setup

```python
# On the robot: publish compressed only to save bandwidth
Node(
    package='my_camera_driver',
    executable='camera_node',
    parameters=[{
        'image_transport': 'compressed',
        'compressed.jpeg_quality': 70,
    }],
),

# On the remote workstation: republish to raw for RViz / detection
Node(
    package='image_transport',
    executable='republish',
    arguments=['compressed', 'raw'],
    remappings=[
        ('in/compressed', '/camera/image/compressed'),
        ('out', '/camera/image_raw'),
    ],
),
```

## Best Practices

- Always use `image_transport` instead of raw `Image` publishers for any camera topic that may cross a network boundary
- Set `jpeg_quality` to 70 for perception pipelines and 85 for recording/display
- For depth images, always use `compressedDepth` with PNG (never JPEG)
- Install all transport plugins: `ros-jazzy-image-transport-plugins`
- Use `ros2 topic bw /camera/image/compressed` to verify actual bandwidth usage
