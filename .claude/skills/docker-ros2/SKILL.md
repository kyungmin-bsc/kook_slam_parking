<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: docker-ros2
description: 'Use Docker for ROS 2. Use when building Docker images, multi-stage builds, docker-compose for robot subsystems, or GPU/device access.'
---

# Docker for ROS 2 Development and Deployment

## Base Images

| Image | Contents | Use Case |
|-------|----------|----------|
| `ros:jazzy` | ROS 2 Jazzy bare | Minimal nodes, custom builds |
| `ros:jazzy-perception` | + OpenCV, image pipeline | Vision/perception nodes |
| `ros:jazzy-desktop` | + RViz, rqt | Development/debugging |
| `ros:rolling` | Latest development | Testing against upcoming changes |

## Development Dockerfile

```dockerfile
FROM ros:jazzy

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws
COPY src/ src/

# Install rosdep dependencies
RUN . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

# Build
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# Source workspace on container start
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

## Multi-Stage Production Build

Separates build tools from runtime, shrinking the final image significantly.

```dockerfile
# === Build Stage ===
FROM ros:jazzy AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY src/ src/

RUN . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# === Runtime Stage ===
FROM ros:jazzy AS runtime

# Install only runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-nav2-bringup \
    ros-jazzy-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Copy only the install space (no build artifacts, no source)
COPY --from=builder /ros2_ws/install /ros2_ws/install

# Custom entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "my_robot_bringup", "my_robot.launch.py"]
```

**entrypoint.sh:**

```bash
#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
exec "$@"
```

## Docker Compose for Multi-Container Robots

```yaml
# docker-compose.yml
services:
  hardware:
    build:
      context: .
      dockerfile: docker/Dockerfile.hardware
    network_mode: host
    privileged: false
    devices:
      - /dev/ttyACM0:/dev/ttyACM0   # Teensy
      - /dev/ttyUSB0:/dev/ttyUSB0   # LIDAR
    volumes:
      - /dev:/dev
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    restart: unless-stopped

  navigation:
    build:
      context: .
      dockerfile: docker/Dockerfile.nav
    network_mode: host
    depends_on:
      - hardware
    environment:
      - ROS_DOMAIN_ID=0
    volumes:
      - ./maps:/ros2_ws/maps:ro
    restart: unless-stopped

  perception:
    build:
      context: .
      dockerfile: docker/Dockerfile.perception
    network_mode: host
    depends_on:
      - hardware
    environment:
      - ROS_DOMAIN_ID=0
    devices:
      - /dev/video0:/dev/video0     # USB camera
    # For OAK-D / USB3 cameras:
    volumes:
      - /dev/bus/usb:/dev/bus/usb
    restart: unless-stopped

  rviz:
    build:
      context: .
      dockerfile: docker/Dockerfile.desktop
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - ROS_DOMAIN_ID=0
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    profiles:
      - debug   # Only started with: docker compose --profile debug up
```

## GUI Support

```bash
# Allow Docker to access X11 display (Linux host)
xhost +local:docker

# Run with display forwarding
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ros:jazzy-desktop \
  rviz2
```

## GPU Access (NVIDIA)

```bash
# Requires nvidia-container-toolkit installed on host
docker run -it --rm \
  --gpus all \
  --network host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  ros:jazzy-perception
```

In Compose:

```yaml
  perception:
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
```

## Networking

```bash
# Simplest: host networking (DDS multicast works naturally)
docker run --network host ...

# If using bridge networking, configure DDS discovery:
# Set ROS_AUTOMATIC_DISCOVERY_RANGE or use a Discovery Server
# In each container:
environment:
  - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  - FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastdds.xml
```

## Hardware Device Access

```bash
# Specific device (preferred over --privileged)
docker run --device /dev/ttyACM0 ...

# For dynamically attached USB devices, mount the bus:
docker run -v /dev/bus/usb:/dev/bus/usb ...

# Match udev rules inside container:
# Copy your udev rules and rebuild, or mount them:
docker run -v /etc/udev/rules.d:/etc/udev/rules.d:ro ...
```

## Building for ARM (Raspberry Pi / Jetson)

```bash
# Setup QEMU for multi-arch builds (once per host)
docker buildx create --name multiarch --use
docker buildx inspect --bootstrap

# Build for ARM64
docker buildx build \
  --platform linux/arm64 \
  -t myrobot:latest-arm64 \
  --load \
  .
```

## Development Workflow with Bind Mounts

```bash
# Mount your source code for live editing (no rebuild needed for Python)
docker run -it --rm \
  --network host \
  -v ~/ros2_ws/src:/ros2_ws/src \
  -v ~/ros2_ws/install:/ros2_ws/install \
  ros:jazzy \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           cd /ros2_ws && \
           colcon build --symlink-install && \
           source install/setup.bash && \
           bash"
```
