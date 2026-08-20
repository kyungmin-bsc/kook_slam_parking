<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: cross-compilation
description: 'Cross-compile ROS 2 for ARM. Use when building for Raspberry Pi, Jetson, or using Docker multi-arch with QEMU.'
---

# Cross-Compiling ROS 2 for ARM Targets

## The Challenge

Building ROS 2 on a Raspberry Pi 4 or Jetson Nano is slow (30+ minutes for a modest workspace). Cross-compilation builds on a fast x86_64 host and deploys the resulting binaries to the ARM target.

## Approaches Compared

| Approach | Speed | Complexity | Reliability |
|----------|-------|------------|-------------|
| Native build on target | 1x (baseline) | Low | High |
| Docker + QEMU emulation | 2-5x slower than native x86, but faster than native ARM | Medium | High |
| Cross-compilation toolchain | Fastest (native x86 speed) | High | Medium |
| `ros_cross_compile` tool | Near-native x86 speed | Medium | Medium |

## Approach 1: Native Build on Target (Simple, Slow)

Best for: small packages, active development on the robot.

```bash
# On the Raspberry Pi / Jetson:
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --parallel-workers 2  # limit RAM usage
```

Tip: use `--packages-select` to only rebuild what changed. Use `--symlink-install` so Python changes are instant.

## Approach 2: Docker + QEMU (Recommended for Most Users)

Docker BuildKit with QEMU transparently emulates ARM instructions on an x86 host. No toolchain setup needed—the same Dockerfile works on both architectures.

### One-Time Host Setup

```bash
# Install QEMU user-mode emulation (enables transparent ARM binary execution)
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Create a multi-architecture builder
docker buildx create --name multiarch --driver docker-container --use
docker buildx inspect --bootstrap
```

### Multi-Architecture Dockerfile

```dockerfile
# Dockerfile.robot
FROM ros:jazzy AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY src/ src/

RUN . /opt/ros/jazzy/setup.sh && \
    rosdep init || true && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- Runtime Stage ---
FROM ros:jazzy AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-nav2-bringup \
    ros-jazzy-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ros2_ws/install /ros2_ws/install
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "my_robot_bringup", "my_robot.launch.py"]
```

### Building for ARM64

```bash
# Build for ARM64 and push to registry
docker buildx build \
  --platform linux/arm64 \
  -t ghcr.io/your_org/my-robot:latest \
  -f Dockerfile.robot \
  --push \
  .

# Or build and load locally (for testing with QEMU)
docker buildx build \
  --platform linux/arm64 \
  -t my-robot:arm64 \
  -f Dockerfile.robot \
  --load \
  .
```

### Building Multi-Arch (x86 + ARM in one image)

```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -t ghcr.io/your_org/my-robot:latest \
  -f Dockerfile.robot \
  --push \
  .
```

Docker automatically selects the right architecture when pulling on the target.

### Deploying to the Robot

```bash
# On the Raspberry Pi / Jetson:
docker pull ghcr.io/your_org/my-robot:latest
docker run --network host --privileged \
  -v /dev:/dev \
  ghcr.io/your_org/my-robot:latest
```

## Approach 3: CMake Cross-Compilation Toolchain

For the fastest builds with full native x86 speed. Requires a properly configured sysroot with ARM libraries.

### Toolchain File

`cmake/aarch64_toolchain.cmake`:

```cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Cross compiler (install: apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu)
set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

# Sysroot containing ARM libraries (extracted from target or Docker image)
set(CMAKE_SYSROOT /path/to/aarch64-sysroot)
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

# Search for programs on the host, libraries/headers in the sysroot
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
```

### Building with the Toolchain

```bash
source /opt/ros/jazzy/setup.bash
colcon build \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=$(pwd)/cmake/aarch64_toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release
```

### Creating the Sysroot

Extract from a running ARM system or from a Docker image:

```bash
# Method: Extract from Docker image
docker create --name temp_sysroot ros:jazzy
docker export temp_sysroot | tar -x -C /path/to/aarch64-sysroot
docker rm temp_sysroot

# Fix symlinks (they'll be absolute, pointing to wrong paths)
cd /path/to/aarch64-sysroot
find . -type l | while read link; do
  target=$(readlink "$link")
  if [[ "$target" = /* ]]; then
    ln -sf "/path/to/aarch64-sysroot$target" "$link"
  fi
done
```

This approach is fragile. Docker + QEMU is recommended unless you have specific performance requirements.

## Approach 4: ros_cross_compile

The `ros_cross_compile` tool automates much of the toolchain setup using Docker internally.

```bash
pip install ros_cross_compile

ros_cross_compile \
  ~/ros2_ws \
  --arch aarch64 \
  --os ubuntu \
  --rosdistro jazzy \
  --colcon-defaults ~/ros2_ws/colcon_defaults.yaml
```

This creates a Docker container with the correct cross-compilation environment, builds your workspace, and outputs ARM64 binaries.

## NVIDIA Jetson Specifics

NVIDIA provides JetPack SDK with cross-compilation support:

```bash
# Use NVIDIA's L4T base image for Jetson-optimized builds
FROM nvcr.io/nvidia/l4t-ros:jazzy-ros-base-r36.4.0

# CUDA is available out of the box
RUN apt-get update && apt-get install -y \
    ros-jazzy-image-proc \
    && rm -rf /var/lib/apt/lists/*

COPY src/ /ros2_ws/src/
WORKDIR /ros2_ws
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install
```

For Jetson Orin with GPU-accelerated inference:

```dockerfile
# Make sure CUDA toolkit is available
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}
```

## CI Integration: Build ARM Images in GitHub Actions

```yaml
name: Build ARM64 Image

on:
  push:
    branches: [main]

jobs:
  build-arm:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4

      - name: Set up QEMU
        uses: docker/setup-qemu-action@v3

      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v3

      - name: Login to GHCR
        uses: docker/login-action@v3
        with:
          registry: ghcr.io
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}

      - name: Build and push
        uses: docker/build-push-action@v6
        with:
          context: .
          file: Dockerfile.robot
          platforms: linux/arm64
          push: true
          tags: ghcr.io/${{ github.repository }}:latest
          cache-from: type=gha
          cache-to: type=gha,mode=max
```

## Practical Recommendation

| Scenario | Recommended Approach |
|----------|---------------------|
| Quick iteration on robot | Native build + `--packages-select` |
| Nightly/CI builds for deployment | Docker + QEMU multi-stage |
| Large workspace, frequent rebuilds | `ros_cross_compile` or CMake toolchain |
| Jetson with CUDA dependencies | NVIDIA L4T Docker images |
| Team with mixed dev machines | Docker multi-arch images |

For the robot specifically, Docker + QEMU for building deployment images and native builds on the robot for active development strikes the best balance.
