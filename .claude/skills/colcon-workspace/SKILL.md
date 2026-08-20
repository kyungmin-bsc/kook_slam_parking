<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: colcon-workspace
description: 'Manage ROS 2 colcon workspaces. Use when building with colcon, using --symlink-install, selective builds, or workspace overlays.'
---

# Colcon Workspace Management

## Directory Structure

```
ros2_ws/                  # Workspace root
├── src/                  # Source packages (git repos, custom packages)
│   ├── my_robot/
│   ├── my_interfaces/
│   └── third_party_pkg/
├── build/                # Build artifacts (per-package subdirectories)
├── install/              # Install space (setup.bash lives here)
└── log/                  # Build/test logs
```

## Creating a Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Clone packages into src/
cd src
git clone https://github.com/your_org/your_robot.git
cd ~/ros2_ws
```

## Building

```bash
# ALWAYS use --symlink-install for development
# Python files, launch files, and config are symlinked, not copied.
# Edits take effect immediately without rebuilding.
colcon build --symlink-install

# Source the workspace AFTER every build
source install/setup.bash
```

Without `--symlink-install`, colcon copies files into `install/`. Any edit to a Python node or launch file requires a full rebuild. With symlinks, only C++ code changes need a rebuild.

## Selective Builds

```bash
# Build a single package (no dependencies)
colcon build --symlink-install --packages-select my_robot

# Build a package AND all its dependencies
colcon build --symlink-install --packages-up-to my_robot

# Skip specific packages
colcon build --symlink-install --packages-skip heavy_sim_pkg

# Build packages that depend on a changed package
colcon build --symlink-install --packages-above my_interfaces
```

`--packages-above` is invaluable after modifying an interface package—it rebuilds everything that depends on the changed messages/services.

## Clean Build

```bash
# Nuclear option: remove all generated directories
rm -rf build/ install/ log/

# Then rebuild from scratch
colcon build --symlink-install
source install/setup.bash
```

For a single package clean:

```bash
rm -rf build/my_robot install/my_robot
colcon build --symlink-install --packages-select my_robot
```

## Build Tuning

```bash
# Limit parallel workers (useful on RAM-constrained robots)
colcon build --symlink-install --parallel-workers 2

# Verbose output (see compiler commands, warnings)
colcon build --symlink-install --event-handlers console_direct+

# Pass CMake args to all packages
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Pass CMake args to a specific package
colcon build --symlink-install --packages-select my_robot \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Continue building other packages if one fails
colcon build --symlink-install --continue-on-error
```

## Workspace Overlays

ROS 2 uses a layered workspace model. Each sourced workspace overlays the previous one.

```bash
# 1. Source the underlay (system ROS 2 install)
source /opt/ros/jazzy/setup.bash

# 2. Source your overlay (your workspace)
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

The overlay takes precedence. If you build `nav2_bringup` in your workspace, your version overrides the apt-installed one.

```bash
# Verify which workspace provides a package
ros2 pkg prefix nav2_bringup
# /home/user/ros2_ws/install/nav2_bringup  ← overlay wins

# Inspect the full prefix path chain
echo $AMENT_PREFIX_PATH | tr ':' '\n'
# /home/user/ros2_ws/install/my_robot
# /home/user/ros2_ws/install/nav2_bringup
# /opt/ros/jazzy
```

## Testing

```bash
# Run all tests
colcon test --event-handlers console_direct+

# Test a single package
colcon test --packages-select my_robot

# View test results
colcon test-result --verbose
```

## Common Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Forgot `source install/setup.bash` | `ros2 run` can't find package | Source after every build |
| Building from wrong directory | Empty build, packages not found | `cd ~/ros2_ws` before building |
| Sourcing install/ from wrong workspace | Wrong package version runs | Check `echo $AMENT_PREFIX_PATH` |
| Editing Python file without `--symlink-install` | Changes don't take effect | Rebuild with `--symlink-install` |
| Stale install/ after branch switch | Mysterious build or runtime errors | Clean build: `rm -rf build/ install/ log/` |
| Missing `source /opt/ros/jazzy/setup.bash` | `colcon` command not found | Source the underlay first |

## Complete Development Workflow

```bash
# Initial setup (once)
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <your_packages>
cd ~/ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash

# Iterate: edit Python/launch files → changes are live (symlinks)
# Iterate: edit C++ code → rebuild the changed package
colcon build --symlink-install --packages-select my_robot

# Always re-source after building C++ packages
source install/setup.bash
```

## colcon Defaults File

Avoid retyping flags by creating `~/ros2_ws/colcon.meta` or using `defaults.yaml`:

```yaml
# ~/ros2_ws/colcon_defaults.yaml
build:
  symlink-install: true
  event-handlers:
    - console_direct+
  cmake-args:
    - -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

```bash
export COLCON_DEFAULTS_FILE=~/ros2_ws/colcon_defaults.yaml
colcon build  # flags applied automatically
```
