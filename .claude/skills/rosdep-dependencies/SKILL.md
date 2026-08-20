<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: rosdep-dependencies
description: 'Manage ROS 2 dependencies with rosdep. Use when installing package dependencies, creating custom rosdep keys, or resolving missing packages.'
---

# Managing Dependencies with rosdep

## What rosdep Does

rosdep resolves abstract ROS package dependency names (declared in `package.xml`) to concrete system package manager commands. It bridges the gap between `<depend>rclcpp</depend>` and `apt install ros-jazzy-rclcpp`.

## Initial Setup

```bash
# Initialize rosdep database (once per machine, requires sudo)
sudo rosdep init

# Update the local rosdep database (run as regular user, run periodically)
rosdep update
```

`rosdep init` creates `/etc/ros/rosdep/sources.list.d/20-default.list`, which points to the upstream rosdep database on GitHub. `rosdep update` downloads the database locally.

## Installing Dependencies

```bash
# Install ALL dependencies for all packages in src/
# This is the single most important rosdep command.
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

| Flag | Meaning |
|------|---------|
| `--from-paths src` | Scan all `package.xml` files under `src/` |
| `--ignore-src` | Don't try to install packages that are in your workspace (you're building them) |
| `-r` | Continue even if some keys can't be resolved |
| `-y` | Auto-confirm apt installs (no interactive prompts) |

### Variant: Specific Package

```bash
# Install deps for a single package
rosdep install --from-paths src/my_robot_bringup --ignore-src -y
```

### Variant: Dry Run

```bash
# Show what would be installed without actually installing
rosdep install --from-paths src --ignore-src --simulate
```

## How package.xml Drives rosdep

Every `<depend>`, `<build_depend>`, `<exec_depend>`, and `<test_depend>` tag in `package.xml` is a rosdep key. rosdep looks up each key in its database to find the corresponding system package.

```xml
<package format="3">
  <depend>rclcpp</depend>           <!-- → apt: ros-jazzy-rclcpp -->
  <depend>sensor_msgs</depend>      <!-- → apt: ros-jazzy-sensor-msgs -->
  <depend>tf2_ros</depend>          <!-- → apt: ros-jazzy-tf2-ros -->
  <exec_depend>python3-numpy</exec_depend>  <!-- → apt: python3-numpy -->
  <exec_depend>python3-opencv</exec_depend> <!-- → apt: python3-opencv -->
</package>
```

rosdep key resolution:
- `rclcpp` → knows this is a ROS package → `apt install ros-jazzy-rclcpp`
- `python3-numpy` → knows this is a system package → `apt install python3-numpy`

## Checking What Keys Resolve To

```bash
# What system package does a rosdep key resolve to?
rosdep resolve rclcpp
# #apt
# ros-jazzy-rclcpp

rosdep resolve python3-numpy
# #apt
# python3-numpy

# List all rosdep keys needed by packages in src/
rosdep keys --from-paths src --ignore-src

# Check which keys are unresolvable
rosdep check --from-paths src --ignore-src
```

## Custom rosdep Keys

When you have dependencies not in the standard rosdep database (e.g., private packages, uncommon libraries), define custom rules.

### Step 1: Create a Custom Rules File

`~/custom_rosdep/my_rules.yaml`:

```yaml
# Map custom rosdep keys to system packages
libroboclaw:
  ubuntu:
    focal: [libroboclaw-dev]
    jammy: [libroboclaw-dev]
    noble: [libroboclaw-dev]
  debian:
    bookworm: [libroboclaw-dev]

oak-d-driver:
  ubuntu:
    pip:
      packages: [depthai]

my_custom_msgs:
  ubuntu: [ros-jazzy-my-custom-msgs]
```

### Step 2: Register the Custom Source

Create `/etc/ros/rosdep/sources.list.d/50-custom.list`:

```
yaml file:///home/robot/custom_rosdep/my_rules.yaml
```

Or for a team (hosted on a web server or GitHub raw URL):

```
yaml https://raw.githubusercontent.com/your_org/rosdep_rules/main/my_rules.yaml
```

### Step 3: Update rosdep

```bash
rosdep update
# Now rosdep will resolve your custom keys
rosdep resolve libroboclaw
```

## Python Dependencies

There are multiple ways to handle Python dependencies:

### Via rosdep (preferred for system packages)

```xml
<!-- Maps to apt install python3-numpy -->
<exec_depend>python3-numpy</exec_depend>

<!-- Maps to apt install python3-opencv -->
<exec_depend>python3-opencv</exec_depend>

<!-- Maps to apt install python3-yaml -->
<exec_depend>python3-yaml</exec_depend>
```

### Via pip (for packages not in apt)

rosdep can resolve pip packages if the database includes pip mappings:

```yaml
# In custom rosdep rules
depthai:
  ubuntu:
    pip:
      packages: [depthai]
ultralytics:
  ubuntu:
    pip:
      packages: [ultralytics]
```

Then in `package.xml`:

```xml
<exec_depend>depthai</exec_depend>
<exec_depend>ultralytics</exec_depend>
```

## CI Pattern: rosdep in GitHub Actions

```yaml
- name: Install rosdep dependencies
  run: |
    apt-get update
    rosdep init || true
    rosdep update
    source /opt/ros/jazzy/setup.bash
    rosdep install --from-paths src --ignore-src -r -y
```

Always run `rosdep install` before `colcon build` in CI to ensure all system dependencies are present.

## Troubleshooting

### Key not found

```
ERROR: the following rosdep keys are not resolved: [my_unknown_pkg]
```

Options:
1. Add it to a custom rosdep rules file
2. Install it manually and use `-r` flag to skip it
3. Check spelling—rosdep keys use underscores, not hyphens for ROS packages

### Version conflicts

```bash
# Pin a specific version via apt (outside rosdep)
apt install ros-jazzy-nav2-bringup=1.3.0-1noble

# Or use a workspace overlay: clone the desired version into src/
```

### rosdep update fails

```bash
# Clear and re-fetch
rm -rf ~/.ros/rosdep
rosdep update

# If behind a proxy:
export https_proxy=http://proxy:8080
rosdep update
```

### List all installed ROS packages

```bash
# Cross-reference with rosdep
apt list --installed 2>/dev/null | grep ros-jazzy
```

## Workflow Summary

```bash
# 1. Create workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws

# 2. Clone packages
cd src && git clone ... && cd ..

# 3. Initialize rosdep (once)
sudo rosdep init
rosdep update

# 4. Install all dependencies
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# 6. Source and run
source install/setup.bash
ros2 launch ...
```
