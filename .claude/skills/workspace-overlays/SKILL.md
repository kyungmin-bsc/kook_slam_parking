<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: workspace-overlays
description: 'Understand ROS 2 workspace overlays. Use when layering workspaces, overriding apt packages, or debugging AMENT_PREFIX_PATH.'
---

# ROS 2 Workspace Overlays (Underlay/Overlay Chain)

## Core Concept

ROS 2 uses a layered workspace model. Each workspace you source overlays the previous one. Packages in an overlay **take precedence** over the same package in an underlay.

```
┌─────────────────────────────┐  ← Highest priority (sourced last)
│  Application Workspace      │     ~/ros2_ws/install/
│  (your packages)            │
├─────────────────────────────┤
│  Middleware Workspace        │     ~/nav2_ws/install/  (optional)
│  (custom Nav2 fork, etc.)   │
├─────────────────────────────┤
│  Underlay (system ROS 2)    │     /opt/ros/jazzy/
│  (apt-installed packages)   │
└─────────────────────────────┘  ← Lowest priority (sourced first)
```

## The Mechanics: What setup.bash Does

Each `setup.bash` prepends paths to environment variables:

```bash
# After sourcing /opt/ros/jazzy/setup.bash:
AMENT_PREFIX_PATH=/opt/ros/jazzy
PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:...
PATH=/opt/ros/jazzy/bin:...
LD_LIBRARY_PATH=/opt/ros/jazzy/lib:...

# After ALSO sourcing ~/ros2_ws/install/setup.bash:
AMENT_PREFIX_PATH=/home/user/ros2_ws/install/my_robot:/home/user/ros2_ws/install/my_msgs:/opt/ros/jazzy
PYTHONPATH=/home/user/ros2_ws/install/my_robot/lib/python3.12/site-packages:...(underlay paths)...
```

Because overlay paths are prepended, they are searched first. Your workspace's `my_robot` is found before any apt-installed version.

## Basic Two-Layer Setup

```bash
# Step 1: Source the underlay (always first)
source /opt/ros/jazzy/setup.bash

# Step 2: Build your workspace
cd ~/ros2_ws
colcon build --symlink-install

# Step 3: Source your overlay
source ~/ros2_ws/install/setup.bash

# Verify the chain
echo $AMENT_PREFIX_PATH | tr ':' '\n'
# /home/user/ros2_ws/install/my_robot_bringup
# /home/user/ros2_ws/install/my_robot_interfaces
# /opt/ros/jazzy
```

## Three-Layer Setup: Custom Nav2 + Application

Use case: you've forked Nav2 to add custom behaviors, and your application depends on this fork.

```bash
# Layer 1: System ROS 2
source /opt/ros/jazzy/setup.bash

# Layer 2: Custom Nav2 workspace
cd ~/nav2_ws
# src/ contains your cloned & modified nav2 packages
colcon build --symlink-install
source install/setup.bash

# Layer 3: Application workspace
cd ~/ros2_ws
# src/ contains my_robot_bringup, my_robot_nav_goals, etc.
colcon build --symlink-install
source install/setup.bash

# Verify which nav2_bringup is active
ros2 pkg prefix nav2_bringup
# /home/user/nav2_ws/install/nav2_bringup  ← your fork, not apt
```

## Verifying the Overlay Chain

```bash
# Which workspace provides a specific package?
ros2 pkg prefix nav2_bringup
ros2 pkg prefix my_robot_bringup

# Full prefix path chain (ordered by priority)
echo $AMENT_PREFIX_PATH | tr ':' '\n'

# Which Python module is actually imported?
python3 -c "import nav2_bringup; print(nav2_bringup.__file__)"

# Which executable runs?
which ros2
which nav2_controller
```

## Overriding Apt-Installed Packages

To replace a system-installed ROS 2 package with a custom version:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/src

# Clone the package you want to modify
git clone https://github.com/ros-navigation/navigation2.git -b jazzy

# Or clone just one package (sparse checkout)
git clone --filter=blob:none --sparse \
  https://github.com/ros-navigation/navigation2.git
cd navigation2
git sparse-checkout set nav2_bt_navigator

# Make your changes, then build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Your modified nav2_bt_navigator now overrides the apt version
ros2 pkg prefix nav2_bt_navigator
# /home/user/ros2_ws/install/nav2_bt_navigator
```

## COLCON_PREFIX_PATH

`COLCON_PREFIX_PATH` tells colcon where to find already-built packages (underlays) during the build process. It's set automatically when you source `setup.bash`.

```bash
# After sourcing the underlay:
echo $COLCON_PREFIX_PATH
# /opt/ros/jazzy

# colcon uses this to resolve dependencies that aren't in your src/
# If a package depends on rclcpp, colcon finds it in /opt/ros/jazzy
```

## Common Pitfalls

### Wrong Source Order

```bash
# WRONG: sourcing overlay before underlay
source ~/ros2_ws/install/setup.bash   # ← fails or incomplete
source /opt/ros/jazzy/setup.bash       # ← now overlay is masked

# CORRECT: always source underlay first
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Stale Install Directory

After switching branches or making major changes:

```bash
# Problem: old artifacts in install/ conflict with new code
# Solution: clean rebuild
cd ~/ros2_ws
rm -rf build/ install/ log/
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Sourcing setup.bash in .bashrc

Convenient but can cause issues with multiple workspaces:

```bash
# ~/.bashrc — pick ONE workspace chain
source /opt/ros/jazzy/setup.bash
# Only add this if you always want this workspace active:
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
```

The `2>/dev/null || true` prevents errors if the workspace hasn't been built yet.

### Mixing Workspaces from Different ROS Distros

```bash
# NEVER do this:
source /opt/ros/jazzy/setup.bash
source ~/humble_ws/install/setup.bash  # ← ABI incompatible!
```

All overlays must target the same ROS 2 distribution.

## Isolated Workspaces via Separate Terminals

For testing, use separate terminal sessions with different overlay chains:

```bash
# Terminal 1: System Nav2
source /opt/ros/jazzy/setup.bash
ros2 launch nav2_bringup ...

# Terminal 2: Custom Nav2 overlay
source /opt/ros/jazzy/setup.bash
source ~/nav2_ws/install/setup.bash
ros2 launch nav2_bringup ...   # ← runs YOUR version
```

## Scripting the Overlay Chain

For robots with complex overlay stacks, create a setup script:

```bash
#!/bin/bash
# ~/ros2_ws/setup_robot.bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f ~/nav2_ws/install/setup.bash ]; then
  source ~/nav2_ws/install/setup.bash
  echo "Sourced custom Nav2 overlay"
fi

source ~/ros2_ws/install/setup.bash
echo "Sourced application workspace"

echo "AMENT_PREFIX_PATH:"
echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | head -5
echo "..."
```

```bash
source ~/ros2_ws/setup_robot.bash
```

## Summary

| Action | Command |
|--------|---------|
| Check which workspace provides a package | `ros2 pkg prefix <pkg>` |
| View full overlay chain | `echo $AMENT_PREFIX_PATH \| tr ':' '\n'` |
| Override an apt package | Clone into your workspace's `src/`, build, source |
| Clean a confused workspace | `rm -rf build/ install/ log/` and rebuild |
| Source order | Underlay first, then each overlay in dependency order |
