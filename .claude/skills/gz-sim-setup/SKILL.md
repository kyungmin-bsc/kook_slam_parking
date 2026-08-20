<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: gz-sim-setup
description: 'Set up Gazebo Harmonic simulation. Use when installing ros-jazzy-ros-gz, creating SDF worlds, or launching Gazebo with ROS 2.'
---

# Gazebo Harmonic Setup for ROS 2 Jazzy

## Gazebo Classic vs New Gazebo

Gazebo Classic (`gazebo11`) and new Gazebo (Harmonic/Garden) are **completely different codebases**. Classic used `gazebo_ros_pkgs`; new Gazebo uses the `ros_gz` integration packages. The new Gazebo (formerly Ignition) was rewritten from scratch with a modular architecture: `gz-sim` (simulator core), `gz-physics` (physics abstraction), `gz-rendering` (rendering engine), `gz-transport` (message passing), `gz-sensors` (sensor simulation), `gz-gui` (Qt-based GUI), and `gz-math`/`gz-common` (utilities). Do **not** mix Classic and Harmonic packages—they conflict.

## Installation

```bash
# ROS 2 Jazzy ships with Gazebo Harmonic bindings
sudo apt install ros-jazzy-ros-gz

# This pulls in: ros-jazzy-ros-gz-sim, ros-jazzy-ros-gz-bridge,
# ros-jazzy-ros-gz-image, ros-jazzy-ros-gz-sim-demos,
# plus gz-harmonic (gz-sim8, gz-physics7, gz-rendering8, etc.)
```

Verify installation:

```bash
gz sim --version  # Should print "Gazebo Sim, version 8.x.x"
gz sim --versions  # Lists all gz library versions
```

## Resource Paths

Gazebo resolves model URIs (`model://my_robot`) and mesh files via environment variables:

```bash
# Add custom model directories
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/my_models:/path/to/my_worlds

# Fuel model database (automatic download from https://app.gazebosim.org)
# Models are cached in ~/.gz/fuel/
```

## Minimal World SDF

SDF (Simulation Description Format) is the native world format—**not URDF**. URDF describes robots only; SDF describes entire worlds including physics, lighting, and multiple models.

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="my_robot_world">
    <!-- Physics engine: DART (default), bullet, tpe (TPE = trivial physics engine) -->
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <dart>
        <collision_detector>fcl</collision_detector>
        <solver>
          <solver_type>dantzig</solver_type>
        </solver>
      </dart>
    </physics>

    <!-- Required plugins for basic sim functionality -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Launch File

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world_file = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'), 'worlds', 'house.sdf'
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_file],  # -r = run immediately (not paused)
            'on_exit_shutdown': 'true',
        }.items(),
    )

    return LaunchDescription([
        gz_sim,
    ])
```

Launch flags: `-r` runs immediately (default is paused), `-v 4` sets verbosity, `--headless-rendering` for CI, `-s` runs server only (no GUI).

## Command-Line Tools

```bash
gz sim -r my_world.sdf          # Launch world
gz topic -l                      # List gz transport topics
gz topic -e -t /world/my_world/clock  # Echo clock topic
gz service -l                    # List gz services
gz model -m my_robot -p          # Print model pose
gz sim -s --iterations 1000      # Run 1000 steps headless then exit
```

## GUI Plugins

The GUI is configured via `<gui>` in the SDF or via `~/.gz/sim/8/gui.config`. Useful plugins: `EntityTree`, `TransformControl`, `ViewAngle`, `TopicViewer`, `Plotting`. Add to world SDF:

```xml
<gui fullscreen="0">
  <plugin filename="GzScene3D" name="3D View"/>
  <plugin filename="EntityTree" name="Entity Tree"/>
  <plugin filename="TransformControl" name="Transform Control"/>
</gui>
```

## Physics Engines

| Engine | Strengths | Use Case |
|--------|-----------|----------|
| DART (default) | Accurate contacts, constraint solver | General robotics |
| Bullet | Fast broadphase, good for many objects | Large environments |
| TPE | No dynamics, kinematic only | Sensor testing, fast iteration |
