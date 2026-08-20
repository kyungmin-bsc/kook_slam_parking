<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: urdf-fundamentals
description: 'Write URDF robot descriptions. Use when creating links, joints, visual/collision/inertial elements, or understanding the URDF tree structure.'
---

# URDF Fundamentals

## Structure

A URDF (Unified Robot Description Format) file is XML describing a robot's kinematic and dynamic properties. The root element is `<robot>`:

```xml
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="base_link">
    <!-- visual, collision, inertial -->
  </link>
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
  </joint>
  <link name="wheel_link">
    <!-- ... -->
  </link>
</robot>
```

Every robot requires at least one link (`base_link` by convention). The entire description must form a **tree**—no loops, no disconnected subgraphs. Every link except the root must have exactly one parent joint.

## Link Elements

Each `<link>` may contain three sub-elements:

| Element | Purpose | Required? |
|---|---|---|
| `<visual>` | Rendering in RViz/Gazebo | No, but strongly recommended |
| `<collision>` | Collision detection geometry | No, but needed for physics |
| `<inertial>` | Mass and moments of inertia | No, but needed for simulation |

```xml
<link name="base_link">
  <visual>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <geometry><box size="0.3 0.2 0.1"/></geometry>
    <material name="gray"><color rgba="0.5 0.5 0.5 1"/></material>
  </visual>
  <collision>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <geometry><box size="0.3 0.2 0.1"/></geometry>
  </collision>
  <inertial>
    <mass value="5.0"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <inertia ixx="0.0108" ixy="0" ixz="0" iyy="0.0417" iyz="0" izz="0.0483"/>
  </inertial>
</link>
```

## Joint Types

| Type | Motion | Use Case |
|---|---|---|
| `fixed` | None | Sensor mounts, rigid structures |
| `continuous` | Unlimited rotation | Wheels |
| `revolute` | Bounded rotation | Steering, pan/tilt |
| `prismatic` | Bounded linear slide | Linear actuators |
| `floating` | 6-DOF (rarely used) | Free-floating objects |
| `planar` | 2D translation + rotation | Planar mechanisms |

## Coordinate Conventions (REP 103)

- **x-forward**, **y-left**, **z-up** (right-hand rule)
- Units: meters, radians, kilograms
- All `<origin>` transforms use `xyz` (meters) and `rpy` (roll-pitch-yaw in radians)

## The TF Tree

URDF defines the **static transform tree** of the robot. `robot_state_publisher` reads the URDF, subscribes to `/joint_states`, and publishes transforms:

- **Fixed joints** → static TF (published once via `/tf_static`)
- **Movable joints** → dynamic TF (published on `/tf` at the joint state rate)

Common frame hierarchy:

```
map → odom → base_footprint → base_link → [sensor_frames, wheel_frames]
```

The `map → odom` and `odom → base_footprint` transforms come from localization/odometry, NOT from the URDF. The URDF defines everything from `base_footprint` downward.

## Minimal Two-Link Robot

```xml
<?xml version="1.0"?>
<robot name="minimal_bot">
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="0.15" length="0.08"/></geometry>
      <origin xyz="0 0 0.04" rpy="0 0 0"/>
      <material name="blue"><color rgba="0.0 0.0 0.8 1.0"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="0.15" length="0.08"/></geometry>
      <origin xyz="0 0 0.04" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.04"/>
      <inertia ixx="0.0048" ixy="0" ixz="0" iyy="0.0048" iyz="0" izz="0.009"/>
    </inertial>
  </link>

  <joint name="head_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0.1 0 0.08" rpy="0 0 0"/>
  </joint>

  <link name="head_link">
    <visual>
      <geometry><sphere radius="0.05"/></geometry>
      <material name="red"><color rgba="0.8 0.0 0.0 1.0"/></material>
    </visual>
    <collision>
      <geometry><sphere radius="0.05"/></geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0003" ixy="0" ixz="0" iyy="0.0003" iyz="0" izz="0.0003"/>
    </inertial>
  </link>
</robot>
```

## Validation

```bash
# Check URDF syntax
check_urdf model.urdf

# Visualize the link/joint tree
urdf_to_graphviz model.urdf
```

In ROS 2 Jazzy, install the validation tools with:

```bash
sudo apt install liburdfdom-tools
```
