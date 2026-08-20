<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: xacro-macros
description: 'Use Xacro macros for URDF. Use when defining reusable macros, properties, conditionals, includes, or parameterized robot components.'
---

# Xacro Macros

## Why Xacro

URDF is verbose and repetitive. Xacro (XML macro language) adds variables, math expressions, macros, conditionals, and file includes. File extension: `.urdf.xacro` or `.xacro`.

Processing:

```bash
# Command line
xacro model.urdf.xacro > model.urdf

# With arguments
xacro model.urdf.xacro use_sim:=true wheel_radius:=0.05 > model.urdf
```

In a ROS 2 launch file:

```python
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

robot_description = ParameterValue(
    Command(['xacro ', PathJoinSubstitution([
        FindPackageShare('my_robot'), 'urdf', 'robot.urdf.xacro'
    ]), ' use_sim:=', use_sim_time]),
    value_type=str
)
```

## Properties (Variables)

```xml
<xacro:property name="wheel_radius" value="0.05"/>
<xacro:property name="wheel_width" value="0.025"/>
<xacro:property name="wheel_separation" value="0.30"/>
<xacro:property name="pi" value="3.14159265359"/>

<!-- Use with ${...} -->
<cylinder radius="${wheel_radius}" length="${wheel_width}"/>
```

## Math Expressions

Xacro evaluates Python expressions inside `${}`:

```xml
<xacro:property name="half_sep" value="${wheel_separation / 2.0}"/>
<origin xyz="0 ${half_sep} 0" rpy="${pi/2} 0 0"/>

<!-- Trigonometry and complex expressions -->
<origin xyz="${0.1 * cos(pi/4)} ${0.1 * sin(pi/4)} 0"/>
```

Available: all Python `math` module functions (`sin`, `cos`, `atan2`, `sqrt`, `pi`, etc.).

## Macros

Define reusable blocks parameterized by arguments:

```xml
<xacro:macro name="wheel" params="prefix y_reflect">
  <link name="${prefix}_wheel_link">
    <visual>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      <material name="black"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      <inertia ixx="${0.5*0.5/12*(3*wheel_radius**2 + wheel_width**2)}"
               ixy="0" ixz="0"
               iyy="${0.5*0.5/12*(3*wheel_radius**2 + wheel_width**2)}"
               iyz="0"
               izz="${0.5*0.5*wheel_radius**2/2}"/>
    </inertial>
  </link>

  <joint name="${prefix}_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="${prefix}_wheel_link"/>
    <origin xyz="0 ${y_reflect * wheel_separation / 2.0} 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.1" friction="0.05"/>
  </joint>
</xacro:macro>

<!-- Instantiate left and right -->
<xacro:wheel prefix="left" y_reflect="1"/>
<xacro:wheel prefix="right" y_reflect="-1"/>
```

## Block Parameters

Pass entire XML blocks as macro parameters using `*`:

```xml
<xacro:macro name="sensor_mount" params="name parent *origin">
  <joint name="${name}_joint" type="fixed">
    <parent link="${parent}"/>
    <child link="${name}_link"/>
    <xacro:insert_block name="origin"/>
  </joint>
  <link name="${name}_link"/>
</xacro:macro>

<xacro:sensor_mount name="lidar" parent="base_link">
  <origin xyz="0.1 0 0.15" rpy="0 0 0"/>
</xacro:sensor_mount>
```

## Conditionals

```xml
<xacro:arg name="use_lidar" default="true"/>

<xacro:if value="$(arg use_lidar)">
  <xacro:include filename="$(find my_robot)/urdf/lidar.xacro"/>
  <xacro:lidar_mount parent="base_link"/>
</xacro:if>

<xacro:unless value="$(arg use_sim)">
  <!-- Hardware-only elements -->
</xacro:unless>
```

## Includes

Split the robot description across files:

```xml
<!-- robot.urdf.xacro (main file) -->
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="use_sim" default="false"/>

  <xacro:include filename="$(find my_robot_description)/urdf/properties.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/base.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/wheels.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/sensors.xacro"/>

  <xacro:if value="$(arg use_sim)">
    <xacro:include filename="$(find my_robot_description)/urdf/gazebo_plugins.xacro"/>
  </xacro:if>
</robot>
```

## Common Pattern: Full Robot Assembly

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="use_sim" default="false"/>

  <!-- Global properties -->
  <xacro:property name="base_length" value="0.40"/>
  <xacro:property name="base_width" value="0.30"/>
  <xacro:property name="base_height" value="0.10"/>
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="wheel_width" value="0.025"/>
  <xacro:property name="wheel_separation" value="0.32"/>

  <!-- Macros -->
  <xacro:include filename="wheel_macro.xacro"/>
  <xacro:include filename="caster_macro.xacro"/>

  <!-- Base link -->
  <link name="base_footprint"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>
  <link name="base_link">
    <!-- visual, collision, inertial for the chassis -->
  </link>

  <!-- Wheels -->
  <xacro:wheel prefix="left" y_reflect="1"/>
  <xacro:wheel prefix="right" y_reflect="-1"/>

  <!-- Caster -->
  <xacro:caster prefix="front" x_offset="${base_length/2 - 0.02}"/>
</robot>
```

## Debugging

```bash
# Expand xacro and check output
xacro robot.urdf.xacro | check_urdf /dev/stdin

# Print expanded XML for inspection
xacro robot.urdf.xacro --inorder
```
