<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: differential-drive-model
description: 'Model a differential drive robot in URDF. Use when building base_link, wheel, caster, and drive joint definitions.'
---

# Differential Drive Model in URDF

## Required Links and Joints

A differential drive robot requires:

| Link | Purpose |
|---|---|
| `base_footprint` | Ground-plane frame (z=0), used by Nav2 for footprint projection |
| `base_link` | Robot body center, elevated by wheel radius |
| `left_wheel_link` | Left drive wheel |
| `right_wheel_link` | Right drive wheel |
| caster link(s) | Passive support wheels |

Frame hierarchy:

```
base_footprint → base_link → left_wheel_link
                            → right_wheel_link
                            → caster_link(s)
                            → sensor frames
```

## base_footprint and base_link

`base_footprint` sits at ground level. `base_link` is offset upward by the wheel radius so the wheel axes are at the correct height:

```xml
<link name="base_footprint"/>

<joint name="base_joint" type="fixed">
  <parent link="base_footprint"/>
  <child link="base_link"/>
  <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
</joint>

<link name="base_link">
  <visual>
    <geometry><box size="${chassis_length} ${chassis_width} ${chassis_height}"/></geometry>
    <origin xyz="0 0 ${chassis_height/2}"/>
    <material name="gray"><color rgba="0.5 0.5 0.5 1"/></material>
  </visual>
  <collision>
    <geometry><box size="${chassis_length} ${chassis_width} ${chassis_height}"/></geometry>
    <origin xyz="0 0 ${chassis_height/2}"/>
  </collision>
  <xacro:box_inertia m="${chassis_mass}"
                     x="${chassis_length}" y="${chassis_width}" z="${chassis_height}"/>
</link>
```

## Wheel Joints

Drive wheels use `continuous` joints. The axis is `[0 1 0]` (rotation around the y-axis, which is the wheel's spinning axis when the joint origin places the wheel to the side):

```xml
<xacro:macro name="drive_wheel" params="prefix y_offset">
  <joint name="${prefix}_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="${prefix}_wheel_link"/>
    <origin xyz="0 ${y_offset} 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.1" friction="0.05"/>
    <limit effort="10.0" velocity="20.0"/>
  </joint>

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
    <xacro:cylinder_inertia m="${wheel_mass}" r="${wheel_radius}" h="${wheel_width}"/>
  </link>
</xacro:macro>

<xacro:drive_wheel prefix="left"  y_offset="${wheel_separation/2}"/>
<xacro:drive_wheel prefix="right" y_offset="${-wheel_separation/2}"/>
```

Note the `rpy="${pi/2} 0 0"` on the visual/collision geometry: URDF cylinders extend along their z-axis, but we want the wheel disc facing sideways (y-axis). The π/2 roll rotation aligns the cylinder correctly.

## Wheel Separation

**Wheel separation** is the center-to-center distance between the two drive wheels. This is the most critical parameter for odometry accuracy:

```xml
<xacro:property name="wheel_separation" value="0.30"/>  <!-- meters, measured carefully -->
```

If wheel separation is wrong, the robot will:
- Turn too much or too little for a given command
- Accumulate heading error during straight-line driving
- Have incorrect odometry-based localization

## Caster Wheels

### Ball Caster (Simple)

Model as a sphere on a fixed joint. The sphere collision provides passive ground contact:

```xml
<xacro:macro name="ball_caster" params="prefix x_offset">
  <joint name="${prefix}_caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="${prefix}_caster_link"/>
    <origin xyz="${x_offset} 0 ${-wheel_radius + caster_radius}" rpy="0 0 0"/>
  </joint>

  <link name="${prefix}_caster_link">
    <visual>
      <geometry><sphere radius="${caster_radius}"/></geometry>
      <material name="silver"><color rgba="0.8 0.8 0.8 1"/></material>
    </visual>
    <collision>
      <geometry><sphere radius="${caster_radius}"/></geometry>
    </collision>
    <xacro:sphere_inertia m="0.1" r="${caster_radius}"/>
  </link>
</xacro:macro>

<xacro:ball_caster prefix="front" x_offset="${chassis_length/2 - 0.03}"/>
<xacro:ball_caster prefix="rear"  x_offset="${-chassis_length/2 + 0.03}"/>
```

Set the caster friction low in Gazebo so it doesn't resist chassis rotation:

```xml
<gazebo reference="front_caster_link">
  <mu1>0.01</mu1>
  <mu2>0.01</mu2>
</gazebo>
```

### Swivel Caster (Realistic)

Two joints: revolute for swivel + continuous for wheel spin:

```xml
<joint name="caster_swivel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="caster_swivel_link"/>
  <origin xyz="0.15 0 -0.02" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <dynamics damping="0.01"/>
</joint>
<link name="caster_swivel_link">
  <xacro:sphere_inertia m="0.05" r="0.01"/>
</link>

<joint name="caster_wheel_joint" type="continuous">
  <parent link="caster_swivel_link"/>
  <child link="caster_wheel_link"/>
  <origin xyz="0.02 0 ${-wheel_radius + caster_wheel_radius}"/>
  <axis xyz="0 1 0"/>
  <dynamics damping="0.01"/>
</joint>
<link name="caster_wheel_link">
  <visual>
    <geometry><cylinder radius="${caster_wheel_radius}" length="0.015"/></geometry>
    <origin rpy="${pi/2} 0 0"/>
  </visual>
  <collision>
    <geometry><cylinder radius="${caster_wheel_radius}" length="0.015"/></geometry>
    <origin rpy="${pi/2} 0 0"/>
  </collision>
  <xacro:cylinder_inertia m="0.05" r="${caster_wheel_radius}" h="0.015"/>
</link>
```

## Complete Differential Drive Properties

```xml
<xacro:property name="chassis_length" value="0.40"/>
<xacro:property name="chassis_width" value="0.30"/>
<xacro:property name="chassis_height" value="0.10"/>
<xacro:property name="chassis_mass" value="5.0"/>

<xacro:property name="wheel_radius" value="0.05"/>
<xacro:property name="wheel_width" value="0.025"/>
<xacro:property name="wheel_separation" value="0.32"/>
<xacro:property name="wheel_mass" value="0.5"/>

<xacro:property name="caster_radius" value="0.02"/>
```

## Gazebo Diff-Drive Plugin

For simulation, attach the `gz-sim-diff-drive-system` plugin:

```xml
<gazebo>
  <plugin filename="gz-sim-diff-drive-system"
          name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>${wheel_separation}</wheel_separation>
    <wheel_radius>${wheel_radius}</wheel_radius>
    <odom_publish_frequency>30</odom_publish_frequency>
    <topic>cmd_vel</topic>
    <odom_topic>odom</odom_topic>
    <frame_id>odom</frame_id>
    <child_frame_id>base_footprint</child_frame_id>
  </plugin>
</gazebo>
```

The plugin parameters `wheel_separation` and `wheel_radius` **must match** the URDF values exactly—otherwise simulated odometry diverges from the actual wheel motion.
