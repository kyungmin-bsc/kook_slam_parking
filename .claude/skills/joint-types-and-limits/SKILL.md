<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: joint-types-and-limits
description: 'Configure URDF joint types and limits. Use when setting up fixed, continuous, revolute, or prismatic joints with effort/velocity limits.'
---

# Joint Types and Limits

## Joint Type Reference

### Fixed Joint

No motion—rigidly attaches child to parent. Used for sensor mounts, structural elements, and `base_footprint → base_link`.

```xml
<joint name="lidar_mount_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.1 0 0.15" rpy="0 0 0"/>
</joint>
```

No `<axis>` or `<limit>` needed for fixed joints.

### Continuous Joint

Unlimited rotation around a single axis. No position limits. Used for drive wheels and free-spinning casters.

```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel_link"/>
  <origin xyz="0 0.16 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <dynamics damping="0.1" friction="0.05"/>
  <limit effort="10.0" velocity="20.0"/>
</joint>
```

Even continuous joints should specify `effort` and `velocity` limits for Gazebo enforcement.

### Revolute Joint

Bounded rotation. Requires `<limit>` with `lower` and `upper` (in radians).

```xml
<joint name="pan_joint" type="revolute">
  <parent link="base_link"/>
  <child link="pan_link"/>
  <origin xyz="0.05 0 0.12" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="${-pi/2}" upper="${pi/2}" effort="5.0" velocity="1.57"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

### Prismatic Joint

Linear sliding along an axis. Limits in meters.

```xml
<joint name="elevator_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="elevator_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0.0" upper="0.25" effort="50.0" velocity="0.1"/>
  <dynamics damping="1.0"/>
</joint>
```

## Joint Sub-Elements

### `<origin>`

Transform from parent link frame to the joint frame. Not the child link origin—the child link frame coincides with the joint frame at zero position.

```xml
<!-- Place the joint 0.16m to the left of parent origin -->
<origin xyz="0 0.16 0" rpy="0 0 0"/>
```

### `<axis>`

The axis of rotation (revolute/continuous) or translation (prismatic). Normalized vector in the **joint frame**.

```xml
<!-- Rotation around the Y axis (typical for wheels) -->
<axis xyz="0 1 0"/>

<!-- Rotation around the Z axis (typical for pan/yaw) -->
<axis xyz="0 0 1"/>
```

### `<limit>`

| Attribute | Unit | Required For |
|---|---|---|
| `lower` | rad / m | revolute, prismatic |
| `upper` | rad / m | revolute, prismatic |
| `effort` | N·m / N | all movable joints |
| `velocity` | rad/s / m/s | all movable joints |

```xml
<limit lower="-1.57" upper="1.57" effort="10.0" velocity="3.14"/>
```

For continuous joints, `lower` and `upper` are ignored but `effort` and `velocity` are still enforced by Gazebo.

### `<dynamics>`

```xml
<dynamics damping="0.5" friction="0.1"/>
```

- **damping**: viscous damping coefficient (N·m·s/rad). Higher values slow rotation.
- **friction**: Coulomb friction (N·m). Static resistance to motion.

These affect simulation fidelity. Start with small values (0.01–0.1) and tune.

## Safety Controller

Applies soft limits before the hard stops defined in `<limit>`:

```xml
<safety_controller soft_lower_limit="-1.40"
                   soft_upper_limit="1.40"
                   k_position="100"
                   k_velocity="10"/>
```

The joint decelerates when entering the soft limit zone, preventing impact at the hard stop. Primarily used with `ros2_control` hardware interfaces.

## Mimic Joints

For coupled joints where one joint's position is a function of another:

```xml
<!-- Gripper: right finger mirrors left finger -->
<joint name="right_finger_joint" type="prismatic">
  <parent link="gripper_base"/>
  <child link="right_finger"/>
  <origin xyz="0 -0.02 0.05"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="0.04" effort="20" velocity="0.1"/>
  <mimic joint="left_finger_joint" multiplier="-1.0" offset="0.0"/>
</joint>
```

- **multiplier**: `mimic_position = multiplier * leader_position + offset`
- **offset**: constant position offset
- In Gazebo, mimic joints need a plugin (e.g., `gz-sim-joint-controller-system`) or a ros2_control mimic interface.

## Complete Pan-Tilt Example

```xml
<!-- Pan (yaw) -->
<joint name="pan_joint" type="revolute">
  <parent link="base_link"/>
  <child link="pan_link"/>
  <origin xyz="0.05 0 0.10" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="${-pi*0.75}" upper="${pi*0.75}" effort="2.0" velocity="2.0"/>
  <dynamics damping="0.3" friction="0.05"/>
</joint>

<link name="pan_link">
  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<!-- Tilt (pitch) -->
<joint name="tilt_joint" type="revolute">
  <parent link="pan_link"/>
  <child link="tilt_link"/>
  <origin xyz="0 0 0.03" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="${-pi/4}" upper="${pi/3}" effort="2.0" velocity="1.5"/>
  <dynamics damping="0.3" friction="0.05"/>
</joint>

<link name="tilt_link">
  <inertial>
    <mass value="0.15"/>
    <inertia ixx="0.00005" ixy="0" ixz="0" iyy="0.00005" iyz="0" izz="0.00003"/>
  </inertial>
</link>
```

## Debugging Joint Issues

```bash
# Verify joint limits in the parsed URDF
check_urdf robot.urdf

# In Gazebo: watch for "Joint position limit reached" warnings
# In ros2_control: check /joint_states for clamped values

# Inspect current joint states
ros2 topic echo /joint_states
```

Common pitfalls:
- Forgetting `effort`/`velocity` limits → Gazebo applies zero effort → joint won't move
- Axis direction wrong → wheel spins sideways or joint rotates unexpectedly
- Origin placed at child link center instead of at the joint pivot point
