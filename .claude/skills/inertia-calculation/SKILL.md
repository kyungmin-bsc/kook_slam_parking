<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: inertia-calculation
description: 'Calculate inertial properties for URDF. Use when computing mass, center of mass, and inertia tensors for box, cylinder, or sphere links.'
---

# Inertia Calculation

## Why Correct Inertias Matter

Gazebo's physics engine **requires** valid inertial parameters for every link in the simulation. Incorrect inertias cause:
- Simulation instability (robot flies away, oscillates, or explodes)
- Unrealistic dynamics (robot slides, tips, or doesn't respond to forces)
- Solver divergence (Gazebo crashes or produces NaN values)

## The `<inertial>` Element

```xml
<inertial>
  <mass value="2.5"/>
  <origin xyz="0 0 0.03" rpy="0 0 0"/>  <!-- center of mass -->
  <inertia ixx="0.0108" ixy="0.0" ixz="0.0"
           iyy="0.0417" iyz="0.0"
           izz="0.0483"/>
</inertial>
```

- **mass**: in kilograms
- **origin**: center of mass relative to the link's origin frame
- **inertia**: 3×3 symmetric inertia tensor (6 unique values), in kg·m², about the center of mass

The inertia tensor matrix:

$$
I = \begin{bmatrix} i_{xx} & i_{xy} & i_{xz} \\ i_{xy} & i_{yy} & i_{yz} \\ i_{xz} & i_{yz} & i_{zz} \end{bmatrix}
$$

## Standard Shape Formulas

### Box (dimensions $l_x$, $l_y$, $l_z$, mass $m$)

$$
i_{xx} = \frac{m}{12}(l_y^2 + l_z^2), \quad
i_{yy} = \frac{m}{12}(l_x^2 + l_z^2), \quad
i_{zz} = \frac{m}{12}(l_x^2 + l_y^2)
$$

All off-diagonal terms = 0 (when axes align with principal axes).

### Cylinder (radius $r$, height $h$ along z-axis, mass $m$)

$$
i_{xx} = i_{yy} = \frac{m}{12}(3r^2 + h^2), \quad
i_{zz} = \frac{mr^2}{2}
$$

### Sphere (radius $r$, mass $m$)

$$
i_{xx} = i_{yy} = i_{zz} = \frac{2mr^2}{5}
$$

## Xacro Inertia Macros

### Box Inertia Macro

```xml
<xacro:macro name="box_inertia" params="m x y z">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${m/12*(y*y + z*z)}" ixy="0" ixz="0"
             iyy="${m/12*(x*x + z*z)}" iyz="0"
             izz="${m/12*(x*x + y*y)}"/>
  </inertial>
</xacro:macro>

<!-- Usage: chassis 0.4m x 0.3m x 0.1m, 5kg -->
<link name="base_link">
  <xacro:box_inertia m="5.0" x="0.4" y="0.3" z="0.1"/>
  <visual>
    <geometry><box size="0.4 0.3 0.1"/></geometry>
  </visual>
</link>
```

### Cylinder Inertia Macro

```xml
<xacro:macro name="cylinder_inertia" params="m r h">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${m/12*(3*r*r + h*h)}" ixy="0" ixz="0"
             iyy="${m/12*(3*r*r + h*h)}" iyz="0"
             izz="${m*r*r/2}"/>
  </inertial>
</xacro:macro>
```

### Sphere Inertia Macro

```xml
<xacro:macro name="sphere_inertia" params="m r">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${2*m*r*r/5}" ixy="0" ixz="0"
             iyy="${2*m*r*r/5}" iyz="0"
             izz="${2*m*r*r/5}"/>
  </inertial>
</xacro:macro>
```

## Parallel Axis Theorem

When combining shapes or when the center of mass is offset from the link origin, use the parallel axis theorem:

$$
I_{new} = I_{cm} + m \cdot d^2
$$

For a full 3D shift by $(d_x, d_y, d_z)$:

$$
i_{xx}' = i_{xx} + m(d_y^2 + d_z^2)
$$
$$
i_{yy}' = i_{yy} + m(d_x^2 + d_z^2)
$$
$$
i_{zz}' = i_{zz} + m(d_x^2 + d_y^2)
$$
$$
i_{xy}' = i_{xy} - m \cdot d_x \cdot d_y
$$

This is needed when the `<origin>` in `<inertial>` is non-zero, or when approximating a complex body as a combination of primitives.

## CAD Export

Fusion 360 and SolidWorks can compute inertial properties directly:

- **Fusion 360**: Inspect → Physical Properties → shows mass, center of mass, and inertia tensor
- **SolidWorks**: Evaluate → Mass Properties

Ensure the coordinate frame in the CAD tool matches the URDF link frame. Export values and paste into the URDF.

## Sanity Checks

Before running simulation, verify:

1. **All diagonal components positive**: $i_{xx} > 0$, $i_{yy} > 0$, $i_{zz} > 0$
2. **Triangle inequality**: $i_{xx} + i_{yy} \geq i_{zz}$ (and cyclic permutations)
3. **Off-diagonals bounded**: $|i_{xy}| \leq \sqrt{i_{xx} \cdot i_{yy}}$
4. **Reasonable magnitudes**: for a 1 kg, 0.1 m object, inertias ~0.001 kg·m²
5. **No huge ratios**: adjacent links shouldn't have inertia ratios > 100:1

## Common Mistakes

| Mistake | Symptom | Fix |
|---|---|---|
| Zero inertia (all zeros) | Simulation crash or NaN | Use formulas above; even tiny links need nonzero inertia |
| Inertia at link origin, not CoM | Asymmetric behavior | Set `<origin>` to actual center of mass |
| Mass too small for link size | Link flies away under contact | Verify mass is physically reasonable |
| Huge mass ratio between links | Solver instability | Keep mass ratios < 50:1 between connected links |
| Forgetting `<inertial>` entirely | Link treated as zero-mass (fixed to world in some solvers) | Always include for simulated links |

## Quick Estimation Script

```python
def box_inertia(m, x, y, z):
    return {
        'ixx': m/12*(y**2 + z**2),
        'iyy': m/12*(x**2 + z**2),
        'izz': m/12*(x**2 + y**2),
    }

def cylinder_inertia(m, r, h):
    return {
        'ixx': m/12*(3*r**2 + h**2),
        'iyy': m/12*(3*r**2 + h**2),
        'izz': m*r**2/2,
    }

# Example: the robot chassis ~5kg, 0.4m x 0.3m x 0.1m
print(box_inertia(5.0, 0.4, 0.3, 0.1))
# {'ixx': 0.0417, 'iyy': 0.0708, 'izz': 0.1042}
```
