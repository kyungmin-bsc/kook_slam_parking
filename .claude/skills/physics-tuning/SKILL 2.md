<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: physics-tuning
description: 'Tune Gazebo physics simulation. Use when adjusting step size, solver iterations, contact parameters, or debugging simulation instability.'
---

# Physics Engine Tuning for Stable Simulation

## Step Size

The physics step size (`max_step_size`) is the fundamental timestep of the simulation. Smaller steps are more accurate but slower.

| Step Size | Use Case | Notes |
|-----------|----------|-------|
| 0.001s (1ms) | Default, general robotics | Good balance for most robots |
| 0.0005s (0.5ms) | Fast-moving robots, complex contacts | 2× slower than default |
| 0.002s (2ms) | Simple scenes, slow robots | Faster but less stable |

```xml
<physics name="default" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

Signs you need a smaller step size: robot jitters at rest, wheels bounce, objects pass through each other (tunneling), joints oscillate.

## DART Solver Configuration

DART is the default physics engine. Its solver resolves contacts and constraints.

```xml
<physics name="precise" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <dart>
    <collision_detector>fcl</collision_detector>  <!-- fcl (default), bullet, ode, dart -->
    <solver>
      <solver_type>dantzig</solver_type>  <!-- dantzig (default, accurate) or pgs (faster) -->
    </solver>
  </dart>
</physics>
```

`dantzig`: direct solver, more accurate for contacts and constraints. Default choice.
`pgs` (Projected Gauss-Seidel): iterative solver, faster for many contacts but can be less stable.

## Real-Time Factor Optimization

If RTF < 1.0, the simulation can't keep up with real-time. Diagnose and fix:

```bash
gz stats  # Shows RTF, sim time, wall time
```

**Performance fixes (ordered by impact)**:

1. **Collision geometry**: Replace trimesh collisions with primitive shapes (boxes, cylinders, spheres). A bookshelf visual mesh can have thousands of triangles; its collision should be a single box.

```xml
<!-- BAD: mesh collision (hundreds of triangles) -->
<collision name="c">
  <geometry><mesh><uri>meshes/detailed_table.dae</uri></mesh></geometry>
</collision>

<!-- GOOD: box approximation -->
<collision name="c">
  <geometry><box><size>1.2 0.6 0.75</size></box></geometry>
</collision>
```

2. **Static models**: Mark all non-moving objects as `<static>true</static>`. Static objects skip dynamics calculations entirely.

3. **Sensor rates**: Reduce `<update_rate>` on sensors. A lidar at 40Hz is 4× the cost of 10Hz.

4. **Rendering**: GPU lidar and cameras require rendering. Use `--headless-rendering` in CI or reduce resolution/FPS. Set `<render_engine>ogre2</render_engine>` (default, faster than ogre).

5. **Model count**: Each dynamic model adds solver cost. Combine small static objects into a single model where possible.

## Wheel-Ground Contact (Critical for Diff-Drive)

Incorrect friction causes wheels to slip (robot moves less than commanded) or spin freely. Set friction on both wheels and ground:

```xml
<!-- Wheel link -->
<gazebo reference="left_wheel_link">
  <collision>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>      <!-- Coulomb friction coefficient -->
          <mu2>1.0</mu2>    <!-- Secondary friction direction -->
          <fdir1>0 0 1</fdir1>  <!-- Primary friction direction -->
          <slip1>0.0</slip1>
          <slip2>0.0</slip2>
        </ode>
        <torsional>
          <coefficient>1.0</coefficient>
          <patch_radius>0.05</patch_radius>  <!-- Approximate contact patch -->
          <surface_radius>0.0</surface_radius>
          <use_patch_radius>true</use_patch_radius>
        </torsional>
      </friction>
      <contact>
        <ode>
          <kp>1e6</kp>     <!-- Contact stiffness -->
          <kd>100</kd>      <!-- Contact damping -->
          <max_vel>0.1</max_vel>
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
    </surface>
  </collision>
</gazebo>

<!-- Ground plane (must also have friction) -->
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
      <surface>
        <friction>
          <ode><mu>1.0</mu><mu2>1.0</mu2></ode>
        </friction>
      </surface>
    </collision>
  </link>
</model>
```

Effective friction = `min(mu_wheel, mu_ground)`. If either is 0, wheels slide freely.

## Inertia

Physically realistic inertia is **essential**. Bad inertia causes: robot launching into space, oscillating uncontrollably, or passing through the ground.

Common formulas (uniform density):

```
Box (mass m, dimensions x,y,z):
  Ixx = m/12 * (y² + z²)
  Iyy = m/12 * (x² + z²)
  Izz = m/12 * (x² + y²)

Cylinder (mass m, radius r, length h, about center):
  Ixx = Iyy = m/12 * (3r² + h²)
  Izz = m/2 * r²

Sphere (mass m, radius r):
  Ixx = Iyy = Izz = 2/5 * m * r²
```

In URDF:

```xml
<link name="base_link">
  <inertial>
    <mass value="15.0"/>  <!-- kg, must be realistic -->
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <inertia ixx="0.234" ixy="0" ixz="0"
             iyy="0.312" iyz="0"
             izz="0.468"/>
  </inertial>
</link>

<!-- Wheel: cylinder, mass=0.5kg, r=0.05m, h=0.03m -->
<link name="left_wheel_link">
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.000145" ixy="0" ixz="0"
             iyy="0.000145" iyz="0"
             izz="0.000625"/>  <!-- Izz = 0.5 * 0.5 * 0.05² -->
  </inertial>
</link>
```

**Rules of thumb**:
- Never set mass to 0 (infinite inertia) or extremely small values
- Off-diagonal terms (ixy, ixz, iyz) should usually be 0 unless the object is asymmetric
- All diagonal terms must be positive
- If `ixx + iyy < izz` (triangle inequality violation) for a physical object, something is wrong

## Caster Wheel / Support

For passive casters, use a sphere with low friction to avoid interfering with diff-drive:

```xml
<gazebo reference="caster_link">
  <collision>
    <surface>
      <friction>
        <ode><mu>0.01</mu><mu2>0.01</mu2></ode>
      </friction>
    </surface>
  </collision>
</gazebo>
```

## Common Problems and Fixes

| Problem | Cause | Fix |
|---------|-------|-----|
| Robot flies into space | Bad inertia or mass=0 in a link | Calculate realistic inertia |
| Wheels slip, no traction | Friction mu=0 or missing | Set mu=1.0 on wheels and ground |
| Robot jitters at rest | Step size too large, high stiffness | Reduce step to 0.5ms, lower kp |
| Objects pass through each other | Step too large, thin geometry | Smaller step, thicker collision shapes |
| Simulation too slow (RTF < 0.5) | Complex collisions, high sensor rates | Simplify meshes, reduce update rates |
| Robot sinks into ground | min_depth too large | Set min_depth=0.001 or smaller |
| Wheels bounce | kp too high, kd too low | Reduce kp to 1e5, increase kd to 1e2 |
