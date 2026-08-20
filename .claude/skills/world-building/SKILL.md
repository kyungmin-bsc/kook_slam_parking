<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: world-building
description: 'Build Gazebo SDF world files. Use when creating indoor environments, adding walls, furniture, lighting, or importing mesh models.'
---

# Building SDF World Files

## SDF World Structure

SDF (Simulation Description Format) is the native format for Gazebo worlds. It describes everything: physics, lighting, terrain, objects, and sensors. Coordinate system matches ROS: X-forward, Y-left, Z-up (ENU).

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="house_patrol">
    <!-- Physics, scene, lighting, models go here -->
  </world>
</sdf>
```

## Physics Configuration

```xml
<physics name="1ms" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

`max_step_size`: simulation timestep (default 1ms). Smaller = more accurate, slower. `real_time_factor`: 1.0 = real-time, 0.5 = half speed. `real_time_update_rate`: ignored when `real_time_factor` is set; otherwise controls target updates/sec.

## Scene and Lighting

```xml
<scene>
  <ambient>0.4 0.4 0.4 1</ambient>
  <background>0.7 0.8 0.9 1</background>
  <shadows>true</shadows>
</scene>

<!-- Sunlight (directional) -->
<light type="directional" name="sun">
  <cast_shadows>true</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.3 0.3 0.3 1</specular>
  <attenuation>
    <range>1000</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <direction>-0.5 0.1 -0.9</direction>
</light>

<!-- Indoor point light (ceiling lamp) -->
<light type="point" name="ceiling_light_1">
  <pose>3.0 2.0 2.4 0 0 0</pose>
  <diffuse>1.0 0.95 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>10</range>
    <constant>0.5</constant>
    <linear>0.1</linear>
    <quadratic>0.02</quadratic>
  </attenuation>
  <cast_shadows>false</cast_shadows>
</light>

<!-- Spot light (desk lamp) -->
<light type="spot" name="desk_lamp">
  <pose>5.0 1.0 1.5 0 0.5 0</pose>
  <diffuse>1.0 1.0 0.9 1</diffuse>
  <spot>
    <inner_angle>0.3</inner_angle>
    <outer_angle>0.6</outer_angle>
    <falloff>1.0</falloff>
  </spot>
</light>
```

## Ground Plane

```xml
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
      <surface>
        <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
      </surface>
    </collision>
    <visual name="visual">
      <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
      <material>
        <ambient>0.6 0.6 0.6 1</ambient>
        <diffuse>0.6 0.6 0.6 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## Wall Construction

Build walls using box geometry. The `<static>true</static>` flag makes objects immovable and excludes them from physics dynamics (much faster).

```xml
<!-- Wall macro pattern: pose is center of the box -->
<model name="wall_north">
  <static>true</static>
  <pose>5.0 0 1.25 0 0 0</pose>  <!-- x y z roll pitch yaw -->
  <link name="link">
    <collision name="collision">
      <geometry><box><size>10.0 0.15 2.5</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>10.0 0.15 2.5</size></box></geometry>
      <material>
        <ambient>0.9 0.9 0.85 1</ambient>
        <diffuse>0.9 0.9 0.85 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## Including External Models

```xml
<!-- From Gazebo Fuel (auto-downloads) -->
<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table</uri>
  <name>kitchen_table</name>
  <pose>3.0 2.0 0 0 0 0</pose>
  <static>true</static>
</include>

<!-- From local model path (set GZ_SIM_RESOURCE_PATH) -->
<include>
  <uri>model://bookshelf</uri>
  <name>bookshelf_1</name>
  <pose>0.5 4.0 0 0 0 1.5708</pose>
</include>
```

## Mesh Models

Use Blender or SketchUp to create custom meshes. Export as COLLADA (`.dae`) or OBJ. Separate visual and collision meshes—collision should be simplified (convex hull or primitive approximation).

Model directory structure:
```
my_model/
├── model.sdf
├── model.config
└── meshes/
    ├── model_visual.dae
    └── model_collision.dae
```

```xml
<!-- model.sdf -->
<sdf version="1.9">
  <model name="couch">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><mesh><uri>meshes/model_collision.dae</uri><scale>1 1 1</scale></mesh></geometry>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>meshes/model_visual.dae</uri><scale>1 1 1</scale></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
```

## Complete Indoor World

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="house_patrol">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.8 0.9 1</background>
    </scene>

    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Floor -->
    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><diffuse>0.7 0.65 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Room: 8m x 6m, walls 2.5m high, 0.15m thick -->
    <!-- South wall -->
    <model name="wall_south">
      <static>true</static>
      <pose>4.0 0.075 1.25 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>8.0 0.15 2.5</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>8.0 0.15 2.5</size></box></geometry>
          <material><diffuse>0.9 0.9 0.85 1</diffuse></material></visual>
      </link>
    </model>
    <!-- North wall -->
    <model name="wall_north">
      <static>true</static>
      <pose>4.0 5.925 1.25 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>8.0 0.15 2.5</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>8.0 0.15 2.5</size></box></geometry>
          <material><diffuse>0.9 0.9 0.85 1</diffuse></material></visual>
      </link>
    </model>
    <!-- West wall -->
    <model name="wall_west">
      <static>true</static>
      <pose>0.075 3.0 1.25 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.15 6.0 2.5</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.15 6.0 2.5</size></box></geometry>
          <material><diffuse>0.9 0.9 0.85 1</diffuse></material></visual>
      </link>
    </model>
    <!-- East wall with doorway (split into two sections) -->
    <model name="wall_east_top">
      <static>true</static>
      <pose>7.925 4.5 1.25 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.15 3.0 2.5</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.15 3.0 2.5</size></box></geometry>
          <material><diffuse>0.9 0.9 0.85 1</diffuse></material></visual>
      </link>
    </model>
    <model name="wall_east_bottom">
      <static>true</static>
      <pose>7.925 0.75 1.25 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.15 1.5 2.5</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.15 1.5 2.5</size></box></geometry>
          <material><diffuse>0.9 0.9 0.85 1</diffuse></material></visual>
      </link>
    </model>

    <!-- Furniture from Fuel -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table</uri>
      <name>table</name>
      <pose>4.0 3.0 0 0 0 0</pose>
      <static>true</static>
    </include>
  </world>
</sdf>
```

## Building Tips

- Measure real rooms and translate to meters. Pose `<x y z roll pitch yaw>` is the center of geometry.
- Use `<static>true</static>` for all immovable objects—dramatically improves performance.
- Keep collision meshes as simple as possible: boxes for walls, cylinders for table legs.
- Test with `gz sim -r house.sdf` before integrating with ROS 2.
