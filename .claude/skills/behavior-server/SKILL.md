<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: behavior-server
description: 'Configure Nav2 behavior server. Use when setting up recovery behaviors, spin, backup, wait, or custom behavior plugins.'
---

# Behavior Server

## Overview

The behavior server (formerly "recovery server" in pre-Humble Nav2) hosts behavior plugins that execute recovery and assistive actions. When the BT Navigator determines that planning or control has failed, it calls behavior actions like spinning, backing up, or waiting.

## Configuration

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait", "assisted_teleop"]
    cycle_frequency: 10.0
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
    robot_base_frame: base_link
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    wait:
      plugin: "nav2_behaviors::Wait"
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
```

## Parameters

### Server-Level Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `behavior_plugins` | `["spin", "backup", "drive_on_heading", "wait"]` | List of behavior plugin names |
| `cycle_frequency` | `10.0` | Hz for behavior execution loop |
| `local_costmap_topic` | `local_costmap/costmap_raw` | For collision checking during behaviors |
| `global_costmap_topic` | `global_costmap/costmap_raw` | For collision checking during behaviors |
| `local_footprint_topic` | `local_costmap/published_footprint` | Robot footprint for collision checking |
| `global_footprint_topic` | `global_costmap/published_footprint` | Robot footprint for collision checking |
| `transform_tolerance` | `0.1` | TF lookup tolerance in seconds |
| `simulate_ahead_time` | `2.0` | Seconds to simulate forward for collision checking before executing motion |

### Spin Parameters

```yaml
spin:
  plugin: "nav2_behaviors::Spin"
  # Uses server-level parameters:
  # max_rotational_vel, min_rotational_vel, rotational_acc_lim
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_rotational_vel` | `1.0` | Max angular velocity (rad/s) during spin |
| `min_rotational_vel` | `0.4` | Min angular velocity (rad/s) during spin |
| `rotational_acc_lim` | `3.2` | Angular acceleration limit (rad/s²) |

The spin behavior rotates the robot in place by a specified angle (default 1.57 rad = 90°). It's the primary recovery for getting unstuck when the planner fails.

### BackUp Parameters

The backup behavior drives the robot straight backward:

```yaml
# No specific plugin parameters - uses action goal values
# Default: backup_dist=0.15m, backup_speed=0.025 m/s
```

### DriveOnHeading Parameters

Drives straight forward/backward along the robot's current heading:

```yaml
# Similar to backup - uses action goal values
# Useful for nudging through tight spaces
```

### Wait Parameters

Simply waits for a specified duration, allowing dynamic obstacles to move:

```yaml
# Uses action goal: wait_duration (seconds)
```

### AssistedTeleop

Allows operator teleoperation while the behavior server provides collision avoidance:

```yaml
assisted_teleop:
  plugin: "nav2_behaviors::AssistedTeleop"
  # Subscribes to cmd_vel_teleop, publishes collision-safe cmd_vel
```

## How the BT Calls Behaviors

In the behavior tree XML, behavior actions are called as BT nodes:

```xml
<!-- Spin 90 degrees -->
<Spin spin_dist="1.57" server_timeout="10000"
      error_code_id="{spin_error_code}"/>

<!-- Back up 30cm at 0.1 m/s -->
<BackUp backup_dist="0.3" backup_speed="0.1" server_timeout="10000"
        error_code_id="{backup_error_code}"/>

<!-- Drive forward 20cm at 0.1 m/s -->
<DriveOnHeading dist_to_travel="0.2" speed="0.1" server_timeout="10000"
                error_code_id="{drive_error_code}"/>

<!-- Wait 5 seconds -->
<Wait wait_duration="5" server_timeout="10000"/>
```

### Typical Recovery Sequence in BT

```xml
<RecoveryNode number_of_retries="6" name="NavigateRecovery">
  <PipelineSequence name="NavigateWithReplanning">
    <RateController hz="1.0">
      <ComputePathToPose goal="{goal}" path="{path}"
                         error_code_id="{compute_path_error}"/>
    </RateController>
    <FollowPath path="{path}" controller_id="FollowPath"
                error_code_id="{follow_path_error}"/>
  </PipelineSequence>
  <ReactiveFallback name="RecoveryFallback">
    <GoalUpdated/>
    <RoundRobin name="RecoveryActions">
      <Sequence name="ClearAndSpin">
        <ClearEntireCostmap name="ClearLocal"
            service_name="/local_costmap/clear_entirely_local_costmap"/>
        <ClearEntireCostmap name="ClearGlobal"
            service_name="/global_costmap/clear_entirely_global_costmap"/>
        <Spin spin_dist="1.57" error_code_id="{spin_error}"/>
      </Sequence>
      <Sequence name="ClearAndBackup">
        <ClearEntireCostmap name="ClearLocal"
            service_name="/local_costmap/clear_entirely_local_costmap"/>
        <BackUp backup_dist="0.3" backup_speed="0.1"
                error_code_id="{backup_error}"/>
      </Sequence>
      <Wait wait_duration="5"/>
    </RoundRobin>
  </ReactiveFallback>
</RecoveryNode>
```

The `RoundRobin` cycles through recovery strategies: first try clear+spin, then clear+backup, then wait. `number_of_retries="6"` means it retries the whole navigate+recover cycle up to 6 times.

## Collision Checking During Behaviors

Before executing a spin or backup, the behavior server:

1. Gets the robot's current footprint from the footprint topic
2. Simulates the motion forward by `simulate_ahead_time` seconds
3. Checks each simulated pose against the costmap
4. If any simulated pose collides, returns `COLLISION_AHEAD` error code

This prevents the robot from spinning into a wall or backing into furniture. If `simulate_ahead_time` is too short, the robot may not check far enough and still collide.

## Common Issues

### Behavior Blocked by Phantom Costmap Obstacle
**Symptom**: Spin or backup immediately returns `COLLISION_AHEAD` but there's nothing around the robot.
**Cause**: The costmap has stale obstacle data (e.g., from a person who walked away). The simulated spin trajectory intersects this phantom obstacle.
**Fix**: Clear costmaps before attempting recovery. In the BT, put `ClearEntireCostmap` before `Spin`:

```xml
<Sequence>
  <ClearEntireCostmap service_name="/local_costmap/clear_entirely_local_costmap"/>
  <Spin spin_dist="1.57"/>
</Sequence>
```

### Spin Too Slow or Too Fast
Adjust `max_rotational_vel` and `rotational_acc_lim`. For a heavy robot, reduce both to prevent tipping. For a small robot on carpet, increase `min_rotational_vel` to overcome friction.

### Backup Distance Insufficient
The default backup distance (0.15m) is often too small to clear the robot from a stuck position. Increase to 0.3–0.5m in the BT XML:

```xml
<BackUp backup_dist="0.4" backup_speed="0.15"/>
```

### Behavior Actions Not Found
**Symptom**: BT fails with `"Action server not available"` for spin/backup.
**Cause**: The behavior server is not running or the plugin is not listed in `behavior_plugins`.
**Fix**: Verify the behavior server is in the lifecycle manager's `node_names` list and all plugins are listed.

## Custom Behavior Plugin

To create a custom behavior:

```cpp
#include "nav2_core/behavior.hpp"

class MyBehavior : public nav2_core::Behavior<my_msgs::action::MyAction>
{
public:
  void onConfigure() override;
  Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) override;
  Status onCycleUpdate() override;
  void onActionCompletion() override;
};
```

Register with `pluginlib` and add to `behavior_plugins` in YAML.
