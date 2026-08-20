<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: bt-action-nodes-nav2
description: 'Use Nav2 BT action nodes. Use when adding NavigateToPose, ComputePathToPose, FollowPath, Spin, BackUp, Wait, or ClearCostmap actions.'
---

# Nav2 Built-in Action BT Nodes

## Core Navigation Actions

### NavigateToPose

The primary single-goal navigation action. Calls the `/navigate_to_pose` action server, which internally runs its own BT for planning, controlling, and recovery.

```xml
<NavigateToPose goal="{goal}" server_name="navigate_to_pose"
                server_timeout="10000" error_code_id="{navigate_error}" />
```

| Port             | Direction | Type                      | Description                    |
|------------------|-----------|---------------------------|--------------------------------|
| goal             | input     | PoseStamped               | Target pose                    |
| behavior_tree    | input     | string                    | Override BT XML path (optional)|
| server_name      | input     | string                    | Action server name             |
| server_timeout   | input     | int (ms)                  | Server connection timeout      |
| error_code_id    | output    | uint16                    | Error code on failure          |

### NavigateThroughPoses

Multi-goal variant. Sends all goals at once to the planner for a single optimized path:

```xml
<NavigateThroughPoses goals="{goal_poses}" server_name="navigate_through_poses"
                      server_timeout="10000" error_code_id="{nav_error}" />
```

| Port             | Direction | Type                       | Description                    |
|------------------|-----------|----------------------------|--------------------------------|
| goals            | input     | vector\<PoseStamped\>      | Ordered list of poses          |
| error_code_id    | output    | uint16                     | Error code on failure          |

### ComputePathToPose

Calls the planner server to compute a path without executing it. This is the planning-only action:

```xml
<ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"
                   server_name="compute_path_to_pose"
                   server_timeout="10000" error_code_id="{plan_error}" />
```

| Port             | Direction | Type          | Description                        |
|------------------|-----------|---------------|------------------------------------|
| goal             | input     | PoseStamped   | Target pose                        |
| start            | input     | PoseStamped   | Start pose (default: current pose) |
| planner_id       | input     | string        | Which planner plugin to use        |
| path             | output    | Path          | Computed path                      |
| error_code_id    | output    | uint16        | Planning error code                |

### ComputePathThroughPoses

Multi-goal planning variant:

```xml
<ComputePathThroughPoses goals="{waypoints}" path="{path}" planner_id="GridBased"
                         server_name="compute_path_through_poses"
                         error_code_id="{plan_error}" />
```

### FollowPath

Sends a precomputed path to the controller server for execution:

```xml
<FollowPath path="{path}" controller_id="FollowPath"
            server_name="follow_path" server_timeout="10000"
            error_code_id="{follow_error}" />
```

| Port             | Direction | Type   | Description                         |
|------------------|-----------|--------|-------------------------------------|
| path             | input     | Path   | Path to follow                      |
| controller_id    | input     | string | Which controller plugin to use      |
| goal_checker_id  | input     | string | Which goal checker to use           |
| progress_checker_id | input  | string | Which progress checker to use       |
| error_code_id    | output    | uint16 | Controller error code               |

## The Compute → Follow Pipeline

The fundamental Nav2 BT pattern connects planner output to controller input via the blackboard:

```xml
<Sequence>
  <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" />
  <FollowPath path="{path}" controller_id="FollowPath" />
</Sequence>
```

The `{path}` blackboard variable is the bridge: `ComputePathToPose` writes it (output port), `FollowPath` reads it (input port). This decoupling allows inserting path smoothing between them:

```xml
<Sequence>
  <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" />
  <SmoothPath unsmoothed_path="{path}" smoothed_path="{path}"
              smoother_id="simple_smoother" />
  <FollowPath path="{path}" controller_id="FollowPath" />
</Sequence>
```

## Recovery and Behavior Actions

### Spin

Rotates the robot in place. Useful for clearing costmap artifacts or looking around:

```xml
<Spin spin_dist="1.57" server_name="spin" server_timeout="10000"
      error_code_id="{spin_error}" />
```

| Port       | Direction | Type   | Description                      |
|------------|-----------|--------|----------------------------------|
| spin_dist  | input     | double | Rotation angle in radians        |
| time_allowance | input | int (ms) | Max time for the behavior      |

### Wait

Pauses for a duration. Does not halt other branches in a Parallel node:

```xml
<Wait wait_duration="5.0" server_name="wait" server_timeout="10000" />
```

### BackUp

Drives the robot backward:

```xml
<BackUp backup_dist="0.3" backup_speed="0.1" server_name="backup"
        server_timeout="10000" error_code_id="{backup_error}" />
```

| Port         | Direction | Type   | Description              |
|--------------|-----------|--------|--------------------------|
| backup_dist  | input     | double | Distance in meters       |
| backup_speed | input     | double | Speed in m/s             |
| time_allowance | input   | int    | Max time in seconds      |

### DriveOnHeading

Drives in a straight line on the current heading:

```xml
<DriveOnHeading dist_to_travel="0.5" speed="0.2" time_allowance="10"
                server_name="drive_on_heading" error_code_id="{drive_error}" />
```

### AssistedTeleop

Blends teleoperation commands with obstacle avoidance:

```xml
<AssistedTeleop time_allowance="30" server_name="assisted_teleop"
                error_code_id="{teleop_error}" />
```

## Costmap Actions

### ClearEntireCostmap

Resets a costmap layer completely. Use for global or local costmaps:

```xml
<ClearEntireCostmap server_name="global_costmap/clear_entirely_global_costmap"
                    server_timeout="5000" />
<ClearEntireCostmap server_name="local_costmap/clear_entirely_local_costmap"
                    server_timeout="5000" />
```

### ClearCostmapExceptRegion

Clears the costmap except within a radius around the robot:

```xml
<ClearCostmapExceptRegion server_name="global_costmap/clear_except_global_costmap"
                          reset_distance="1.0" server_timeout="5000" />
```

### ClearCostmapAroundRobot

Clears the costmap within a radius around the robot:

```xml
<ClearCostmapAroundRobot server_name="local_costmap/clear_around_local_costmap"
                         reset_distance="2.0" server_timeout="5000" />
```

## Localization Actions

### ReinitializeGlobalLocalization

Scatters AMCL particles uniformly for global relocalization:

```xml
<ReinitializeGlobalLocalization server_name="reinitialize_global_localization"
                                server_timeout="5000" />
```

Use as a last-resort recovery—the robot loses its localization confidence and must re-converge.

## Path Smoothing

### SmoothPath

Applies a smoother plugin to a path:

```xml
<SmoothPath unsmoothed_path="{path}" smoothed_path="{smoothed_path}"
            smoother_id="simple_smoother" server_name="smooth_path"
            server_timeout="5000" error_code_id="{smooth_error}" />
```

Note: you can write the smoothed output back to the same blackboard variable (`smoothed_path="{path}"`) to replace the original path in-place.

## Error Code Propagation

Every Nav2 action node has an `error_code_id` output port. Error codes are defined in `nav2_msgs/action/` for each action type. Common codes:

| Code | Meaning                                          |
|------|--------------------------------------------------|
| 0    | No error (success)                               |
| 100  | Unknown / unspecified                            |
| 101  | TF error                                         |
| 102  | Start/goal occupied                              |
| 103  | Start/goal outside map                           |
| 104  | Timeout                                          |
| 105  | No valid path found                              |
| 106  | Patience exceeded (progress checker)             |

You can use the error code in downstream logic:

```xml
<Sequence>
  <ComputePathToPose goal="{goal}" path="{path}" error_code_id="{plan_error}" />
  <Script code="plan_ok := (plan_error == 0)" />
</Sequence>
```

## Common server_name and server_timeout Defaults

All Nav2 BT action nodes default `server_timeout` to **10000 ms** (10 seconds). The `server_name` defaults match the Nav2 server names (`navigate_to_pose`, `compute_path_to_pose`, `follow_path`, `spin`, `wait`, `backup`, etc.). Override these only if you have renamed your servers or run multiple instances.
