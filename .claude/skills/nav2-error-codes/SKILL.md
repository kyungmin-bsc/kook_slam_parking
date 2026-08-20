<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: nav2-error-codes
description: 'Interpret Nav2 error and result codes. Use when debugging navigation failures, handling planner/controller errors, or recovery logic.'
---

# Nav2 Error Codes

## Overview

Nav2 servers return integer error codes through their action results. These codes propagate from server → BT action node → BT blackboard → user code. Understanding them is essential for building robust recovery strategies.

## Error Code Definitions

### Planner Server Error Codes

| Code | Name | Value | Common Cause |
|------|------|-------|--------------|
| `NONE` | No error | 0 | Success |
| `UNKNOWN` | Unknown error | 1 | Unhandled exception in planner plugin |
| `TF_ERROR` | TF lookup failed | 2 | Transform chain broken (`map→odom→base_link`) |
| `START_OUTSIDE_MAP` | Start pose outside map | 3 | Robot localization is wrong or map is too small |
| `GOAL_OUTSIDE_MAP` | Goal pose outside map | 4 | Goal coordinates outside map bounds |
| `START_OCCUPIED` | Start in lethal costmap cell | 5 | Robot is in collision in the costmap (often stale obstacle) |
| `GOAL_OCCUPIED` | Goal in lethal costmap cell | 6 | Goal is inside an obstacle in the costmap |
| `TIMEOUT` | Planner exceeded time limit | 7 | Map too large, planner too slow, or no path exists |
| `NO_VALID_PATH` | No feasible path found | 8 | Start and goal are in disconnected regions |

### Controller Server Error Codes

| Code | Name | Value | Common Cause |
|------|------|-------|--------------|
| `NONE` | No error | 0 | Success |
| `UNKNOWN` | Unknown error | 1 | Unhandled exception in controller plugin |
| `TF_ERROR` | TF lookup failed | 2 | Odom→base_link transform missing or delayed |
| `INVALID_PATH` | Received invalid path | 3 | Empty path or path in wrong frame |
| `PATIENCE_EXCEEDED` | No progress | 4 | Robot stuck, can't make progress along path |
| `FAILED_TO_MAKE_PROGRESS` | Controller can't proceed | 5 | Obstacle blocking path, controller cycling |
| `NO_VALID_CONTROL` | No valid velocity found | 6 | All sampled trajectories in collision (MPPI) |

### Behavior Server Error Codes

| Code | Name | Value | Common Cause |
|------|------|-------|--------------|
| `NONE` | No error | 0 | Success |
| `UNKNOWN` | Unknown error | 1 | Plugin failure |
| `TF_ERROR` | TF lookup failed | 2 | Transform unavailable |
| `COLLISION_AHEAD` | Would collide | 3 | Spin/backup would hit obstacle |

## Error Flow Through the System

```
Planner/Controller plugin
  → Server action result (error_code integer)
    → BT action node reads result
      → Sets error_code on BT blackboard
        → BT error handling nodes check code
          → Recovery or abort
```

### In the Behavior Tree

BT action nodes expose `error_code_id` to map server error codes to blackboard variables:

```xml
<ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"
                   error_code_id="{compute_path_error_code}"/>

<FollowPath path="{path}" controller_id="FollowPath"
            error_code_id="{follow_path_error_code}"/>
```

The `error_code_names` parameter on the BT Navigator maps error code names to their integer values for BT condition checking:

```yaml
bt_navigator:
  ros__parameters:
    error_code_names:
      - compute_path_error_code
      - follow_path_error_code
```

### Recovery Based on Error Code

In the BT XML, you can check error codes to select appropriate recovery:

```xml
<RecoveryNode number_of_retries="3" name="NavigateRecovery">
  <PipelineSequence name="NavigateWithReplanning">
    <RateController hz="1.0">
      <ComputePathToPose goal="{goal}" path="{path}"
                         error_code_id="{compute_path_error_code}"/>
    </RateController>
    <FollowPath path="{path}" controller_id="FollowPath"
                error_code_id="{follow_path_error_code}"/>
  </PipelineSequence>
  <Sequence name="RecoveryActions">
    <!-- Clear costmaps if path planning failed -->
    <ClearEntireCostmap name="ClearGlobalCostmap"
                        server_timeout="5000" service_name="/global_costmap/clear_entirely_global_costmap"/>
    <ClearEntireCostmap name="ClearLocalCostmap"
                        server_timeout="5000" service_name="/local_costmap/clear_entirely_local_costmap"/>
    <!-- Spin if controller is stuck -->
    <Spin spin_dist="1.57" server_timeout="5000"
          error_code_id="{spin_error_code}"/>
    <!-- Backup if spin fails -->
    <BackUp backup_dist="0.3" backup_speed="0.1"
            error_code_id="{backup_error_code}"/>
    <!-- Wait as last resort -->
    <Wait wait_duration="5"/>
  </Sequence>
</RecoveryNode>
```

## Interpreting Error Codes in Python

```python
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

nav = BasicNavigator()
nav.goToPose(goal)

while not nav.isTaskComplete():
    pass

result = nav.getResult()
if result == TaskResult.FAILED:
    # The error_code is in the action result
    # Access via the underlying action client if needed
    nav.get_logger().error('Navigation failed')
```

For more detailed error information, use the action client directly:

```python
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
goal_handle = client.send_goal(goal_msg)
result = goal_handle.get_result()
error_code = result.result.error_code
# Map integer to meaning using the tables above
```

## Error Code → Recovery Strategy Mapping

| Error Code | Recommended Recovery |
|------------|---------------------|
| `TF_ERROR` | Check TF tree. Is `robot_state_publisher` running? Is odom being published? Wait and retry. |
| `START_OCCUPIED` | Clear costmaps. The robot's position shows as occupied due to stale sensor data. |
| `GOAL_OCCUPIED` | Shift goal slightly. Or clear costmaps if the obstacle is phantom. |
| `NO_VALID_PATH` | Clear costmaps, then replan. If persistent, the goal is truly unreachable (blocked room). |
| `TIMEOUT` | Planner needs more time or the problem is unsolvable. Check map size and planner config. |
| `PATIENCE_EXCEEDED` | Robot is stuck. Spin, backup, clear costmaps, then retry. If persistent, the path is blocked. |
| `NO_VALID_CONTROL` | All trajectories in collision. Backup, clear local costmap, then retry. Check for phantom obstacles near robot. |
| `COLLISION_AHEAD` (behavior) | The recovery action itself would cause collision. Try a different recovery or wait. |

## Common Diagnostic Pattern

When navigation repeatedly fails:

1. Check `error_code` in the action result
2. If `TF_ERROR`: run `ros2 run tf2_tools view_frames` to verify the TF tree
3. If `NO_VALID_PATH` or `START/GOAL_OCCUPIED`: visualize costmaps in RViz, look for phantom obstacles
4. If `PATIENCE_EXCEEDED`: check that the local costmap shows a clear path and the controller parameters allow sufficient velocity
5. If `NO_VALID_CONTROL`: the robot is surrounded by obstacles in the local costmap—check sensor data quality

## Error Codes in Logging

Nav2 servers log error codes at WARN or ERROR level. Search logs for:
```
[controller_server]: Failed to find a valid control. Error code: 6
[planner_server]: No valid path found. Error code: 8
```
