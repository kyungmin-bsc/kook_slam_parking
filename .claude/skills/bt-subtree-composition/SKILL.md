<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: bt-subtree-composition
description: 'Compose BT subtrees. Use when breaking large behavior trees into reusable SubTree includes.'
---

# BT Sub-Tree Composition

## Why Compose Sub-Trees

Large behavior trees become unmaintainable as monolithic XML files. Sub-tree composition enables:

- **Reuse**: A "navigate with recovery" sub-tree used by patrol, docking, and exploration trees
- **Testing**: Individual sub-trees can be tested in isolation
- **Team collaboration**: Different engineers work on different sub-trees
- **Readability**: Top-level tree reads like pseudocode when sub-trees have descriptive names

## Multiple Trees in One File

A single XML file can contain multiple `<BehaviorTree>` blocks. The `SubTree` node references them by ID:

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">

  <BehaviorTree ID="MainTree">
    <Sequence>
      <SubTree ID="Initialize" />
      <SubTree ID="PatrolLoop" waypoints="{patrol_waypoints}" />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="Initialize">
    <Sequence>
      <Script code="patrol_cycle := 0; failures := 0" />
      <TransformAvailable child_frame="base_link" parent_frame="map" />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="PatrolLoop">
    <KeepRunningUntilFailure>
      <SequenceWithMemory>
        <SubTree ID="NavigateWithRecovery" target="{wp1}" />
        <SubTree ID="NavigateWithRecovery" target="{wp2}" />
        <SubTree ID="NavigateWithRecovery" target="{wp3}" />
      </SequenceWithMemory>
    </KeepRunningUntilFailure>
  </BehaviorTree>

  <BehaviorTree ID="NavigateWithRecovery">
    <RecoveryNode number_of_retries="3">
      <Sequence>
        <ComputePathToPose goal="{target}" path="{path}" planner_id="GridBased" />
        <FollowPath path="{path}" controller_id="FollowPath" />
      </Sequence>
      <Sequence>
        <ClearEntireCostmap server_name="global_costmap/clear_entirely_global_costmap" />
        <Wait wait_duration="2.0" server_name="wait" />
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>

</root>
```

## External File Inclusion

Split trees across files with `<include>`:

```
bt_trees/
├── main_patrol.xml          # Top-level tree
├── navigate_w_recovery.xml  # Reusable navigation sub-tree
├── recovery_actions.xml     # Recovery strategies
└── anomaly_handler.xml      # Intruder response sub-tree
```

### main_patrol.xml

```xml
<root BTCPP_format="4" main_tree_to_execute="HousePatrol">
  <include path="navigate_w_recovery.xml" />
  <include path="recovery_actions.xml" />
  <include path="anomaly_handler.xml" />

  <BehaviorTree ID="HousePatrol">
    <ReactiveSequence>
      <Inverter>
        <IsBatteryLow battery_topic="/battery_state" min_battery="0.15" />
      </Inverter>
      <KeepRunningUntilFailure>
        <SequenceWithMemory>
          <SubTree ID="NavigateWithRecovery" target="{room_1}" />
          <SubTree ID="NavigateWithRecovery" target="{room_2}" />
          <SubTree ID="NavigateWithRecovery" target="{room_3}" />
        </SequenceWithMemory>
      </KeepRunningUntilFailure>
    </ReactiveSequence>
  </BehaviorTree>
</root>
```

### navigate_w_recovery.xml

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="NavigateWithRecovery">
    <RecoveryNode number_of_retries="3">
      <Sequence>
        <ComputePathToPose goal="{target}" path="{path}" planner_id="GridBased"
                           error_code_id="{plan_error}" />
        <FollowPath path="{path}" controller_id="FollowPath"
                    error_code_id="{follow_error}" />
      </Sequence>
      <SubTree ID="RecoveryActions" />
    </RecoveryNode>
  </BehaviorTree>
</root>
```

### recovery_actions.xml

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="RecoveryActions">
    <ReactiveFallback>
      <GoalUpdated />
      <RoundRobin>
        <Sequence>
          <ClearEntireCostmap
            server_name="global_costmap/clear_entirely_global_costmap" />
          <ClearEntireCostmap
            server_name="local_costmap/clear_entirely_local_costmap" />
        </Sequence>
        <Spin spin_dist="1.57" server_name="spin" />
        <Wait wait_duration="5.0" server_name="wait" />
        <BackUp backup_dist="0.3" backup_speed="0.15" server_name="backup" />
      </RoundRobin>
    </ReactiveFallback>
  </BehaviorTree>
</root>
```

The `path` in `<include>` is relative to the file containing the include directive.

## Port Remapping Between Parent and Sub-Tree

Explicit port remapping creates clean interfaces between trees:

```xml
<!-- Parent: passes specific data into/out of sub-tree -->
<SubTree ID="NavigateWithRecovery"
         target="{kitchen_pose}"
         result_error="{kitchen_error}"
         max_retries="3" />
```

Inside the sub-tree, `{target}`, `{result_error}`, and `{max_retries}` refer to the remapped variables. The sub-tree doesn't know or care that the parent calls them `kitchen_pose` and `kitchen_error`.

### Remapping Rules

| Sub-tree uses   | Parent provides           | Effect                                |
|-----------------|---------------------------|---------------------------------------|
| `{target}`      | `target="{kitchen_pose}"` | Sub-tree reads parent's kitchen_pose  |
| `{result}`      | `result="{nav_result}"`   | Sub-tree writes to parent's nav_result|
| `{speed}`       | `speed="0.5"`             | Literal value, not blackboard entry   |
| `{path}`        | *(not mapped)*            | Sub-tree-local variable               |

Unmapped variables (like `{path}` above) are local to the sub-tree's blackboard scope if the sub-tree uses its own scope. With `_autoremap="true"`, they map to the parent's blackboard.

## How Nav2's Default Trees Are Structured

Nav2 ships several default behavior trees in `nav2_bt_navigator/behavior_trees/`:

| File                                    | Purpose                                     |
|-----------------------------------------|---------------------------------------------|
| `navigate_to_pose_w_replanning_and_recovery.xml` | Default single-goal navigation     |
| `navigate_through_poses_w_replanning_and_recovery.xml` | Multi-goal navigation       |
| `navigate_to_pose_w_replanning_goal_patience_and_recovery.xml` | With goal patience |

The default tree structure follows this pattern:

```
RecoveryNode (outer: num_retries=6)
├── PipelineSequence (navigate with replanning)
│   ├── RateController (1 Hz)
│   │   └── RecoveryNode (inner: plan recovery)
│   │       ├── ComputePathToPose
│   │       └── ClearGlobalCostmap + Replan
│   └── RecoveryNode (inner: follow recovery)
│       ├── FollowPath
│       └── ClearLocalCostmap + Re-follow
└── ReactiveFallback (recovery)
    ├── GoalUpdated
    └── RoundRobin
        ├── ClearCostmaps
        ├── Spin
        ├── Wait
        └── BackUp
```

## Building a Reusable Library

Organize sub-trees by functionality:

```
bt_library/
├── navigation/
│   ├── navigate_to_pose_w_recovery.xml
│   ├── navigate_through_poses_w_recovery.xml
│   └── dock_to_charger.xml
├── recovery/
│   ├── standard_recovery.xml
│   ├── aggressive_recovery.xml
│   └── relocalization_recovery.xml
├── patrol/
│   ├── waypoint_patrol.xml
│   └── perimeter_patrol.xml
├── tasks/
│   ├── capture_panorama.xml
│   ├── check_sensor.xml
│   └── send_alert.xml
└── main_trees/
    ├── house_patrol.xml
    └── security_patrol.xml
```

### Composing from the Library

```xml
<root BTCPP_format="4" main_tree_to_execute="SecurityPatrol">
  <include path="../navigation/navigate_to_pose_w_recovery.xml" />
  <include path="../recovery/standard_recovery.xml" />
  <include path="../tasks/capture_panorama.xml" />
  <include path="../tasks/send_alert.xml" />

  <BehaviorTree ID="SecurityPatrol">
    <KeepRunningUntilFailure>
      <Sequence>
        <SubTree ID="NavigateWithRecovery" target="{next_waypoint}" />
        <SubTree ID="CapturePanorama" output_dir="/patrol_images/" />
        <Script code="waypoint_idx := (waypoint_idx + 1) % num_waypoints" />
      </Sequence>
    </KeepRunningUntilFailure>
  </BehaviorTree>
</root>
```

## Configuring bt_navigator to Use Custom Trees

Point Nav2 to your custom tree XML:

```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "/path/to/bt_library/main_trees/house_patrol.xml"
    default_nav_through_poses_bt_xml: "/path/to/bt_library/main_trees/multi_goal.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_rate_controller_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_bt_node
      # Custom nodes:
      - my_intruder_detection_bt_node
      - my_notification_bt_node
```

## Testing Sub-Trees in Isolation

Test individual sub-trees by wrapping them in a minimal main tree:

```xml
<root BTCPP_format="4" main_tree_to_execute="TestRecovery">
  <include path="recovery_actions.xml" />

  <BehaviorTree ID="TestRecovery">
    <Sequence>
      <!-- Set up test conditions -->
      <Script code="test_goal_updated := false" />
      <!-- Run the sub-tree under test -->
      <SubTree ID="RecoveryActions" />
    </Sequence>
  </BehaviorTree>
</root>
```

This allows validating recovery logic without running the full navigation stack.
