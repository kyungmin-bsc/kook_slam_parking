<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: bt-condition-nodes-nav2
description: 'Use Nav2 BT condition nodes. Use when checking GoalReached, GoalUpdated, IsBatteryLow, TimeExpired, or IsPathValid conditions.'
---

# Nav2 Built-in Condition BT Nodes

Condition nodes are leaf nodes that return **SUCCESS** or **FAILURE** only—never RUNNING. They inspect the world state and are typically placed as the first child of a `ReactiveSequence` or `ReactiveFallback` to gate action execution.

## GoalUpdated

**The most critical condition node for replanning behavior trees.** Returns SUCCESS if the navigation goal has been updated since the last tick. Used in recovery sub-trees to short-circuit recovery when a new goal arrives.

```xml
<GoalUpdated />
```

No ports. Reads from the internal goal tracking in the BT navigator. This node is essential inside recovery fallbacks:

```xml
<ReactiveFallback>
  <GoalUpdated />          <!-- Skip recovery if goal changed -->
  <Sequence>
    <ClearEntireCostmap server_name="global_costmap/clear_entirely_global_costmap" />
    <Spin spin_dist="1.57" />
  </Sequence>
</ReactiveFallback>
```

When `GoalUpdated` returns SUCCESS, the ReactiveFallback immediately returns SUCCESS without executing any recovery actions, allowing the main navigation branch to re-plan with the new goal.

## GoalReached

Returns SUCCESS if the robot is within tolerance of the current goal. Uses the goal checker plugin's tolerance values:

```xml
<GoalReached goal="{goal}" server_name="bt_navigator" server_timeout="10000" />
```

| Port          | Direction | Type        | Description              |
|---------------|-----------|-------------|--------------------------|
| goal          | input     | PoseStamped | Goal to check against    |
| server_name   | input     | string      | Navigator server         |
| server_timeout| input     | int (ms)    | Timeout                  |

Typically used to terminate navigation loops early.

## IsBatteryLow

Subscribes to a `sensor_msgs/msg/BatteryState` topic and returns SUCCESS when the battery percentage is below a threshold:

```xml
<IsBatteryLow battery_topic="/battery_state" min_battery="0.15"
              is_voltage="false" />
```

| Port           | Direction | Type   | Description                                      |
|----------------|-----------|--------|--------------------------------------------------|
| battery_topic  | input     | string | Topic publishing BatteryState                    |
| min_battery    | input     | double | Threshold (0.0–1.0 for percentage, or voltage)   |
| is_voltage     | input     | bool   | If true, compare against voltage instead of %    |

Returns SUCCESS when battery is LOW (below threshold). This is a condition to trigger charging behavior:

```xml
<ReactiveSequence>
  <IsBatteryLow battery_topic="/battery_state" min_battery="0.15" is_voltage="false" />
  <NavigateToPose goal="{charger_pose}" />  <!-- Go charge -->
</ReactiveSequence>
```

**Note the inversion**: SUCCESS means the battery IS low. To use it as a "battery OK" guard, wrap it in an Inverter:

```xml
<ReactiveSequence>
  <Inverter>
    <IsBatteryLow battery_topic="/battery_state" min_battery="0.15" />
  </Inverter>
  <!-- Continue normal operation only if battery is NOT low -->
  <NavigateToPose goal="{patrol_goal}" />
</ReactiveSequence>
```

## IsPathValid

Checks if a stored path is still traversable by querying the costmap. Returns FAILURE if any point on the path is now in collision:

```xml
<IsPathValid path="{path}" server_name="compute_path_to_pose"
             server_timeout="5000" />
```

| Port          | Direction | Type | Description                    |
|---------------|-----------|------|--------------------------------|
| path          | input     | Path | Path to validate               |
| server_name   | input     | string | Planner server for validation |
| server_timeout| input     | int  | Timeout                        |

Used in a ReactiveSequence to detect when obstacles invalidate the current path, triggering replanning:

```xml
<ReactiveSequence>
  <IsPathValid path="{path}" />
  <FollowPath path="{path}" controller_id="FollowPath" />
</ReactiveSequence>
```

If an obstacle appears on the path, `IsPathValid` returns FAILURE, halting `FollowPath` and propagating failure up the tree where a recovery or replanning branch handles it.

## DistanceTraveled

Returns SUCCESS when the robot has moved at least `distance` meters since this node was last reset. Used to trigger periodic replanning:

```xml
<DistanceTraveled distance="1.0" global_frame="map" robot_base_frame="base_link" />
```

| Port              | Direction | Type   | Description                  |
|-------------------|-----------|--------|------------------------------|
| distance          | input     | double | Distance threshold in meters |
| global_frame      | input     | string | Global reference frame       |
| robot_base_frame  | input     | string | Robot base frame             |

Common pattern—replan every 2 meters:

```xml
<ReactiveSequence>
  <DistanceTraveled distance="2.0" global_frame="map" robot_base_frame="base_link" />
  <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" />
</ReactiveSequence>
```

## TimeExpired

Returns SUCCESS when `seconds` have elapsed since this node was first ticked:

```xml
<TimeExpired seconds="10.0" />
```

Use for periodic operations. Note: the timer resets when the parent node resets this child (e.g., after the enclosing Sequence restarts).

## TransformAvailable

Returns SUCCESS if a TF transform between two frames is available:

```xml
<TransformAvailable child_frame="base_link" parent_frame="map" />
```

Use in startup sequences to wait for localization to initialize before attempting navigation.

## InitialPoseReceived

Returns SUCCESS once an initial pose has been set (e.g., via RViz or the `/initialpose` topic). Useful for gating navigation until localization is bootstrapped:

```xml
<Sequence>
  <InitialPoseReceived />
  <SubTree ID="PatrolTree" />
</Sequence>
```

## IsBatteryCharging

Returns SUCCESS when the robot is currently charging. Subscribes to the same `BatteryState` topic:

```xml
<IsBatteryCharging battery_topic="/battery_state" />
```

Use to hold the robot at the charging station:

```xml
<ReactiveSequence>
  <IsBatteryCharging battery_topic="/battery_state" />
  <Wait wait_duration="60.0" />
</ReactiveSequence>
```

## Custom Condition Patterns

### GlobalUpdatedGoal

A custom condition (not built-in) that detects if the global goal has changed from an external replanning request versus user input. Implement by comparing the current goal blackboard entry against a cached value:

```xml
<!-- Pattern: detect goal change for replanning -->
<ReactiveSequence>
  <Inverter>
    <GoalUpdated />  <!-- Only replan if goal has NOT changed -->
  </Inverter>
  <DistanceTraveled distance="5.0" global_frame="map" robot_base_frame="base_link" />
  <ComputePathToPose goal="{goal}" path="{path}" />
</ReactiveSequence>
```

### PathExpiringTimer

A custom condition that tracks path age and triggers replanning:

```xml
<!-- Pattern: replan if path is older than N seconds -->
<ReactiveSequence>
  <TimeExpired seconds="30.0" />  <!-- Path "expires" after 30s -->
  <ComputePathToPose goal="{goal}" path="{path}" />
</ReactiveSequence>
```

## Usage Pattern Summary

The typical condition placement in Nav2 trees follows this hierarchy:

```xml
<ReactiveFallback name="TopLevelRecovery">
  <GoalUpdated />          <!-- Highest priority: new goal cancels recovery -->
  <RecoveryActions />
</ReactiveFallback>

<ReactiveSequence name="NavigateWithMonitoring">
  <Inverter>
    <IsBatteryLow min_battery="0.10" />
  </Inverter>                              <!-- Abort if battery critical -->
  <IsPathValid path="{path}" />            <!-- Abort if path blocked -->
  <FollowPath path="{path}" />             <!-- Execute only when safe -->
</ReactiveSequence>
```

Conditions placed earlier in a ReactiveSequence have **higher priority**—they are checked first on every tick and can preempt later children.
