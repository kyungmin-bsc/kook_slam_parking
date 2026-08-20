<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: bt-blackboard-patterns
description: 'Use BT blackboard for data sharing. Use when passing goals, paths, or state between BT nodes via the blackboard.'
---

# Blackboard Patterns and Data Flow

## Port Remapping Syntax

The `{variable}` syntax connects node ports to blackboard entries. When a port value is enclosed in curly braces, the BT engine reads from or writes to that blackboard key:

```xml
<!-- Writing: ComputePathToPose writes its result to blackboard key "my_path" -->
<ComputePathToPose goal="{goal}" path="{my_path}" />

<!-- Reading: FollowPath reads from blackboard key "my_path" -->
<FollowPath path="{my_path}" />
```

Without braces, the value is treated as a literal:

```xml
<Wait wait_duration="5.0" />       <!-- literal 5.0 -->
<Wait wait_duration="{timeout}" /> <!-- reads blackboard key "timeout" -->
```

## Default Values in Port Declarations

Ports can specify defaults in C++ that apply when the XML attribute is omitted:

```cpp
static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("speed", 0.5, "Movement speed in m/s"),
        BT::InputPort<std::string>("planner_id", "GridBased", "Planner plugin"),
    };
}
```

```xml
<!-- Uses default speed=0.5 and planner_id="GridBased" -->
<MyAction goal="{goal}" />

<!-- Overrides speed, uses default planner_id -->
<MyAction goal="{goal}" speed="1.0" />

<!-- Reads speed from blackboard -->
<MyAction goal="{goal}" speed="{configured_speed}" />
```

## SetBlackboard Node

Sets a blackboard variable from XML. Only supports string values that are converted to the appropriate type when read:

```xml
<SetBlackboard output_key="goal_count" value="4" />
<SetBlackboard output_key="patrol_active" value="true" />
<SetBlackboard output_key="current_room" value="kitchen" />
```

For geometry types, the string format depends on the registered converter. Nav2 registers converters for `PoseStamped` using semicolon-separated values:

```xml
<!-- x;y;z;qx;qy;qz;qw -->
<SetBlackboard output_key="charger_pose" value="1.0;2.0;0.0;0.0;0.0;0.0;1.0" />
```

## Script Node for Inline Computation

BT.CPP v4's `<Script>` node is more powerful than `SetBlackboard` for arithmetic and logic:

```xml
<!-- Simple assignment -->
<Script code="retry_count := 0" />

<!-- Arithmetic -->
<Script code="goal_index := (goal_index + 1) % total_goals" />

<!-- Multiple assignments (semicolon-separated) -->
<Script code="x := 1.0; y := 2.0; heading := 0.0" />

<!-- Conditional assignment -->
<Script code="speed := (is_narrow_passage == true) ? 0.2 : 0.5" />

<!-- String assignment (single quotes) -->
<Script code="current_room := 'living_room'" />
```

### Script Operators

| Operator      | Syntax               | Example                          |
|---------------|----------------------|----------------------------------|
| Assignment    | `:=`                 | `x := 5`                        |
| Arithmetic    | `+ - * / %`          | `count := count + 1`            |
| Comparison    | `== != < > <= >=`    | `is_close := (dist < 1.0)`      |
| Logical       | `&& || !`             | `ok := (a > 0) && (b > 0)`     |
| Ternary       | `? :`                | `v := (x > 0) ? x : -x`        |
| String        | Single quotes        | `name := 'robot1'`              |

**Note**: The assignment operator is `:=`, not `=`. Using `=` inside a Script node is a syntax error.

## Blackboard Scoping in Sub-Trees

By default, sub-trees share the parent's blackboard. Port remapping controls what data flows between parent and sub-tree:

```xml
<!-- Parent tree -->
<BehaviorTree ID="ParentTree">
  <Sequence>
    <SetBlackboard output_key="room_goal" value="3.0;1.0;0.0;0.0;0.0;0.0;1.0" />
    <SubTree ID="NavigateWithRecovery"
             target_pose="{room_goal}"
             nav_result="{room_nav_result}" />
  </Sequence>
</BehaviorTree>

<!-- Sub-tree -->
<BehaviorTree ID="NavigateWithRecovery">
  <Sequence>
    <!-- {target_pose} is remapped from parent's {room_goal} -->
    <ComputePathToPose goal="{target_pose}" path="{path}" />
    <FollowPath path="{path}" />
    <!-- {nav_result} is remapped to parent's {room_nav_result} -->
    <Script code="nav_result := 'success'" />
  </Sequence>
</BehaviorTree>
```

The sub-tree's `target_pose` reads from the parent's `room_goal`, and the sub-tree's `nav_result` writes to the parent's `room_nav_result`. The `path` variable is local to the sub-tree since it is not remapped.

### Port Declaration in SubTree

The sub-tree XML implicitly declares ports based on the `{...}` variables used inside it. The parent's `<SubTree>` element provides the mapping:

```xml
<!-- Each attribute on SubTree maps a sub-tree port to a parent blackboard entry -->
<SubTree ID="NavigateWithRecovery"
         target_pose="{room_goal}"      <!-- input: parent → subtree -->
         nav_result="{room_nav_result}" <!-- output: subtree → parent -->
/>
```

### The _autoremap Attribute

For sub-trees that should transparently share the parent's entire blackboard:

```xml
<SubTree ID="RecoverySubTree" _autoremap="true" />
```

With `_autoremap="true"`, every `{variable}` in the sub-tree automatically maps to the same-named variable in the parent blackboard. This is convenient but reduces encapsulation—changes in the sub-tree can accidentally overwrite parent variables.

**Recommendation**: Use explicit port remapping for production trees. Reserve `_autoremap` for debugging and rapid prototyping.

## Typing Rules

Blackboard variables are dynamically typed, but once a type is established by the first write, subsequent reads expect the same type:

```xml
<!-- First write establishes type as double -->
<Script code="speed := 0.5" />

<!-- This works: reading as double -->
<FollowPath path="{path}" speed="{speed}" />

<!-- This would fail at runtime: type mismatch if the node expects int -->
```

### Common Type Pitfalls

**Pitfall 1: Two nodes write the same variable with different types**

```xml
<!-- Node A writes path as nav_msgs::msg::Path -->
<ComputePathToPose path="{shared_path}" />

<!-- Node B tries to write path as std::string — RUNTIME ERROR -->
<SetBlackboard output_key="shared_path" value="some_string" />
```

**Pitfall 2: Reading before writing**

```xml
<!-- RUNTIME ERROR if "goal" was never set -->
<NavigateToPose goal="{goal}" />
```

Fix with a default value in the port declaration or an initialization Script:

```xml
<Sequence>
  <Script code="goal := default_goal" />  <!-- Ensure variable exists -->
  <NavigateToPose goal="{goal}" />
</Sequence>
```

## Passing Data Between Unrelated Sub-Trees

Sub-trees that run sequentially can pass data through the parent blackboard:

```xml
<Sequence>
  <!-- Sub-tree 1: detects an object, outputs its pose -->
  <SubTree ID="DetectObject"
           detected_pose="{object_pose}"
           detection_confidence="{confidence}" />

  <!-- Script: decide whether to approach based on confidence -->
  <Script code="should_approach := (confidence > 0.8)" />

  <!-- Sub-tree 2: navigates to the detected pose -->
  <SubTree ID="ApproachObject"
           target="{object_pose}"
           approach_enabled="{should_approach}" />
</Sequence>
```

The parent blackboard acts as the data bus. `{object_pose}` is written by DetectObject and read by ApproachObject. The Script node performs intermediate logic.

## Debugging Blackboard State

Enable BT logging to see blackboard reads/writes in real-time:

```yaml
bt_navigator:
  ros__parameters:
    enable_groot_monitoring: true
```

In Groot2, the blackboard panel shows all current variable values and their types during replay. For command-line debugging, subscribe to the `/behavior_tree_log` topic or enable `FileLogger2` (see `bt_logging_and_replay.md`).

## Pattern: Blackboard as Configuration Store

Load navigation parameters into the blackboard at tree startup:

```xml
<Sequence name="ConfigureAndPatrol">
  <!-- Load configuration -->
  <Script code="
    max_speed := 0.5;
    min_battery := 0.15;
    replan_hz := 1.0;
    recovery_retries := 3
  " />

  <!-- Use configuration in sub-trees via remapping -->
  <SubTree ID="PatrolLoop"
           speed_limit="{max_speed}"
           battery_threshold="{min_battery}"
           replanning_rate="{replan_hz}"
           max_retries="{recovery_retries}" />
</Sequence>
```

This centralizes tunable parameters at the tree root, making them easy to find and adjust.
