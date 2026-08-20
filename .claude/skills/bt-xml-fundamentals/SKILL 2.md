<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: bt-xml-fundamentals
description: 'Write Nav2 behavior tree XML files. Use when creating BT XML, understanding tick flow, or using BehaviorTree.CPP v4 syntax.'
---

# BehaviorTree.CPP v4 XML Fundamentals

## XML Root Structure

Every BT.CPP v4 XML file begins with the `<root>` element declaring the format version:

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <!-- tree nodes here -->
  </BehaviorTree>
</root>
```

The `BTCPP_format="4"` attribute is **mandatory** in v4. Omitting it or using `"3"` triggers legacy parsing with different port semantics. When multiple `<BehaviorTree>` blocks exist, specify which tree to execute:

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">...</BehaviorTree>
  <BehaviorTree ID="RecoverySubTree">...</BehaviorTree>
</root>
```

If `main_tree_to_execute` is omitted and only one tree exists, it is used by default. With multiple trees and no attribute, the factory throws an error.

## Node Type Categories

BT.CPP v4 defines four fundamental node categories:

| Category    | Children | Return Values              | Purpose                        |
|-------------|----------|----------------------------|--------------------------------|
| Action      | 0 (leaf) | SUCCESS, FAILURE, RUNNING  | Execute work                   |
| Condition   | 0 (leaf) | SUCCESS, FAILURE           | Check state (must NOT return RUNNING) |
| Control     | 1+       | SUCCESS, FAILURE, RUNNING  | Route tick to children         |
| Decorator   | 1        | SUCCESS, FAILURE, RUNNING  | Modify child behavior          |

Condition nodes **must never** return RUNNING. This is a hard contract—violating it causes undefined behavior in reactive control nodes.

## Port System (v4 Key Change)

In v4, ports use **named attributes** instead of positional arguments. Every port has a direction:

```xml
<!-- Input port: data flows INTO the node -->
<MyAction goal="{navigation_goal}" />

<!-- Output port: data flows OUT of the node -->
<ComputePathToPose path="{planned_path}" />

<!-- Bidirectional port: read and write -->
<UpdateCounter counter="{visit_count}" />
```

Port declarations in C++:

```cpp
static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Target pose"),
        BT::OutputPort<nav_msgs::msg::Path>("path", "Computed path"),
        BT::BidirectionalPort<int>("counter", "Visit counter")
    };
}
```

### Blackboard Variable Syntax

Curly braces `{variable_name}` denote blackboard remapping. The variable is resolved at runtime from the shared blackboard:

```xml
<SetBlackboard output_key="target" value="1.0;2.0;0.0" />
<NavigateToPose goal="{target}" />
```

Literal values (no braces) are parsed directly:

```xml
<Wait wait_duration="5.0" />  <!-- literal 5.0 seconds -->
<Wait wait_duration="{timeout}" />  <!-- read from blackboard -->
```

## Script System (v4 Feature)

BT.CPP v4 introduces inline scripting for blackboard manipulation:

```xml
<!-- Post-conditions: execute on specific return status -->
<NavigateToPose goal="{goal}" _onSuccess="visit_count := visit_count + 1"
                              _onFailure="failures := failures + 1" />

<!-- Post-condition running on any completion -->
<MyAction _post="last_action := 'MyAction'" />
```

The `<Script>` node executes arbitrary blackboard expressions:

```xml
<Script code="goal_index := 0; total_goals := 4" />
<Script code="goal_index := (goal_index + 1) % total_goals" />
```

Script expressions support: assignment `:=`, arithmetic `+ - * / %`, comparison `== != < > <= >=`, logical `&& || !`, ternary `? :`, and string literals with single quotes.

## XML Include

Split large trees across files with `<include>`:

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <include path="recovery_subtree.xml" />
  <include path="patrol_subtree.xml" />

  <BehaviorTree ID="MainTree">
    <Sequence>
      <SubTree ID="PatrolWaypoints" />
    </Sequence>
  </BehaviorTree>
</root>
```

The `path` is relative to the file containing the `<include>`. Included files must also have `BTCPP_format="4"` root elements.

## TreeNodesModel for Groot2

Define node metadata so the Groot2 editor displays correct port names and types:

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <CheckBattery min_level="{min_battery}" level="{battery_level}" />
      <NavigateToPose goal="{target_pose}" />
    </Sequence>
  </BehaviorTree>

  <TreeNodesModel>
    <Action ID="CheckBattery">
      <input_port name="min_level" type="double">Minimum battery percentage</input_port>
      <output_port name="level" type="double">Current battery level</output_port>
    </Action>
  </TreeNodesModel>
</root>
```

Without `<TreeNodesModel>`, Groot2 shows ports as generic untyped entries.

## Version 4 vs Version 3 Differences

| Aspect            | v3                              | v4                                  |
|-------------------|---------------------------------|-------------------------------------|
| Port syntax       | Positional or mixed             | Named attributes only               |
| Format attribute  | `BTCPP_format="3"` or omitted  | `BTCPP_format="4"` required         |
| Script support    | None                            | `_onSuccess`, `_onFailure`, `_post`, `<Script>` |
| SubTree ports     | `__autoremap`                   | `_autoremap`                        |
| SequenceStar      | `SequenceStar`                  | Renamed to `SequenceWithMemory`     |

## Minimal Complete Tree

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="PatrolOnce">
  <BehaviorTree ID="PatrolOnce">
    <Sequence>
      <Script code="goal_x := 1.0; goal_y := 2.0" />
      <ComputePathToPose goal="{goal}" path="{path}"
                         planner_id="GridBased" />
      <FollowPath path="{path}" controller_id="FollowPath" />
    </Sequence>
  </BehaviorTree>

  <TreeNodesModel>
    <Action ID="ComputePathToPose">
      <input_port name="goal" type="geometry_msgs::msg::PoseStamped" />
      <input_port name="planner_id" type="std::string" />
      <output_port name="path" type="nav_msgs::msg::Path" />
    </Action>
    <Action ID="FollowPath">
      <input_port name="path" type="nav_msgs::msg::Path" />
      <input_port name="controller_id" type="std::string" />
    </Action>
  </TreeNodesModel>
</root>
```

This tree computes a path to a goal and follows it. The `{path}` blackboard variable connects the planner output to the controller input. The `<TreeNodesModel>` section enables Groot2 visualization with correct port metadata.
