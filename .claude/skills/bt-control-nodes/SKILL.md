<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: bt-control-nodes
description: 'Use BT control flow nodes. Use when choosing between Sequence, Fallback, Parallel, ReactiveFallback, ReactiveSequence, or RoundRobin.'
---

# BT Control Nodes — Execution Flow in Detail

## Sequence

Ticks children left-to-right. Returns FAILURE on the first child failure. Returns SUCCESS only when **all** children succeed. On the next tick after returning SUCCESS or FAILURE, it **restarts from the first child**.

```xml
<Sequence>
  <CheckDoorOpen />      <!-- Condition: SUCCESS/FAILURE -->
  <WalkThroughDoor />    <!-- Action: SUCCESS/FAILURE/RUNNING -->
  <CloseDoor />          <!-- Action: SUCCESS/FAILURE/RUNNING -->
</Sequence>
```

**Execution trace:**

| Tick | Child 1     | Child 2          | Child 3    | Sequence Returns |
|------|-------------|------------------|------------|------------------|
| 1    | SUCCESS     | RUNNING          | —          | RUNNING          |
| 2    | SUCCESS     | SUCCESS          | RUNNING    | RUNNING          |
| 3    | SUCCESS     | SUCCESS          | SUCCESS    | SUCCESS          |
| 4    | SUCCESS     | RUNNING          | —          | RUNNING (restart)|

On tick 4, the Sequence restarts from child 1 because it completed on tick 3.

## ReactiveSequence

Like Sequence but **always re-ticks from the first child** on every tick, even when a later child is RUNNING. This is the workhorse for condition monitoring.

```xml
<ReactiveSequence>
  <IsPathValid path="{path}" />     <!-- Re-checked EVERY tick -->
  <FollowPath path="{path}" />      <!-- Continues RUNNING -->
</ReactiveSequence>
```

**Execution trace:**

| Tick | Child 1 (Condition) | Child 2 (Action) | Returns |
|------|---------------------|-------------------|---------|
| 1    | SUCCESS             | RUNNING           | RUNNING |
| 2    | SUCCESS             | RUNNING           | RUNNING |
| 3    | FAILURE (path invalid) | — (halted)     | FAILURE |

On tick 3, IsPathValid fails. The ReactiveSequence **halts** the RUNNING FollowPath child and immediately returns FAILURE. This is how you implement "keep checking a condition while an action runs."

### Critical Mistake: Sequence vs ReactiveSequence

```xml
<!-- WRONG: Condition only checked once at the start -->
<Sequence>
  <IsBatteryOk />
  <NavigateToPose goal="{goal}" />
</Sequence>

<!-- RIGHT: Battery is monitored continuously during navigation -->
<ReactiveSequence>
  <IsBatteryOk />
  <NavigateToPose goal="{goal}" />
</ReactiveSequence>
```

With plain `Sequence`, once `IsBatteryOk` returns SUCCESS, it is never re-evaluated until the entire Sequence completes and restarts. The robot could drain its battery mid-navigation without noticing.

## SequenceWithMemory (formerly SequenceStar)

Remembers which child was RUNNING and resumes from there. Does **not** re-tick already-succeeded children.

```xml
<SequenceWithMemory>
  <NavigateToPose goal="{pose_A}" />
  <NavigateToPose goal="{pose_B}" />
  <NavigateToPose goal="{pose_C}" />
</SequenceWithMemory>
```

**Execution trace:**

| Tick | Child 1   | Child 2   | Child 3 | Returns |
|------|-----------|-----------|---------|---------|
| 1    | RUNNING   | —         | —       | RUNNING |
| 2    | SUCCESS   | RUNNING   | —       | RUNNING |
| 3    | (skipped) | RUNNING   | —       | RUNNING |
| 4    | (skipped) | SUCCESS   | RUNNING | RUNNING |
| 5    | (skipped) | (skipped) | SUCCESS | SUCCESS |

Child 1 is never re-ticked after succeeding. Use this for sequential goal visiting where you don't want to revisit completed goals.

## Fallback (Selector)

Tries children until one succeeds. Returns SUCCESS on the first child success. Returns FAILURE only when **all** children fail. Restarts from child 1 on next tick.

```xml
<Fallback>
  <NavigateWithPlanner planner_id="GridBased" />
  <NavigateWithPlanner planner_id="SmacHybrid" />
  <Spin spin_dist="1.57" />
</Fallback>
```

**Execution trace:**

| Tick | Child 1    | Child 2  | Child 3 | Returns |
|------|------------|----------|---------|---------|
| 1    | RUNNING    | —        | —       | RUNNING |
| 2    | FAILURE    | RUNNING  | —       | RUNNING |
| 3    | (restart)  | (restart)| —       | depends |

## ReactiveFallback

Re-ticks from the first child every tick. Used to prioritize higher-priority alternatives:

```xml
<ReactiveFallback>
  <IsGoalReached />           <!-- Highest priority: are we done? -->
  <FollowPath path="{path}" />
</ReactiveFallback>
```

If `IsGoalReached` returns SUCCESS at any point, `FollowPath` is halted immediately.

## Parallel

Ticks **all** children simultaneously. Completes based on thresholds:

```xml
<Parallel success_count="2" failure_count="1">
  <MonitorBattery />
  <NavigateToPose goal="{goal}" />
  <RecordVideo />
</Parallel>
```

- Returns SUCCESS when `success_count` children have succeeded (here: 2)
- Returns FAILURE when `failure_count` children have failed (here: 1)
- Returns RUNNING otherwise
- When threshold is `-1`, it means "all children"

**Execution trace** (success_count=2, failure_count=1):

| Tick | Child 1   | Child 2   | Child 3   | Returns |
|------|-----------|-----------|-----------|---------|
| 1    | SUCCESS   | RUNNING   | RUNNING   | RUNNING (1 success, need 2) |
| 2    | SUCCESS   | RUNNING   | SUCCESS   | SUCCESS (2 successes reached) |

## IfThenElse

Three children: condition, then-branch, else-branch.

```xml
<IfThenElse>
  <IsBatteryLow />                  <!-- condition -->
  <NavigateToPose goal="{charger}" />  <!-- then: battery IS low -->
  <NavigateToPose goal="{patrol}" />   <!-- else: battery is fine -->
</IfThenElse>
```

On each tick: evaluates child 1. If SUCCESS, ticks child 2. If FAILURE, ticks child 3. If the selected branch returns RUNNING, the condition is **not** re-evaluated on the next tick (the running branch continues).

## WhileDoElse

Like IfThenElse but the condition is re-evaluated every tick while the "do" branch is RUNNING:

```xml
<WhileDoElse>
  <IsPathValid path="{path}" />     <!-- condition: re-checked each tick -->
  <FollowPath path="{path}" />      <!-- do: execute while condition holds -->
  <ComputePathToPose goal="{goal}" path="{path}" />  <!-- else: replan -->
</WhileDoElse>
```

If the condition flips to FAILURE while `FollowPath` is RUNNING, the action is halted and the else-branch is ticked.

## Switch

Multi-branch selection based on a variable:

```xml
<Switch4 variable="{nav_mode}" case_1="fast" case_2="careful" case_3="patrol" case_4="dock">
  <FollowPath controller_id="HighSpeed" path="{path}" />
  <FollowPath controller_id="Regulated" path="{path}" />
  <SubTree ID="PatrolLoop" />
  <NavigateToPose goal="{dock_pose}" />
  <Wait wait_duration="1.0" />  <!-- default case -->
</Switch4>
```

Switch2 through Switch6 are available. The last child is the default case when no case matches. The variable is compared as a string against each case value.

## Choosing the Right Control Node

| Scenario                                    | Use                   |
|---------------------------------------------|-----------------------|
| Do A then B then C in order                 | Sequence              |
| Do A then B, but keep checking condition X  | ReactiveSequence      |
| Visit waypoints, don't revisit completed    | SequenceWithMemory    |
| Try A, if it fails try B, then C            | Fallback              |
| Continuously check if done, otherwise act   | ReactiveFallback      |
| Run monitoring alongside navigation         | Parallel              |
| Branch based on a condition                 | IfThenElse            |
| Branch with continuous condition checking   | WhileDoElse           |
