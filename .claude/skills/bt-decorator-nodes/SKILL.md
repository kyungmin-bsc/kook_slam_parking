<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: bt-decorator-nodes
description: 'Use BT decorator nodes. Use when adding RateController, SpeedController, RecoveryNode, SingleChild, or timeout decorators.'
---

# BT Decorator Nodes

Decorators wrap exactly one child node and modify its tick behavior, return status, or execution frequency. They are essential for throttling expensive operations, retrying failures, and implementing timeouts.

## RateController

Limits child tick frequency to a maximum rate in Hz. Between ticks, returns the child's last status. Use to throttle expensive computations like path planning:

```xml
<RateController hz="1.0">
  <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" />
</RateController>
```

The child is ticked at most once per second. If the parent ticks this decorator at 10 Hz, the planner runs at 1 Hz while the decorator returns RUNNING (or the last result) for the intervening ticks.

**Navigation example** — replan at 0.5 Hz while following:

```xml
<ReactiveSequence>
  <RateController hz="0.5">
    <ComputePathToPose goal="{goal}" path="{path}" />
  </RateController>
  <FollowPath path="{path}" controller_id="FollowPath" />
</ReactiveSequence>
```

**Warning**: If `hz` is too low and the environment changes rapidly, the robot may follow stale paths. If too high, the planner overloads the CPU.

## DistanceController

Only ticks the child when the robot has traveled at least `distance` meters since the last tick. Returns RUNNING otherwise:

```xml
<DistanceController distance="2.0" global_frame="map" robot_base_frame="base_link">
  <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" />
</DistanceController>
```

Useful in environments where time-based replanning wastes resources while the robot is stationary (e.g., waiting at a door).

## SpeedController

Adjusts tick rate based on the robot's current speed. Faster movement → more frequent ticks. Parameters define the mapping:

```xml
<SpeedController min_rate="0.5" max_rate="5.0" min_speed="0.0" max_speed="1.0"
                 filter_coeff="0.1">
  <ComputePathToPose goal="{goal}" path="{path}" />
</SpeedController>
```

| Port          | Type   | Description                              |
|---------------|--------|------------------------------------------|
| min_rate      | double | Tick rate (Hz) when speed ≤ min_speed    |
| max_rate      | double | Tick rate (Hz) when speed ≥ max_speed    |
| min_speed     | double | Speed below which min_rate applies       |
| max_speed     | double | Speed above which max_rate applies       |
| filter_coeff  | double | Low-pass filter on speed (0–1)           |

The rate scales linearly between the bounds. At high speed, the robot replans more often to react to close obstacles.

## SingleTrigger

Ticks the child exactly once, then returns the cached result on all subsequent ticks:

```xml
<SingleTrigger>
  <ReinitializeGlobalLocalization />
</SingleTrigger>
```

Use for one-shot initialization actions that should not repeat. After the child returns SUCCESS or FAILURE, that result is returned on every future tick without re-executing the child.

## Inverter

Flips SUCCESS to FAILURE and FAILURE to SUCCESS. RUNNING passes through unchanged:

```xml
<Inverter>
  <IsBatteryLow battery_topic="/battery_state" min_battery="0.15" />
</Inverter>
<!-- Returns SUCCESS when battery is NOT low (i.e., battery is fine) -->
```

Essential for negating condition nodes. `IsBatteryLow` returns SUCCESS when the battery IS low. Wrapping it in `Inverter` gives a "battery OK" condition.

## ForceSuccess

Returns SUCCESS regardless of the child's result (SUCCESS or FAILURE). RUNNING passes through:

```xml
<ForceSuccess>
  <ClearEntireCostmap server_name="global_costmap/clear_entirely_global_costmap" />
</ForceSuccess>
```

Use when a recovery action is "best effort"—you don't want its failure to propagate up and abort the recovery sequence.

## ForceFailure

Returns FAILURE regardless of the child's result. RUNNING passes through:

```xml
<ForceFailure>
  <Wait wait_duration="0.5" />
</ForceFailure>
```

Useful in Fallback nodes where you want a child to always "fail" to ensure the next fallback option runs.

## Repeat

Ticks the child N times. Returns SUCCESS after all repetitions complete. Returns FAILURE immediately if the child fails:

```xml
<Repeat num_cycles="3">
  <Spin spin_dist="1.57" />
</Repeat>
```

The robot spins three times (total ~4.71 radians). If any single spin fails, the Repeat immediately returns FAILURE.

**Patrol N laps:**

```xml
<Repeat num_cycles="10">
  <Sequence>
    <NavigateToPose goal="{waypoint_A}" />
    <NavigateToPose goal="{waypoint_B}" />
  </Sequence>
</Repeat>
```

## Retry

Retries the child up to N times on FAILURE. Returns SUCCESS on the first successful attempt. Returns FAILURE if all retries are exhausted:

```xml
<Retry num_attempts="3">
  <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" />
</Retry>
```

If path planning fails, retry up to 3 times before giving up. Combine with recovery actions:

```xml
<Retry num_attempts="3">
  <Sequence>
    <ClearEntireCostmap server_name="global_costmap/clear_entirely_global_costmap" />
    <ComputePathToPose goal="{goal}" path="{path}" />
  </Sequence>
</Retry>
```

Each retry clears the costmap first, giving the planner fresh data.

## KeepRunningUntilFailure

Returns RUNNING as long as the child returns SUCCESS or RUNNING. Only returns FAILURE when the child fails:

```xml
<KeepRunningUntilFailure>
  <Sequence>
    <Script code="goal_index := (goal_index + 1) % 4" />
    <NavigateToPose goal="{current_goal}" />
  </Sequence>
</KeepRunningUntilFailure>
```

The inner Sequence visits goals endlessly. Each time it succeeds (one goal reached), `KeepRunningUntilFailure` returns RUNNING, causing the parent to tick it again. The loop continues until NavigateToPose fails (e.g., unreachable goal).

This is the standard pattern for infinite patrol loops.

## Timeout

Returns FAILURE if the child takes longer than `msec` milliseconds to complete:

```xml
<Timeout msec="30000">
  <NavigateToPose goal="{goal}" />
</Timeout>
```

If navigation takes longer than 30 seconds, the child is halted and FAILURE is returned. Use for time-bounded operations:

```xml
<Timeout msec="10000">
  <Spin spin_dist="3.14" />
</Timeout>
```

If the spin behavior gets stuck (e.g., blocked by an obstacle), the timeout prevents it from running forever.

## Delay

Waits for `delay_msec` milliseconds before ticking the child for the first time. On subsequent ticks, the child is ticked normally:

```xml
<Delay delay_msec="2000">
  <ComputePathToPose goal="{goal}" path="{path}" />
</Delay>
```

Use when you need a brief pause before starting an action (e.g., wait for sensor data to stabilize after a recovery behavior).

## Combining Decorators

Decorators compose by nesting. The outermost decorator is applied first:

```xml
<Timeout msec="60000">
  <Retry num_attempts="3">
    <RateController hz="1.0">
      <ComputePathToPose goal="{goal}" path="{path}" />
    </RateController>
  </Retry>
</Timeout>
```

This means: plan at most once per second, retry up to 3 times on failure, but abandon entirely if 60 seconds elapse. The evaluation order from the child's perspective is: rate-limited execution → retry on failure → global timeout.

## Decorator Selection Guide

| Need                                        | Decorator              |
|---------------------------------------------|------------------------|
| Throttle an expensive operation             | RateController         |
| Replan only after moving                    | DistanceController     |
| Adaptive replanning by speed                | SpeedController        |
| Run something exactly once                  | SingleTrigger          |
| Negate a condition                          | Inverter               |
| Ignore failure of a best-effort action      | ForceSuccess           |
| Repeat a patrol loop N times               | Repeat                 |
| Retry planning with recovery               | Retry                  |
| Infinite loop until failure                 | KeepRunningUntilFailure|
| Bound execution time                        | Timeout                |
