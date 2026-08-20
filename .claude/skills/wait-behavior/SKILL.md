<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: wait-behavior
description: 'Configure Nav2 wait recovery behavior. Use when adding timed waits for dynamic obstacle clearing or sensor stabilization.'
---

# Wait Recovery Behavior

## Overview

The Wait behavior pauses the robot for a specified duration, publishing zero velocity. It is the simplest recovery behavior and the appropriate first response when the obstruction is likely a dynamic obstacle (person, pet, door) that will clear on its own. Plugin: `nav2_behaviors::Wait`.

## BT Node

```xml
<Wait wait_duration="5" server_name="behavior_server" server_timeout="10"/>
```

`wait_duration` is in **seconds** (integer). The behavior always returns `SUCCESS` after the duration elapses — it cannot fail.

## How It Works

1. The behavior server receives the wait duration from the BT action node.
2. Each cycle, it publishes `geometry_msgs/msg/Twist` with all fields zero, ensuring the robot is stationary.
3. It monitors elapsed time since the behavior started.
4. After `wait_duration` seconds, it returns `SUCCESS`.

There are no collision checks, costmap interactions, or TF requirements beyond basic operation. This is intentionally minimal.

## YAML Configuration

Wait requires no behavior-specific parameters. It only needs to be registered in the behavior plugin list:

```yaml
behavior_server:
  ros__parameters:
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    wait:
      plugin: "nav2_behaviors::Wait"
```

The `wait_duration` is set per-invocation in the BT node, not in YAML.

## Where Wait Fits in Recovery BTs

Wait is most effective as the **first** recovery action in a `RoundRobin` sequence. The logic: try the cheapest, least-disruptive recovery first. If the obstacle clears during the wait, navigation resumes without any robot motion.

### Recommended Recovery Ordering

```xml
<RecoveryNode number_of_retries="6" name="NavigateRecovery">
  <PipelineSequence name="NavigateWithReplanning">
    <RateController hz="1.0">
      <ComputePathToPose goal="{goal}" path="{path}"/>
    </RateController>
    <FollowPath path="{path}"/>
  </PipelineSequence>
  <RoundRobin name="RecoveryActions">
    <!-- 1st attempt: just wait, obstacle may clear -->
    <Wait wait_duration="5"/>
    <!-- 2nd attempt: small spin to clear costmap artifacts -->
    <Spin spin_dist="1.0"/>
    <!-- 3rd attempt: wait longer -->
    <Wait wait_duration="8"/>
    <!-- 4th attempt: back up and spin -->
    <Sequence>
      <BackUp backup_dist="0.15" backup_speed="0.05"/>
      <Spin spin_dist="1.57"/>
    </Sequence>
    <!-- 5th attempt: clear costmaps entirely -->
    <Sequence>
      <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
      <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
    </Sequence>
    <!-- 6th attempt: aggressive backup -->
    <Sequence>
      <BackUp backup_dist="0.30" backup_speed="0.10"/>
      <Spin spin_dist="3.14"/>
    </Sequence>
  </RoundRobin>
</RecoveryNode>
```

The `RoundRobin` node cycles through children on each failure. So the first navigation failure triggers Wait(5s), the second triggers Spin, the third triggers Wait(8s), and so on. After all children are exhausted, `number_of_retries` resets the cycle.

## When to Use Wait vs. Other Recoveries

| Situation | Best Recovery |
|---|---|
| Person walking through path | **Wait** — they will move |
| Door briefly blocked | **Wait** — it will open |
| Costmap ghost from sensor noise | **Spin** — rotation clears phantom readings |
| Physically stuck against obstacle | **BackUp** — need to create space |
| Narrow passage, path blocked | **ClearCostmap + Replan** |

## Combining Wait with Decorators

You can use BT decorator nodes to add conditions to Wait:

```xml
<!-- Wait up to 10 seconds, but recheck the path every 2 seconds -->
<RetryUntilSuccessful num_attempts="5">
  <Sequence>
    <Wait wait_duration="2"/>
    <ComputePathToPose goal="{goal}" path="{path}"/>
    <FollowPath path="{path}"/>
  </Sequence>
</RetryUntilSuccessful>
```

This pattern retries navigation after short waits rather than committing to a single long pause.

## House Patrol Considerations

For a home patrol robot, Wait is particularly important because:
- Most obstructions are household members or pets that will move within seconds.
- Aggressive recoveries (spinning, backing up) in a home setting risk bumping furniture or alarming occupants.
- A patient robot that waits quietly is preferable to one that thrashes around.

Consider using longer `wait_duration` values (8–15 seconds) for the first recovery attempt in domestic environments.

## Debugging

Wait is the simplest behavior to debug. If it is not executing:
```bash
ros2 action list           # verify /wait action exists
ros2 topic echo /cmd_vel   # should show all-zero Twist during wait
```

If the BT skips Wait, the issue is in BT structure, not the behavior itself.
