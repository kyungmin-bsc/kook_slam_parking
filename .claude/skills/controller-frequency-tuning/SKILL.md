<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: controller-frequency-tuning
description: 'Tune Nav2 controller server frequency. Use when adjusting controller_frequency, balancing CPU load, or diagnosing stale velocity commands.'
---

# Controller Frequency and Timing Tuning

## controller_frequency

The `controller_frequency` parameter sets how often `computeVelocityCommands()` is called per second. This is the control loop rate for the `controller_server` node.

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Hz
```

### Choosing the Right Frequency

| Frequency | Use Case | Notes |
|-----------|----------|-------|
| 10 Hz | Low-speed robots, RPP controller | Minimum for reasonable tracking |
| 20 Hz | Typical indoor robot with MPPI/DWB | Good balance of smoothness and CPU |
| 30 Hz | Fast robots or high-precision tracking | Requires fast controller computation |
| 50+ Hz | Competition or high-speed applications | Only feasible with RPP or very fast hardware |

**The frequency must be achievable.** If MPPI takes 40ms to compute, the maximum effective frequency is 25Hz. Setting `controller_frequency: 30.0` would cause the controller to run behind schedule.

### Diagnosis: Controller Too Slow

When the controller cannot keep up with the requested frequency, you'll see warnings in the logs:

```
[controller_server]: Control loop missed its desired rate of 20.0000Hz...
```

**Causes and fixes**:
- MPPI `batch_size` too high → reduce to 1000–1500.
- `consider_footprint: true` on CostCritic → set to `false` or reduce footprint vertices.
- Local costmap too large → reduce `width`/`height` or increase `resolution`.
- CPU contention → check other processes, reduce sensor processing.
- Visualization enabled → set `visualize: false`.

### Measuring Controller Compute Time

```bash
# Check the controller_server's actual loop timing
ros2 topic hz /cmd_vel
# If the rate is lower than controller_frequency, the controller is overrunning

# For detailed timing, enable debug logging:
ros2 param set /controller_server use_sim_time false
# Then check log output for timing information
```

Alternatively, instrument with `rclcpp::Clock`:
```bash
ros2 topic echo /diagnostics | grep -A5 controller
```

## costmap_update_timeout

How long the controller server waits for a fresh costmap before computing on stale data.

```yaml
controller_server:
  ros__parameters:
    costmap_update_timeout: 0.30  # seconds
```

**Too short**: Controller runs on stale costmap data — may drive into recently-appeared obstacles.
**Too long**: Robot pauses while waiting for costmap, causing jerky motion.
**Typical**: 0.2–0.5s. Should be longer than the costmap update period (usually 1/costmap_frequency).

If the local costmap is configured at 5Hz (0.2s period), set `costmap_update_timeout: 0.30` to allow some slack.

## failure_tolerance

Number of consecutive `computeVelocityCommands()` failures before the controller server declares navigation failed.

```yaml
controller_server:
  ros__parameters:
    failure_tolerance: 10
```

**Behavior**: If the controller throws an exception (e.g., `NoValidControl`) for `failure_tolerance` consecutive cycles, the `FollowPath` action returns FAILED, triggering recovery behaviors.

**Typical values**:
- 5–10: Allows brief blocking (robot momentarily stuck) without aborting.
- 1: Strict — any failure triggers recovery immediately.
- 0: Never tolerate failures.
- -1: Infinite tolerance (never fail from controller errors — dangerous).

## Velocity Thresholds

These prevent the robot from receiving tiny velocity commands that the motors cannot execute:

```yaml
controller_server:
  ros__parameters:
    min_x_velocity_threshold: 0.001   # m/s
    min_y_velocity_threshold: 0.0     # m/s (0 for diff-drive)
    min_theta_velocity_threshold: 0.001  # rad/s
```

If the controller commands a velocity below this threshold, it is snapped to zero. This prevents:
- Motor whine from low PWM signals.
- Imperceptible creep that the odometry can't track.
- The robot "vibrating" when trying to hold position.

**Relationship to VelocityDeadbandCritic (MPPI)**: `min_x_velocity_threshold` operates at the controller server level (post-controller), while `VelocityDeadbandCritic` operates within MPPI's trajectory optimization. Both address motor deadband from different angles. Use both for robust behavior.

## Controller Selection Parameters

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    # For multiple controllers:
    # controller_plugins: ["FollowPath", "DockApproach"]
```

Each plugin entry has its own parameter namespace.

## progress_checker

The progress checker detects if the robot is stuck (not making progress toward the goal):

```yaml
controller_server:
  ros__parameters:
    progress_checker_plugins: ["progress_checker"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5  # meters
      movement_time_allowance: 10.0  # seconds
```

If the robot doesn't move `required_movement_radius` within `movement_time_allowance`, it's considered stuck and the FollowPath action fails.

**Tuning**:
- Aggressive (quick detection): `required_movement_radius: 0.3`, `movement_time_allowance: 5.0`.
- Lenient (allow waiting): `required_movement_radius: 0.5`, `movement_time_allowance: 15.0`.

## goal_checker

Determines when the robot has "arrived" at the goal:

```yaml
controller_server:
  ros__parameters:
    goal_checker_plugins: ["goal_checker"]
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25      # meters
      yaw_goal_tolerance: 0.25     # radians
      stateful: true               # once within tolerance, stay "reached"
```

**`stateful: true`**: Once the goal is reached, don't un-reach it if the robot drifts slightly. Prevents oscillation at the goal boundary.

## Complete Timing Configuration

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    costmap_update_timeout: 0.30
    failure_tolerance: 10
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.0
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker_plugins: ["goal_checker"]
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      # ... controller-specific params
```

## Diagnostic Commands

```bash
# Check actual control loop rate
ros2 topic hz /cmd_vel

# Check for timing warnings
ros2 topic echo /rosout | grep -i "missed.*rate\|too slow\|timeout"

# Check controller server status
ros2 lifecycle get /controller_server

# Monitor CPU usage
ros2 run rqt_top rqt_top
```
