<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: mppi-trajectory-visualization
description: 'Visualize MPPI trajectory rollouts. Use when debugging MPPI controller behavior, viewing sampled trajectories, or analyzing critic scoring.'
---

# MPPI Trajectory Visualization

## Enabling Visualization

Set `visualize: true` in the MPPI controller configuration:

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      visualize: true
      # ... other params
```

## Published Topics

When visualization is enabled, MPPI publishes:

| Topic | Type | Description |
|-------|------|-------------|
| `/mppi_controller/trajectories` | `visualization_msgs/msg/MarkerArray` | All sampled trajectory rollouts |
| `/mppi_controller/transformed_global_plan` | `nav_msgs/msg/Path` | The portion of the global plan used by MPPI |
| `/mppi_controller/optimal_trajectory` | `visualization_msgs/msg/MarkerArray` | The selected optimal trajectory |

## RViz2 Setup

1. Open RViz2 and click **Add** → **By topic**.
2. Expand `/mppi_controller/trajectories` and select **MarkerArray**.
3. Expand `/mppi_controller/optimal_trajectory` and select **MarkerArray**.
4. Optionally add `/mppi_controller/transformed_global_plan` as a **Path** display.

Recommended RViz2 display settings:
```
MarkerArray (trajectories):
  Topic: /mppi_controller/trajectories
  Marker Alpha: 0.3  (reduce opacity to avoid visual clutter)
  
MarkerArray (optimal):
  Topic: /mppi_controller/optimal_trajectory
  Marker Alpha: 1.0
```

## Color Coding

Trajectories are colored by cost using a gradient:
- **Green**: Low cost — good trajectories that satisfy critics.
- **Yellow**: Moderate cost — suboptimal but viable.
- **Red**: High cost — near-collision, constraint violation, or heavily penalized.

The optimal trajectory (the weighted average of good trajectories) is displayed separately and is typically a distinct color (blue or bright green).

## Interpreting Healthy Visualization

### Healthy Patterns

- **Fan-shaped spread**: Trajectories radiate forward and to the sides from the robot, forming a fan. The green trajectories cluster around the path, red trajectories are off-path or near obstacles. This indicates good sampling diversity and proper critic balance.
- **Smooth convergence**: Frame-to-frame, the optimal trajectory shifts smoothly. No sudden jumps.
- **Green tunnel through obstacles**: In narrow passages, green trajectories form a narrow band through the corridor center. Red trajectories hit walls on either side.

### Problematic Patterns

#### All Trajectories Red
- **Cause**: Critics are too punishing OR costmap is too restrictive (inflation too wide) OR the robot is near-surrounded by obstacles.
- **Fix**: Reduce `CostCritic.cost_weight`, reduce costmap inflation radius, increase `batch_size` to find viable gaps.
- **Danger**: If all trajectories are red, MPPI may command zero velocity (stuck) or produce erratic motion.

#### Trajectories Cluster Very Tightly
- **Cause**: `vx_std` and/or `wz_std` are too small. The Gaussian perturbations don't explore enough of the trajectory space.
- **Fix**: Increase `vx_std` (e.g., 0.2 → 0.3) and `wz_std` (e.g., 0.4 → 0.6). Also ensure `batch_size` is sufficient.

#### Trajectories Oscillate Frame-to-Frame
- **Cause**: `temperature` too high (averaging too many mediocre trajectories) OR conflicting critic weights causing the optimal to flip between two competing solutions.
- **Fix**: Lower `temperature` (e.g., 0.3 → 0.15). Check for critic weight conflicts, especially `PathAlignCritic` vs `CostCritic` near obstacles.

#### Trajectories Extend Too Far or Too Short
- **Cause**: `time_steps × model_dt` is mismatched to the environment.
- **Fix**: For indoor use, a 2–3 second horizon is typical. Adjust `time_steps` or `model_dt`.

#### Trajectories Curve Away from the Path
- **Cause**: `PathAlignCritic.cost_weight` too low relative to `CostCritic` — the costmap is repelling the robot from the path.
- **Fix**: Increase `PathAlignCritic.cost_weight` or reduce `CostCritic.cost_weight` or reduce inflation.

#### Optimal Trajectory Passes Through Obstacles
- **Cause**: `CostCritic.cost_weight` too low OR `trajectory_point_step` too high (skipping collision checks) OR costmap not up-to-date.
- **Fix**: Increase `CostCritic.cost_weight`, set `trajectory_point_step: 1`, verify costmap update rate.

## CPU Impact

Visualization adds overhead:
- Constructing MarkerArray messages for `batch_size` trajectories × `time_steps` points: 5–15ms per cycle.
- Network serialization and RViz2 rendering: depends on subscriber.
- For 2000 trajectories × 56 time steps = 112,000 markers per message.

**Always disable for production** (`visualize: false`). The overhead can push the controller computation over the frequency budget.

## Diagnostic Workflow

1. Set `visualize: true` and open RViz2 with the trajectory MarkerArray.
2. Command a navigation goal through a challenging area (doorway, corner, narrow hall).
3. Observe:
   - Is there a clear green "best corridor" through obstacles?
   - Does the optimal trajectory track the planned path?
   - Are there sudden jumps in the optimal trajectory?
4. If problems are visible, adjust critic weights and re-test.
5. Compare with the global plan (Path display) to verify alignment.
6. Once satisfied, disable visualization.

## Recording for Off-line Analysis

```bash
ros2 bag record /mppi_controller/trajectories /mppi_controller/optimal_trajectory \
  /local_costmap/costmap /plan -o mppi_debug_bag
```

Replay with:
```bash
ros2 bag play mppi_debug_bag
```

Then open RViz2 and add the topics for frame-by-frame analysis.

## Combining with Costmap Visualization

For full diagnostic visibility, also display:
- `/local_costmap/costmap` (OccupancyGrid) — see what the controller sees.
- `/global_costmap/costmap` — see what the planner sees.
- `/plan` (Path) — the planned global path.
- `/local_plan` (Path) — the controller's local trajectory command.

Overlay all four with the MPPI trajectory markers for a complete picture of the controller's decision-making.
