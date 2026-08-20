<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: smac-lattice-planner
description: 'Configure Nav2 SMAC Lattice planner. Use when using pre-computed motion primitives for non-circular or complex robot footprints.'
---

# SMAC Lattice Planner

## Plugin
```
nav2_smac_planner::SmacPlannerLattice
```

## Algorithm
The Lattice planner uses precomputed motion primitives (short trajectory arcs) to search the SE2 state space (x, y, theta). Unlike Hybrid-A*, which generates Dubins/Reeds-Shepp curves on the fly, the Lattice planner loads a fixed set of primitives from a JSON file. Each primitive defines a kinematically feasible trajectory segment that the robot can actually execute, making paths highly realistic for robots with complex kinematic constraints.

The search expands nodes by applying each applicable primitive at the current (x, y, theta) state, checking for collisions, and evaluating cost. This is equivalent to searching a state lattice graph where edges are the precomputed primitives.

## Lattice File

The lattice file is a JSON file defining motion primitives. Each primitive specifies:
- Start heading (quantized)
- End heading
- A sequence of (x, y, theta) poses along the trajectory
- Left and right turning radii
- Trajectory length

### Generating a Lattice File

Use the provided Python tool from the `nav2_smac_planner` package:

```bash
ros2 run nav2_smac_planner lattice_generator \
  --output lattice_primitives.json \
  --turning_radius 0.20 \
  --grid_resolution 0.05 \
  --heading_bins 16 \
  --max_length 1.0
```

For differential drive robots, the key parameters are:
- `--turning_radius`: Minimum turning radius in meters (use 0.1–0.4 for diff-drive).
- `--grid_resolution`: Must match your costmap resolution.
- `--heading_bins`: Number of discrete headings (16 = 22.5° resolution, 8 = 45°). More bins = more primitives = slower planning but finer control.

Alternatively, you can write a custom lattice file. The JSON structure:

```json
{
  "version": 1.0,
  "resolution": 0.05,
  "heading_angles": 16,
  "number_of_trajectories": 128,
  "motion_model": "diff",
  "trajectories": [
    {
      "trajectory_id": 0,
      "start_angle_index": 0,
      "end_angle_index": 0,
      "left_turn_radius": 0.0,
      "right_turn_radius": 0.0,
      "trajectory_length": 0.15,
      "arc_length": 0.15,
      "straight_length": 0.15,
      "poses": [
        {"x": 0.0, "y": 0.0, "theta": 0.0},
        {"x": 0.05, "y": 0.0, "theta": 0.0},
        {"x": 0.10, "y": 0.0, "theta": 0.0},
        {"x": 0.15, "y": 0.0, "theta": 0.0}
      ]
    }
  ]
}
```

## Key Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lattice_filepath` | string | "" | **Required.** Absolute path to the lattice primitives JSON file. |
| `allow_unknown` | bool | true | Plan through unknown space. |
| `allow_reverse_expansion` | bool | false | Allow reverse motion primitives. |
| `max_iterations` | int | 1000000 | Maximum search iterations. |
| `max_on_approach_iterations` | int | 1000 | Iterations to improve path after finding goal. |
| `max_planning_time` | double | 5.0 | Hard time limit in seconds. |
| `cost_travel_multiplier` | double | 2.0 | Costmap cell cost multiplier during search. |
| `change_penalty` | double | 0.0 | Penalty for switching between forward and reverse. |
| `non_straight_penalty` | double | 1.20 | Penalty multiplier for turning primitives. |
| `cost_penalty` | double | 2.0 | Penalty multiplier for high-cost cells. |
| `reverse_penalty` | double | 2.0 | Penalty multiplier for reverse primitives. |

## Complete YAML Configuration

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerLattice"
      lattice_filepath: "/path/to/lattice_primitives.json"
      tolerance: 0.25
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0
      allow_unknown: true
      allow_reverse_expansion: false
      cost_travel_multiplier: 2.0
      change_penalty: 0.0
      non_straight_penalty: 1.20
      cost_penalty: 2.0
      reverse_penalty: 2.0
      downsample_costmap: false
      downsampling_factor: 1
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
```

## When To Use

**Good fit:**
- Ackermann steering robots (car-like) with a fixed minimum turning radius — the lattice exactly encodes the vehicle's motion capabilities.
- Robots with complex or asymmetric kinematic constraints (e.g., different forward/reverse turning radii).
- Applications requiring paths that can be directly tracked by the controller with minimal correction.
- Custom vehicle types where Dubins/Reeds-Shepp curves are not an accurate model.

**Poor fit:**
- Differential drive robots in typical indoor environments — Hybrid-A* or even NavFn + controller smoothing is simpler and usually sufficient.
- Environments requiring very fast replanning — the Lattice planner has overhead from loading and matching primitives.
- Prototyping or quick iteration — generating and tuning lattice files adds complexity.

## Lattice vs Hybrid-A*

| Aspect | Hybrid-A* | Lattice |
|--------|-----------|---------|
| Motion model | Dubins/Reeds-Shepp curves | Precomputed arbitrary primitives |
| Configuration | Parameters only | Requires lattice JSON file |
| Kinematic fidelity | Good for circular arcs | Exact match to robot capability |
| Setup complexity | Low | Medium-High |
| Path quality | Smooth arcs | Depends on primitive set quality |
| Best for | Diff-drive, simple Ackermann | Complex kinematics, custom vehicles |

## Troubleshooting

- **"Lattice file not found" error:** Ensure `lattice_filepath` is an absolute path and the file exists on the robot's filesystem at runtime.
- **Planner produces jerky paths:** The lattice file may have too few heading bins. Regenerate with more bins (16→32).
- **Planner fails in tight spaces:** The primitives may be too long or the turning radius too large. Regenerate with smaller `--max_length` and `--turning_radius`.
- **Resolution mismatch error:** The lattice file's `resolution` field must exactly match your costmap resolution.
- **Slow planning:** Reduce the number of heading bins or increase `downsampling_factor`. Consider switching to Hybrid-A* if lattice precision isn't necessary.

## File Deployment

The lattice JSON file must be accessible at the path specified in `lattice_filepath` on the machine running the planner. For ROS 2 packages, place it in your config directory and reference it with:

```yaml
lattice_filepath: ""  # Set via launch file
```

In your launch file:
```python
lattice_path = os.path.join(
    get_package_share_directory('my_robot_nav'),
    'config', 'lattice_primitives.json'
)
```

Then pass it as a parameter override.
