<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: nav2-architecture-overview
description: 'Understand Nav2 architecture and component interactions. Use when learning how planner, controller, BT navigator, and costmaps fit together.'
---

# Nav2 Architecture Overview

## System Architecture

Nav2 is a collection of lifecycle-managed servers orchestrated by a Behavior Tree (BT) Navigator. Every server runs as a lifecycle node managed by `nav2_lifecycle_manager`.

### Core Servers

| Server | Role | Default Action |
|--------|------|----------------|
| **BT Navigator** | Orchestrates navigation via behavior trees | `navigate_to_pose`, `navigate_through_poses` |
| **Planner Server** | Computes global paths (e.g., NavFn) | `compute_path_to_pose` |
| **Controller Server** | Follows paths locally (e.g., MPPI) | `follow_path` |
| **Behavior Server** | Executes recoveries (spin, backup, wait) | `spin`, `backup`, `drive_on_heading`, `wait` |
| **Smoother Server** | Refines global paths post-planning | `smooth_path` |
| **Velocity Smoother** | Limits acceleration/jerk on cmd_vel | Subscribes/publishes cmd_vel |
| **Collision Monitor** | Final safety layer before motors | Subscribes/publishes cmd_vel |
| **Waypoint Follower** | Manages multi-goal sequences | `follow_waypoints` |

### Costmap Servers

The **Planner Server** hosts the **global costmap** and the **Controller Server** hosts the **local costmap**. Each costmap is a `Costmap2D` instance with its own plugin layers (static, inflation, obstacle, voxel). The global costmap covers the full map; the local costmap is a rolling window around the robot.

## Data Flow Pipeline

```
Goal (PoseStamped)
  → BT Navigator (selects planner + controller)
    → Planner Server → global path (nav_msgs/Path)
      → Smoother Server (optional) → smoothed path
        → Controller Server → raw cmd_vel
          → Velocity Smoother → rate-limited cmd_vel
            → Collision Monitor → safe cmd_vel
              → Motor driver
```

The BT Navigator is the central coordinator. It does NOT compute paths or velocities itself—it delegates to the planner and controller servers via action calls and handles recovery logic through behavior tree nodes.

## Lifecycle Management

All Nav2 servers implement the ROS 2 managed (lifecycle) node interface. The `nav2_lifecycle_manager` transitions them through: `unconfigured → inactive → active`. Two lifecycle managers are typical:

- **`lifecycle_manager_localization`**: manages `map_server` and `amcl`
- **`lifecycle_manager_navigation`**: manages `controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother`, `collision_monitor`

Startup order matters—nodes are activated in the order listed in `node_names`.

## Launch Structure (nav2_bringup)

The standard `nav2_bringup` package provides:

```
bringup_launch.py          # Top-level: localization + navigation + rviz
  ├── localization_launch.py   # map_server + amcl + lifecycle_manager_localization
  ├── navigation_launch.py     # All nav servers + lifecycle_manager_navigation
  └── rviz_launch.py           # RViz2 with nav2 panel
```

`navigation_launch.py` uses `RewrittenYaml` to inject `use_sim_time` into every node's parameters:

```python
configured_params = RewrittenYaml(
    source_file=params_file,
    root_key='',
    param_rewrites={'use_sim_time': use_sim_time},
    convert_types=True
)
```

## BT Navigator Details

The BT Navigator loads an XML behavior tree file. Default trees are in `nav2_bt_navigator/behavior_trees/`. Key trees:

- `navigate_to_pose_w_replanning_and_recovery.xml` — standard single-goal with replanning and spin/backup/wait recovery
- `navigate_through_poses_w_replanning_and_recovery.xml` — multi-pose variant

The BT uses action nodes (`ComputePathToPose`, `FollowPath`), condition nodes (`GoalReached`, `IsPathValid`), decorator nodes (`RateController` for replanning rate), and control nodes (`PipelineSequence`, `RecoveryNode`).

## Key Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/cmd_vel` | `geometry_msgs/Twist` | Final velocity command to motors |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Controller output before smoothing |
| `/plan` | `nav_msgs/Path` | Current global plan for visualization |
| `/local_plan` | `nav_msgs/Path` | Controller's local trajectory |
| `/map` | `nav_msgs/OccupancyGrid` | Static map |
| `/global_costmap/costmap` | `nav2_msgs/Costmap` | Global costmap |
| `/local_costmap/costmap` | `nav2_msgs/Costmap` | Local costmap |

## Key Design Decisions

- **Plugin-based**: Planners, controllers, behaviors, costmap layers, BT nodes are all plugins loaded at runtime. This means you configure which algorithms to use entirely through YAML parameters.
- **Action-based communication**: Servers expose ROS 2 actions, enabling preemption, feedback, and result handling.
- **Bond connections**: The lifecycle manager maintains bonds with managed nodes. If a node dies, the manager detects the broken bond and can restart it.
- **Separation of concerns**: The planner only sees the global costmap. The controller only sees the local costmap. The collision monitor only sees raw sensor data. This layered approach provides defense in depth.

## Common Debugging Entry Points

- If navigation doesn't start: check lifecycle manager logs, ensure all nodes reached `active` state
- If the robot doesn't move: check the cmd_vel chain from controller → velocity_smoother → collision_monitor
- If paths are wrong: visualize the costmaps and check layer configuration
- If recovery loops forever: inspect the BT XML and the behavior server logs
