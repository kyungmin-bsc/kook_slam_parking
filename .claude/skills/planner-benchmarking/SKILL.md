<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: planner-benchmarking
description: 'Benchmark Nav2 planner performance. Use when measuring planning time, path length, or comparing planner quality.'
---

# Planner Benchmarking

## What to Measure

| Metric | Source | Description |
|--------|--------|-------------|
| **Planning time** | Planner server logs or `/plan` topic timestamps | Wall-clock time from request to path publication. |
| **Path length** | Sum of Euclidean segment lengths in `/plan` | Total distance the robot must travel. |
| **Path smoothness** | Angular change at each waypoint | Sum or max of heading changes between consecutive segments. Lower = smoother. |
| **Direction reversals** | Sign changes in forward velocity along path | Count of forward→reverse transitions (relevant for Reeds-Shepp). |
| **Costmap cost along path** | Sample costmap at each path pose | Total or max cost encountered. High values indicate proximity to obstacles. |
| **Planning success rate** | Count failures over N trials | Percentage of requests that produce a valid path. |

## Collecting Data

### Planning Time from Logs

The planner server logs planning duration at DEBUG level:

```bash
ros2 run nav2_planner planner_server --ros-args --log-level nav2_planner:=DEBUG
```

Look for log lines like:
```
[planner_server]: Created plan of length X.XX in Y.YYs
```

### Recording Plans with rosbag2

```bash
ros2 bag record /plan /goal_pose /tf /tf_static \
  --output planner_benchmark_bag \
  --max-bag-duration 300
```

Replay for fair comparison across planners:

```bash
# Record goals separately
ros2 bag record /goal_pose --output goals_only

# Replay goals while running different planner configs
ros2 bag play goals_only --topics /goal_pose
```

### Echoing the Plan Topic

```bash
ros2 topic echo /plan --once
```

The `/plan` topic publishes `nav_msgs/msg/Path`, containing a header and a `poses[]` array of `geometry_msgs/msg/PoseStamped`.

## Repeatable Test Scenarios

### Fixed Start/Goal Pairs

Define a set of test cases covering different planning challenges:

```yaml
test_scenarios:
  - name: "open_room_diagonal"
    start: {x: 1.0, y: 1.0, yaw: 0.0}
    goal:  {x: 8.0, y: 6.0, yaw: 1.57}

  - name: "narrow_corridor"
    start: {x: 2.0, y: 5.0, yaw: 0.0}
    goal:  {x: 12.0, y: 5.0, yaw: 0.0}

  - name: "around_obstacle"
    start: {x: 1.0, y: 3.0, yaw: 0.0}
    goal:  {x: 1.0, y: 7.0, yaw: 3.14}

  - name: "tight_turn"
    start: {x: 3.0, y: 2.0, yaw: 1.57}
    goal:  {x: 4.0, y: 3.0, yaw: 0.0}

  - name: "long_range"
    start: {x: 0.5, y: 0.5, yaw: 0.0}
    goal:  {x: 19.0, y: 14.0, yaw: -1.57}
```

### Automated Goal Publisher

```python
#!/usr/bin/env python3
"""Publish a sequence of goals for planner benchmarking."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import math
import time
import json

class PlannerBenchmark(Node):
    def __init__(self):
        super().__init__('planner_benchmark')
        self.navigator = BasicNavigator()
        self.plan_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )
        self.plan_sub = self.create_subscription(
            PoseStamped, '/plan', self.plan_callback, 10
        )
        # Subscribe to the full path
        from nav_msgs.msg import Path
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10
        )
        self.results = []

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def path_callback(self, msg):
        """Analyze a received path."""
        if len(msg.poses) < 2:
            return
        metrics = compute_path_metrics(msg)
        self.results.append(metrics)
        self.get_logger().info(
            f"Path: length={metrics['length']:.3f}m, "
            f"smoothness={metrics['smoothness']:.4f}, "
            f"poses={metrics['num_poses']}"
        )
```

## Computing Metrics from nav_msgs/Path

```python
import math
import numpy as np
from nav_msgs.msg import Path

def compute_path_metrics(path_msg: Path) -> dict:
    """Compute benchmarking metrics from a Path message."""
    poses = path_msg.poses
    n = len(poses)
    if n < 2:
        return {'length': 0.0, 'smoothness': 0.0, 'num_poses': n,
                'max_cost': 0, 'reversals': 0}

    # Path length: sum of Euclidean segment distances
    length = 0.0
    segments = []
    for i in range(1, n):
        dx = poses[i].pose.position.x - poses[i-1].pose.position.x
        dy = poses[i].pose.position.y - poses[i-1].pose.position.y
        seg_len = math.hypot(dx, dy)
        length += seg_len
        segments.append((dx, dy, seg_len))

    # Smoothness: sum of absolute angular changes between segments
    smoothness = 0.0
    angular_changes = []
    for i in range(1, len(segments)):
        dx1, dy1, _ = segments[i-1]
        dx2, dy2, _ = segments[i]
        angle1 = math.atan2(dy1, dx1)
        angle2 = math.atan2(dy2, dx2)
        delta = abs(math.atan2(
            math.sin(angle2 - angle1),
            math.cos(angle2 - angle1)
        ))
        smoothness += delta
        angular_changes.append(delta)

    # Direction reversals (dot product sign change)
    reversals = 0
    for i in range(1, len(segments)):
        dot = (segments[i-1][0] * segments[i][0] +
               segments[i-1][1] * segments[i][1])
        if dot < 0 and segments[i-1][2] > 0.01 and segments[i][2] > 0.01:
            reversals += 1

    return {
        'length': length,
        'smoothness': smoothness,
        'max_angular_change': max(angular_changes) if angular_changes else 0.0,
        'num_poses': n,
        'reversals': reversals,
    }
```

## Costmap Cost Along Path

To measure obstacle proximity, sample the costmap at each path pose:

```python
def sample_costmap_along_path(costmap, path_msg):
    """Sample costmap values at each path pose.

    costmap: nav2_simple_commander costmap or OccupancyGrid.
    """
    costs = []
    for pose_stamped in path_msg.poses:
        wx = pose_stamped.pose.position.x
        wy = pose_stamped.pose.position.y
        # Convert world to map coordinates
        mx = int((wx - costmap.metadata.origin.position.x)
                 / costmap.metadata.resolution)
        my = int((wy - costmap.metadata.origin.position.y)
                 / costmap.metadata.resolution)
        if 0 <= mx < costmap.metadata.size_x and 0 <= my < costmap.metadata.size_y:
            idx = my * costmap.metadata.size_x + mx
            costs.append(costmap.data[idx])
    return {
        'max_cost': max(costs) if costs else 0,
        'mean_cost': sum(costs) / len(costs) if costs else 0,
        'total_cost': sum(costs),
        'lethal_cells': sum(1 for c in costs if c >= 253),
    }
```

## Planner Server Parameters Affecting Benchmarks

| Parameter | Effect on Benchmarking |
|-----------|----------------------|
| `expected_planner_frequency` | If set too low, the server may warn but still function. Set to 0.0 during benchmarking to suppress warnings. |
| `costmap_update_timeout` | Affects time waiting for costmap. Use a consistent value across tests. |
| Costmap `update_frequency` | Higher frequency means more current data but more CPU load. Keep constant across comparisons. |

## Results Reporting

Structure results for comparison:

```python
import json

def report_results(planner_name, scenario_name, metrics, planning_time_ms):
    return {
        'planner': planner_name,
        'scenario': scenario_name,
        'planning_time_ms': planning_time_ms,
        'path_length_m': round(metrics['length'], 3),
        'smoothness_rad': round(metrics['smoothness'], 4),
        'max_angular_change_rad': round(metrics['max_angular_change'], 4),
        'num_poses': metrics['num_poses'],
        'reversals': metrics['reversals'],
    }

# Write results to JSON for later analysis
with open('benchmark_results.json', 'w') as f:
    json.dump(all_results, f, indent=2)
```

## Fair Comparison Checklist

1. **Same map and costmap configuration** across all planners.
2. **Same start/goal pairs** — use recorded bag files or fixed coordinates.
3. **Warm up the planner** — discard the first planning request (cold-start overhead from costmap subscription).
4. **Multiple runs per scenario** — plan each scenario 10+ times and report mean/std.
5. **Same hardware and system load** — close unnecessary processes, use `nice -n -10` for the planner if needed.
6. **Record raw data** — save Path messages and timing for post-hoc analysis.
