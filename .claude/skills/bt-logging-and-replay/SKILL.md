<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: bt-logging-and-replay
description: 'Log and replay behavior tree execution. Use when recording BT traces, analyzing failure sequences, or debugging BT logic.'
---

# BT Logging and Replay

## Why Log Behavior Trees

BT logging captures every node status transition with timestamps, enabling:

- **Post-mortem debugging**: Understand why navigation failed without reproducing the scenario
- **Performance analysis**: Identify which nodes take the longest, how often recoveries trigger
- **Regression testing**: Compare BT execution traces before and after tree modifications
- **Compliance**: Record that the robot followed its intended behavior policy

## FileLogger2 (BT.CPP v4 Built-in)

`FileLogger2` is the recommended logger for BT.CPP v4. It writes a compact binary `.btlog` file recording every status change for every node:

### Enabling in bt_navigator

Nav2 does not expose FileLogger2 directly in its parameter interface. You enable it by modifying the bt_navigator plugin or using the `PublisherZMQ` logger which Groot2 can replay. For direct file logging, add it in a custom bt_navigator wrapper:

```cpp
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"

// After creating the tree:
BT::FileLogger2 logger(tree, "bt_trace.btlog");

// The logger automatically records all status transitions
// until it goes out of scope or the tree is destroyed.
```

### Log File Format

The `.btlog` format stores:
- Tree structure (node names, types, hierarchy)
- Timestamp for each status transition (microsecond precision)
- Node ID → status mapping for each change
- Blackboard entry changes (in v4.x builds that support it)

File sizes are compact: a 50-node tree running for 10 minutes at 30 Hz produces ~5-15 MB depending on how often nodes change status.

## Viewing Logs in Groot2 Replay Mode

1. Open Groot2
2. File → **Open Log** → select your `.btlog` file
3. The tree structure appears with a timeline slider at the bottom
4. Use the timeline to scrub through execution:
   - Green nodes: SUCCESS at that timestamp
   - Red nodes: FAILURE
   - Yellow: RUNNING
   - Gray: IDLE

### Replay Controls

| Control        | Function                                      |
|----------------|-----------------------------------------------|
| Play/Pause     | Animate execution at real-time or adjusted speed |
| Speed slider   | 0.1x to 10x replay speed                     |
| Step forward    | Advance one status change at a time           |
| Step backward   | Rewind one status change                      |
| Timeline scrub  | Jump to any point in the recording            |

## Alternative Loggers

### MinitraceLogger

Writes a Chrome-compatible trace format (`.json`) viewable in `chrome://tracing`:

```cpp
#include "behaviortree_cpp/loggers/bt_minitrace_logger.h"

BT::MinitraceLogger logger(tree, "bt_trace.json");
```

Open the JSON in Chrome DevTools → Performance tab → Load profile. Shows a flame chart of node execution with timing.

### SqliteLogger

Stores execution data in a SQLite database for programmatic analysis:

```cpp
#include "behaviortree_cpp/loggers/bt_sqlite_logger.h"

BT::SqliteLogger logger(tree, "bt_trace.db");
```

Query execution data with SQL:

```sql
-- Find all FAILURE events
SELECT timestamp, node_name, status FROM transitions
WHERE status = 'FAILURE' ORDER BY timestamp;

-- Count recovery triggers
SELECT node_name, COUNT(*) as trigger_count
FROM transitions
WHERE node_name LIKE '%Recovery%' AND status = 'SUCCESS'
GROUP BY node_name;

-- Average time spent in RUNNING state per node
SELECT node_name,
       AVG(duration_ms) as avg_running_ms
FROM (
  SELECT t1.node_name,
         (t2.timestamp - t1.timestamp) as duration_ms
  FROM transitions t1
  JOIN transitions t2 ON t1.node_name = t2.node_name
    AND t2.timestamp > t1.timestamp
    AND t1.status = 'RUNNING'
    AND t2.status != 'RUNNING'
) durations
GROUP BY node_name
ORDER BY avg_running_ms DESC;
```

## The /behavior_tree_log Topic

When `enable_groot_monitoring: true`, Nav2 publishes BT status changes to a ROS 2 topic:

```bash
ros2 topic echo /behavior_tree_log
```

Output format (simplified):
```
timestamp: 1714567890.123
event_log:
  - node_name: "NavigateToPose"
    previous_status: RUNNING
    current_status: SUCCESS
  - node_name: "Sequence_1"
    previous_status: RUNNING
    current_status: SUCCESS
```

### Recording for Later Analysis

```bash
# Record the BT log topic
ros2 bag record /behavior_tree_log -o bt_session_001

# Replay later
ros2 bag play bt_session_001
ros2 topic echo /behavior_tree_log
```

## Interpreting Replay Data

### Identifying Failure Points

Look for the **first** FAILURE transition in a sequence. In a Sequence node, the first child to fail causes the entire sequence to fail:

```
[t=10.5] ComputePathToPose: SUCCESS → RUNNING
[t=10.7] ComputePathToPose: RUNNING → FAILURE    ← Root cause
[t=10.7] PipelineSequence: RUNNING → FAILURE      ← Propagated
[t=10.7] RecoveryNode: ticks recovery branch
```

### Counting Recovery Frequency

If recoveries trigger frequently, the environment or parameters need tuning:

```
Recovery triggers in 10-minute session:
  ClearCostmaps: 12 times
  Spin: 8 times
  Wait: 4 times
  BackUp: 2 times

→ Costmap clearing is the most common recovery
→ Consider: is the obstacle layer too aggressive? Are phantom obstacles appearing?
```

### Time Analysis

Track how long the robot spends in each phase:

```
Navigation phase breakdown (10 min patrol):
  Planning (ComputePathToPose RUNNING): 12 seconds total
  Following (FollowPath RUNNING): 8 minutes 20 seconds
  Recovery (any recovery node RUNNING): 45 seconds
  Waiting (Wait RUNNING): 15 seconds
  Idle/transitioning: 28 seconds

→ Recovery consumes 7.5% of patrol time — acceptable
→ Planning is fast — planner is well-tuned
```

## Configuration for Production Logging

Recommended setup: file-based logging without ZMQ overhead:

```yaml
bt_navigator:
  ros__parameters:
    # Disable live monitoring for production
    enable_groot_monitoring: false

    # Custom parameter for log file path (if using custom bt_navigator)
    bt_log_filename: "/var/log/my_robot/bt_trace.btlog"
    bt_log_enabled: true
```

### Log Rotation

BT logs grow continuously. Implement rotation in your launch file or systemd service:

```python
# In launch file: generate timestamped log filenames
from datetime import datetime
log_file = f"/var/log/my_robot/bt_{datetime.now().strftime('%Y%m%d_%H%M%S')}.btlog"
```

Or use a cron job to clean old logs:

```bash
# Delete BT logs older than 7 days
find /var/log/my_robot/ -name "bt_*.btlog" -mtime +7 -delete
```

## Regression Testing with BT Logs

Compare BT execution traces across code changes:

1. **Baseline**: Record a `.btlog` during a known-good patrol run
2. **After change**: Record another `.btlog` with the modified tree
3. **Compare**: Check for new FAILURE events, increased recovery counts, or timing regressions

Automated comparison script pattern:

```python
import sqlite3

def compare_bt_logs(baseline_db: str, test_db: str):
    """Compare two BT execution logs for regressions."""
    baseline = sqlite3.connect(baseline_db)
    test = sqlite3.connect(test_db)

    # Compare failure counts per node
    query = """
        SELECT node_name, COUNT(*) as failures
        FROM transitions WHERE status = 'FAILURE'
        GROUP BY node_name
    """
    baseline_failures = dict(baseline.execute(query).fetchall())
    test_failures = dict(test.execute(query).fetchall())

    for node, count in test_failures.items():
        baseline_count = baseline_failures.get(node, 0)
        if count > baseline_count * 1.5:  # 50% regression threshold
            print(f"REGRESSION: {node} failures increased "
                  f"{baseline_count} → {count}")
```

## Summary: Logger Selection

| Logger        | Format   | Real-time | Groot2 Replay | Programmatic | Overhead |
|---------------|----------|-----------|---------------|--------------|----------|
| PublisherZMQ  | ZMQ      | Yes       | Via connect   | No           | High     |
| FileLogger2   | .btlog   | No        | Yes           | No           | Low      |
| MinitraceLogger | .json  | No        | No            | Chrome trace | Low      |
| SqliteLogger  | .db      | No        | No            | SQL queries  | Low      |
| ROS topic log | ROS bag  | Yes       | No            | ROS tools    | Medium   |

For most workflows: use `FileLogger2` in production, `PublisherZMQ` during development, and `SqliteLogger` for automated analysis.
