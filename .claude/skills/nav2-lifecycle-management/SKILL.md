<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: nav2-lifecycle-management
description: 'Manage Nav2 lifecycle node transitions. Use when debugging startup, configuring lifecycle_manager, or handling node state transitions.'
---

# Nav2 Lifecycle Management

## Overview

Nav2 uses `nav2_lifecycle_manager` to coordinate startup, shutdown, and health monitoring of all navigation servers. Each managed node implements the ROS 2 lifecycle interface (`rclcpp_lifecycle::LifecycleNode`), transitioning through: `unconfigured → inactive → active`.

## Configuration

```yaml
lifecycle_manager_navigation:
  ros__parameters:
    autostart: true
    node_names:
      - controller_server
      - smoother_server
      - planner_server
      - behavior_server
      - bt_navigator
      - waypoint_follower
      - velocity_smoother
      - collision_monitor
    bond_timeout: 4.0
    bond_respawn_max_duration: 10.0
    attempt_respawn_reconnection: true
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `autostart` | `true` | Automatically transition all nodes to active on startup |
| `node_names` | `[]` | Ordered list of managed nodes. **Startup order follows this list.** |
| `bond_timeout` | `4.0` | Seconds before a bond is considered broken |
| `bond_respawn_max_duration` | `10.0` | Max seconds to wait for a crashed node to respawn |
| `attempt_respawn_reconnection` | `true` | Whether to attempt reconnection after bond break |

## Startup Sequence

When `autostart: true`, the lifecycle manager:

1. Waits for each node in `node_names` to appear (service discovery)
2. Calls `configure` on each node sequentially (unconfigured → inactive)
3. Calls `activate` on each node sequentially (inactive → active)
4. Establishes bond connections with each active node

**Order matters.** Nodes that depend on others must come later in the list. For example, `bt_navigator` depends on `planner_server` and `controller_server`, so it must be listed after them.

## Bond Connections

Bonds are heartbeat connections between the lifecycle manager and each managed node. The managed node calls `createBond()` after activation:

```cpp
void MyServer::on_activate(const rclcpp_lifecycle::State &)
{
  // ... setup ...
  createBond();  // Establishes heartbeat with lifecycle manager
}
```

The lifecycle manager monitors these bonds. If a bond is broken (node crash, hang, or network partition), the manager:

1. Logs the broken bond
2. If `attempt_respawn_reconnection: true`, waits for the node to respawn (up to `bond_respawn_max_duration`)
3. If the node respawns, re-transitions it through configure → activate
4. If the node doesn't respawn in time, shuts down ALL managed nodes (cascade shutdown)

## autostart vs Manual Transition

### autostart: true (default)
All nodes transition to active automatically when the lifecycle manager starts. This is the standard mode for production robots.

### autostart: false
Nodes remain in `unconfigured` state. You must manually transition them using:

```bash
# Bring up all managed nodes
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"

# Pause (deactivate) all managed nodes
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 1}"

# Resume (reactivate) all managed nodes
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 2}"

# Reset (cleanup + unconfigure) all managed nodes
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 3}"
```

Or use the RViz nav2 panel which provides buttons for these transitions.

## Two Lifecycle Managers Pattern

The standard Nav2 setup uses TWO lifecycle managers:

```yaml
# Manager 1: Localization stack
lifecycle_manager_localization:
  ros__parameters:
    autostart: true
    node_names:
      - map_server
      - amcl

# Manager 2: Navigation stack
lifecycle_manager_navigation:
  ros__parameters:
    autostart: true
    node_names:
      - controller_server
      - planner_server
      - behavior_server
      - bt_navigator
      - velocity_smoother
      - collision_monitor
```

Localization must be active BEFORE navigation. In `nav2_bringup`, this is handled by separate launch files that start the localization manager first.

## Common Failures

### Bond Timeout on Startup
**Symptom**: `"Server xyz did not respond to bond request"` in logs.
**Cause**: The node takes longer than `bond_timeout` to finish activation. When loading large costmaps, complex plugins, or slow TF lookups, activation can exceed 4 seconds.
**Fix**: Increase `bond_timeout` to 10.0 or higher.

### Cascade Shutdown
**Symptom**: All navigation suddenly stops. Logs show `"Lifecycle node xyz is not active"`.
**Cause**: One server crashed or lost its bond. The lifecycle manager shuts down all managed nodes.
**Fix**: Check which node crashed first, fix the root cause, and ensure `attempt_respawn_reconnection: true`.

### Node Not Found
**Symptom**: `"Lifecycle node xyz was not found"` on startup.
**Cause**: The node name in `node_names` doesn't match the actual node name. Remember that node names can be remapped in launch files.
**Fix**: Run `ros2 node list` to verify actual node names and match them exactly.

### Checking Current State
```bash
# Check if navigation is active
ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger

# Get lifecycle state of a specific node
ros2 lifecycle get /controller_server
```

## Health Monitoring in Code

From a Python node, you can check if Nav2 is ready:

```python
from nav2_simple_commander.robot_navigator import BasicNavigator

nav = BasicNavigator()
nav.waitUntilNav2Active()  # Blocks until lifecycle managers report all nodes active
```

This calls the `is_active` service on both lifecycle managers and blocks until both return true.
