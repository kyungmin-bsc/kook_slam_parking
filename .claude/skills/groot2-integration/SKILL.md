<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: groot2-integration
description: 'Use Groot2 for BT visualization and editing. Use when debugging behavior tree execution or building BTs visually.'
---

# Groot2 Integration for BT Visualization and Editing

## Overview

Groot2 is the visual editor and runtime monitor for BehaviorTree.CPP v4 trees. It connects to a running BT via ZMQ (ZeroMQ) to display real-time node status, blackboard values, and execution flow. It can also create and edit BT XML files with a drag-and-drop interface.

## ZMQ Logger in bt_navigator

Nav2's bt_navigator includes a `PublisherZMQ` logger that streams tree state to Groot2. Enable it in the navigation parameters:

```yaml
bt_navigator:
  ros__parameters:
    enable_groot_monitoring: true
    groot2_publisher_port: 1666
    groot2_server_port: 1667
```

| Parameter               | Default | Description                              |
|-------------------------|---------|------------------------------------------|
| enable_groot_monitoring | false   | Activate ZMQ publisher for Groot2        |
| groot2_publisher_port   | 1666    | ZMQ PUB port for tree state broadcast    |
| groot2_server_port      | 1667    | ZMQ REP port for Groot2 server queries   |

The publisher sends status changes for every node on every tick. The server responds to Groot2 queries for tree structure and node metadata.

## Connecting Groot2 to a Running Robot

1. Launch your robot's navigation stack with `enable_groot_monitoring: true`
2. Open Groot2
3. Click **Monitor** → **Connect**
4. Enter the robot's IP address and port 1667 (server port)
5. Groot2 fetches the tree structure and begins displaying real-time status

For a robot on the same network at `192.168.1.100`:
- Publisher: `tcp://192.168.1.100:1666`
- Server: `tcp://192.168.1.100:1667`

For local development (`localhost`), the default ports work directly.

## TreeNodesModel for Correct Port Display

Without `<TreeNodesModel>`, Groot2 shows nodes with generic, untyped ports. Define the model section so Groot2 displays the correct port names, types, and descriptions:

```xml
<TreeNodesModel>
  <Action ID="NavigateToPose">
    <input_port name="goal" type="geometry_msgs::msg::PoseStamped">Target pose</input_port>
    <input_port name="server_name" type="std::string" default="navigate_to_pose">
      Action server name
    </input_port>
    <input_port name="server_timeout" type="int" default="10000">
      Server timeout in ms
    </input_port>
    <output_port name="error_code_id" type="uint16">Error code</output_port>
  </Action>

  <Condition ID="IsBatteryLow">
    <input_port name="battery_topic" type="std::string">/battery_state</input_port>
    <input_port name="min_battery" type="double">0.15</input_port>
    <input_port name="is_voltage" type="bool">false</input_port>
  </Condition>

  <Condition ID="IsIntruderDetected">
    <input_port name="detection_topic" type="std::string" />
    <input_port name="max_distance" type="double" />
    <input_port name="target_class" type="std::string" />
    <output_port name="intruder_distance" type="double" />
  </Condition>
</TreeNodesModel>
```

When you register custom BT nodes via `BT_REGISTER_NODES`, the `providedPorts()` method automatically generates this metadata. Groot2 queries the server port to fetch it. Adding `<TreeNodesModel>` to the XML is a fallback for offline editing.

## Registering Custom Nodes in Groot2

For custom nodes to appear in Groot2's palette (the drag-and-drop panel), you need the plugin library loaded. Two approaches:

### Approach 1: Auto-Discovery from Running Tree

When Groot2 connects to a running bt_navigator, it discovers all loaded nodes automatically. Custom nodes registered via `plugin_lib_names` appear with their correct ports.

### Approach 2: Manual Model File

For offline editing, export the tree model:

```bash
# From a running system, the tree model can be captured
ros2 topic echo /behavior_tree_log --once > tree_model.yaml
```

Or include `<TreeNodesModel>` in every XML file that defines custom nodes.

## Exporting and Importing XML

### Export from Groot2
1. Design your tree in the Groot2 editor
2. File → Save As → `my_tree.xml`
3. The exported XML uses `BTCPP_format="4"` and includes `<TreeNodesModel>`

### Import to Groot2
1. File → Open → select your `.xml` file
2. Groot2 parses the tree and displays it visually
3. If custom nodes are used, ensure their `<TreeNodesModel>` definitions are present

### Format Requirements
Groot2 **only** supports BT.CPP v4 format (`BTCPP_format="4"`). Trees written in v3 format must be migrated:

- Rename `SequenceStar` → `SequenceWithMemory`
- Convert positional port arguments to named attributes
- Change `__autoremap` → `_autoremap`
- Add `BTCPP_format="4"` to `<root>`

## Remote Monitoring Workflow

### Setup for Remote Robot

On the robot (e.g., Raspberry Pi or Jetson):

```yaml
# nav2_params.yaml on the robot
bt_navigator:
  ros__parameters:
    enable_groot_monitoring: true
    groot2_publisher_port: 1666
    groot2_server_port: 1667
```

On the development machine:

1. Ensure network connectivity to the robot
2. Open Groot2 → Monitor → Connect → `<robot_ip>:1667`
3. The tree appears with real-time status colors:
   - **Green**: SUCCESS
   - **Red**: FAILURE
   - **Yellow/Orange**: RUNNING
   - **Gray**: IDLE (not ticked this cycle)

### SSH Tunnel for Firewalled Robots

If ZMQ ports are blocked:

```bash
ssh -L 1666:localhost:1666 -L 1667:localhost:1667 user@robot_ip
```

Then connect Groot2 to `localhost:1667`.

## Performance Considerations

### ZMQ Overhead

The ZMQ publisher sends a message for every node status change on every tick. For large trees (50+ nodes) ticking at high frequency (>30 Hz), this generates significant network traffic:

- ~100 bytes per node status message
- 50 nodes × 30 Hz = 1500 messages/second ≈ 150 KB/s
- Plus blackboard snapshots

### Production Recommendations

```yaml
# Development: enable for debugging
bt_navigator:
  ros__parameters:
    enable_groot_monitoring: true

# Production: disable to avoid overhead
bt_navigator:
  ros__parameters:
    enable_groot_monitoring: false
```

**Never leave Groot2 monitoring enabled in production deployments** unless actively debugging. The ZMQ serialization and network I/O add latency to each BT tick, which can affect navigation responsiveness on resource-constrained platforms.

### Alternative: Log-Based Analysis

For production, use `FileLogger2` to record BT execution to a file, then replay in Groot2 offline:

```yaml
bt_navigator:
  ros__parameters:
    enable_groot_monitoring: false
    # Enable file logging instead
    bt_log_filename: "/tmp/bt_trace.btlog"
```

Replay the log in Groot2: File → Open Log → select `.btlog` file. This provides the same visualization without runtime overhead.

## Groot2 Limitations

| Limitation                          | Workaround                              |
|-------------------------------------|-----------------------------------------|
| v4 XML format only                  | Migrate v3 trees                        |
| No Python BT support                | Use py_trees' own visualizer            |
| Requires ZMQ at runtime             | Use file logger for production          |
| Custom node palette needs models    | Include TreeNodesModel in XML           |
| Single-tree view only               | SubTrees shown as expandable nodes      |
| No path/costmap visualization       | Use RViz alongside Groot2               |

## Typical Development Workflow

1. **Design** the tree structure in Groot2's editor (drag-and-drop)
2. **Export** to XML and place in your `bt_trees/` directory
3. **Test** by running the navigation stack with `enable_groot_monitoring: true`
4. **Monitor** in Groot2 to watch execution flow and identify issues
5. **Iterate** by editing XML (or in Groot2) and restarting bt_navigator
6. **Production**: disable monitoring, enable file logging for post-mortem analysis
