<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: custom-bt-action-python
description: 'Create custom BT action nodes in Python. Use when implementing BT nodes using py_trees or Python BT plugins.'
---

# Custom BT Action Nodes in Python

## Python BT Options for Nav2

There are three approaches to Python-based behavior tree nodes with Nav2:

1. **BehaviorTree.CPP Python bindings** — Limited v4 support, not production-recommended
2. **py_trees / py_trees_ros** — Mature Python BT framework, runs alongside Nav2
3. **Custom Nav2 BT plugin via Python wrapper** — C++ shim calling Python via pybind11

For production Nav2 deployments, C++ BT nodes are strongly recommended. Python BT nodes are best for prototyping, testing, and non-realtime tasks.

## py_trees_ros Approach

py_trees is the most mature Python BT library for ROS 2. It runs as a separate tree executor that interacts with Nav2 via action clients and topics.

### Installation

```bash
sudo apt install ros-jazzy-py-trees ros-jazzy-py-trees-ros ros-jazzy-py-trees-ros-interfaces
```

### Basic py_trees Action Node

```python
import py_trees
import py_trees_ros
import rclpy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavigateToGoal(py_trees_ros.action_clients.FromBlackboard):
    """Navigate to a pose using Nav2's NavigateToPose action."""

    def __init__(self, name: str, action_name: str = "navigate_to_pose"):
        super().__init__(
            name=name,
            action_type=NavigateToPose,
            action_name=action_name,
            key="goal",  # blackboard key for the goal
            generate_feedback_message=self._feedback_msg,
        )

    def _feedback_msg(self, msg) -> str:
        feedback = msg.feedback
        return f"Distance remaining: {feedback.distance_remaining:.2f}m"
```

### Custom py_trees Condition Node

```python
import py_trees
from sensor_msgs.msg import BatteryState


class IsBatteryLow(py_trees.behaviour.Behaviour):
    """Check if robot battery is below threshold."""

    def __init__(self, name: str, threshold: float = 0.15):
        super().__init__(name=name)
        self.threshold = threshold
        self.battery_level = None
        self.sub = None

    def setup(self, **kwargs):
        """Create ROS 2 subscription during setup phase."""
        try:
            self.node = kwargs["node"]
        except KeyError:
            raise KeyError("Node not provided in setup kwargs")

        self.sub = self.node.create_subscription(
            BatteryState,
            "/battery_state",
            self._battery_callback,
            10,
        )
        self.logger.info(f"Subscribed to /battery_state, threshold={self.threshold}")

    def _battery_callback(self, msg: BatteryState):
        self.battery_level = msg.percentage

    def update(self) -> py_trees.common.Status:
        if self.battery_level is None:
            self.feedback_message = "No battery data received yet"
            return py_trees.common.Status.FAILURE

        if self.battery_level < self.threshold:
            self.feedback_message = (
                f"Battery LOW: {self.battery_level:.1%} < {self.threshold:.1%}"
            )
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = (
                f"Battery OK: {self.battery_level:.1%} >= {self.threshold:.1%}"
            )
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        pass
```

### Building a py_trees Patrol Tree

```python
import py_trees
import py_trees_ros


def create_patrol_tree() -> py_trees.behaviour.Behaviour:
    """Create a patrol behavior tree using py_trees."""

    # Root: priority selector
    root = py_trees.composites.Selector(name="Root", memory=False)

    # High priority: battery check
    battery_guard = py_trees.composites.Sequence(
        name="BatteryGuard", memory=True
    )
    battery_guard.add_children([
        IsBatteryLow(name="CheckBattery", threshold=0.15),
        NavigateToGoal(name="GoToCharger"),
    ])

    # Normal priority: patrol waypoints
    patrol = py_trees.composites.Sequence(name="Patrol", memory=True)

    waypoints = py_trees.composites.Sequence(name="Waypoints", memory=True)
    for i, pose in enumerate(get_patrol_poses()):
        set_goal = py_trees.behaviours.SetBlackboardVariable(
            name=f"SetGoal_{i}",
            variable_name="goal",
            variable_value=create_nav_goal(pose),
            overwrite=True,
        )
        navigate = NavigateToGoal(name=f"NavTo_{i}")
        waypoints.add_children([set_goal, navigate])

    patrol.add_children([waypoints])

    root.add_children([battery_guard, patrol])
    return root


def main():
    rclpy.init()

    root = create_patrol_tree()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True,
    )

    try:
        tree.setup(timeout=15.0)
        tree.tick_tock(period_ms=500)
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()
```

### py_trees Lifecycle

| Phase        | Method          | Purpose                                  |
|--------------|-----------------|------------------------------------------|
| Construction | `__init__()`    | Store parameters, no ROS interactions    |
| Setup        | `setup()`       | Create subscriptions, action clients     |
| Tick         | `update()`      | Return SUCCESS, FAILURE, or RUNNING      |
| Halt         | `terminate()`   | Clean up when node is halted             |

## BehaviorTree.CPP Python Bindings

BT.CPP v4 has experimental Python bindings, but they are not well-integrated with Nav2's plugin system:

```python
import btpy  # BehaviorTree.CPP Python bindings

class MyAction(btpy.SyncActionNode):
    @staticmethod
    def provided_ports():
        return btpy.PortsList([
            btpy.InputPort("message", "Hello"),
        ])

    def tick(self):
        msg = self.get_input("message")
        print(f"Action says: {msg}")
        return btpy.NodeStatus.SUCCESS
```

**Limitations**: No Groot2 support, no Nav2 `BtActionNode` base class, limited type conversion between C++ and Python blackboard entries.

## Hybrid Approach: Python Logic with C++ BT

The recommended pattern for Python-heavy logic with Nav2:

1. Write your detection/AI logic as a **ROS 2 action server in Python**
2. Write a **C++ BT action node** that calls that server
3. The BT remains C++-native for Groot2 compatibility and performance

```python
# Python action server (runs as a separate ROS 2 node)
class IntruderAnalysisServer(Node):
    def __init__(self):
        super().__init__("intruder_analysis_server")
        self._action_server = ActionServer(
            self, AnalyzeScene, "analyze_scene",
            execute_callback=self.execute_callback,
        )

    async def execute_callback(self, goal_handle):
        # Heavy Python logic: ML inference, API calls, etc.
        result = AnalyzeScene.Result()
        result.threat_level = self.run_ml_model(goal_handle.request.image)
        goal_handle.succeed()
        return result
```

```cpp
// C++ BT node that calls the Python action server
class AnalyzeSceneAction
  : public nav2_behavior_tree::BtActionNode<my_interfaces::action::AnalyzeScene>
{
  // ... standard BtActionNode implementation
};
```

## Python vs C++ BT Nodes Comparison

| Aspect                 | C++ (BT.CPP)              | Python (py_trees)           |
|------------------------|---------------------------|-----------------------------|
| Nav2 integration       | Native plugin system      | External tree executor      |
| Groot2 visualization   | Full support              | py_trees has own viewer     |
| Performance            | Microsecond ticks         | Millisecond ticks           |
| GIL impact             | None                      | Blocks during CPU work      |
| Rapid prototyping      | Slower iteration          | Fast iteration              |
| ML/AI integration      | Needs pybind11 or service | Direct Python imports       |
| Type safety            | Compile-time              | Runtime only                |
| Production Nav2        | Recommended               | Use for non-critical tasks  |

## py_trees Visualization

py_trees includes its own tree visualization, separate from Groot2:

```bash
# ASCII tree in terminal
py-trees-tree-viewer

# Web-based viewer
ros2 run py_trees_ros py_trees_tree_viewer
```

The `unicode_tree_debug=True` flag in `BehaviourTree` prints the tree state on every tick to the console, useful for development.

## When to Use Python BT Nodes

- **Prototyping** behavior tree logic before implementing in C++
- **Testing** tree structures and flow without compilation
- **Non-realtime tasks** like sending notifications, logging analytics, calling web APIs
- **ML inference** where the model is Python-only and latency tolerance is >100ms
- **Education** and rapid experimentation with BT concepts

For anything in the Nav2 navigation critical path (planning, control, recovery), use C++.
