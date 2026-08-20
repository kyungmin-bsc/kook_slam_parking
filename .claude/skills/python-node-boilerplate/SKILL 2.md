<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: python-node-boilerplate
description: 'Create ROS 2 Python rclpy nodes. Use when writing a new Python node, subscriber, publisher, or timer callback.'
---

# Role
You are an expert in writing ROS 2 Python (rclpy) nodes. You produce correct, idiomatic node implementations following best practices for ROS 2 Jazzy/Rolling.

## Minimal Node Pattern
```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node started')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Production Node Pattern
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

class ProductionNode(Node):
    def __init__(self):
        super().__init__('production_node')

        # Declare parameters with descriptors and constraints
        self.declare_parameter('update_rate', 10.0, ParameterDescriptor(
            description='Publishing rate in Hz',
            floating_point_range=[FloatingPointRange(
                from_value=0.1, to_value=100.0, step=0.1
            )]
        ))
        self.declare_parameter('topic_name', 'output', ParameterDescriptor(
            description='Output topic name', read_only=True
        ))

        rate = self.get_parameter('update_rate').value
        topic = self.get_parameter('topic_name').value

        # Dynamic parameter callback
        self.add_on_set_parameters_callback(self._param_callback)

        # Callback groups for concurrency control
        self._cb_group_timer = MutuallyExclusiveCallbackGroup()
        self._cb_group_sub = ReentrantCallbackGroup()

        # QoS profile
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # Publisher, subscriber, timer
        self._pub = self.create_publisher(String, topic, 10)
        self._sub = self.create_subscription(
            String, 'input', self._sub_callback, sensor_qos,
            callback_group=self._cb_group_sub
        )
        self._timer = self.create_timer(
            1.0 / rate, self._timer_callback,
            callback_group=self._cb_group_timer
        )
        self._count = 0

    def _param_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name == 'update_rate':
                self._timer.cancel()
                self._timer = self.create_timer(1.0 / param.value, self._timer_callback)
        return SetParametersResult(successful=True)

    def _timer_callback(self):
        msg = String()
        msg.data = f'Count: {self._count}'
        self._pub.publish(msg)
        self._count += 1

    def _sub_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ProductionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Rules and Best Practices
- Always call `rclpy.init()` before creating any node. Forgetting this produces cryptic errors.
- Always call `rclpy.shutdown()` in a finally block. Failing to do so leaks resources and can hang.
- Use class-based nodes, not standalone publisher/subscriber functions.
- Declare all parameters in `__init__`. Do not use `get_parameter` without prior `declare_parameter`.
- Use `try/except KeyboardInterrupt` around spin for clean Ctrl+C handling.
- Never call `rclpy.spin()` from within a callback — it will deadlock.
- Use `destroy_node()` before `rclpy.shutdown()`.

## Critical Warnings
- **GIL and MultiThreadedExecutor**: Python's GIL means callbacks in a MultiThreadedExecutor do NOT truly run in parallel for CPU-bound work. They only help with I/O-bound or blocking calls. For CPU-heavy work, use separate processes or C++ nodes.
- **Timer drift**: `create_timer` does not guarantee exact periodicity; it schedules the next callback after the period elapses.
- **Callback ordering**: With `SingleThreadedExecutor` (default `rclpy.spin`), callbacks are serialized. If one callback blocks, all others stall.
- **setup.py entry_points**: Ensure your `console_scripts` entry point calls `main` — colcon will not find nodes without this.

## Entry Point in setup.py
```python
entry_points={
    'console_scripts': [
        'my_node = my_package.my_module:main',
    ],
},
```

## Parameter YAML Loading
```yaml
# config/params.yaml
production_node:
  ros__parameters:
    update_rate: 20.0
    topic_name: /robot/output
```
```python
# In launch file
Node(
    package='my_package',
    executable='my_node',
    parameters=[os.path.join(pkg_share, 'config', 'params.yaml')]
)
```
