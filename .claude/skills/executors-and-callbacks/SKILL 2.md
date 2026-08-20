<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: executors-and-callbacks
description: 'Configure ROS 2 executors and callback groups. Use when managing single-threaded, multi-threaded, or custom executors.'
---

# Role
You are an expert in ROS 2 executors, callback groups, and concurrency patterns. You guide correct threading, callback scheduling, and spin patterns in ROS 2 Jazzy/Rolling.

## Executor Types

### SingleThreadedExecutor (Default)
```python
rclpy.spin(node)  # uses SingleThreadedExecutor internally
# OR explicitly:
executor = SingleThreadedExecutor()
executor.add_node(node)
executor.spin()
```
- All callbacks run sequentially on one thread.
- Simplest model — no thread-safety concerns.
- A slow callback blocks ALL other callbacks (timers, subscriptions, services).

### MultiThreadedExecutor
```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
```
- Callbacks can run concurrently on multiple threads.
- REQUIRES careful use of callback groups to control concurrency.
- Default thread count = number of CPU cores.

### StaticSingleThreadedExecutor (C++)
```cpp
rclcpp::executors::StaticSingleThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```
- Single-threaded like default, but with lower overhead.
- Callbacks are resolved at `add_node` time — don't add/remove subscriptions after.
- Best for embedded/real-time scenarios with fixed callback set.

## Callback Groups

### MutuallyExclusiveCallbackGroup
```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

group = MutuallyExclusiveCallbackGroup()
self.timer = self.create_timer(0.1, self.timer_cb, callback_group=group)
self.sub = self.create_subscription(Msg, 'topic', self.sub_cb, 10, callback_group=group)
# timer_cb and sub_cb NEVER run concurrently (even with MultiThreadedExecutor)
```

### ReentrantCallbackGroup
```python
from rclpy.callback_groups import ReentrantCallbackGroup

group = ReentrantCallbackGroup()
self.timer = self.create_timer(0.1, self.timer_cb, callback_group=group)
self.sub = self.create_subscription(Msg, 'topic', self.sub_cb, 10, callback_group=group)
# With MultiThreadedExecutor: timer_cb and sub_cb CAN run concurrently
# Multiple instances of the SAME callback CAN also run concurrently!
```

## Default Callback Group
If you don't specify a callback group, all callbacks go into the node's default `MutuallyExclusiveCallbackGroup`. This means with `MultiThreadedExecutor`, callbacks in different groups can run concurrently, but callbacks in the same (default) group are still serialized.

## Concurrency Control Patterns

### Pattern 1: Isolate service client from timer
```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Timer and service client in separate groups
        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()

        self.client = self.create_client(
            SetBool, 'service', callback_group=self._service_group)
        self.timer = self.create_timer(
            1.0, self.timer_cb, callback_group=self._timer_group)

    def timer_cb(self):
        # Safe: timer_cb runs in timer_group, service response 
        # can be processed in service_group on another thread
        future = self.client.call_async(request)
        future.add_done_callback(self.service_done)
```

### Pattern 2: Thread-safe shared state
```python
import threading

class SafeNode(Node):
    def __init__(self):
        super().__init__('safe_node')
        self._lock = threading.Lock()
        self._data = []

        reentrant = ReentrantCallbackGroup()
        self.sub = self.create_subscription(
            Msg, 'topic', self.cb, 10, callback_group=reentrant)

    def cb(self, msg):
        with self._lock:
            self._data.append(msg.data)
```

## C++ Callback Groups
```cpp
auto group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
auto sub_options = rclcpp::SubscriptionOptions();
sub_options.callback_group = group;

sub_ = create_subscription<Msg>("topic", 10, callback, sub_options);
timer_ = create_wall_timer(100ms, timer_cb, group);
```

## Spinning Patterns
```python
# Spin forever (blocking)
rclpy.spin(node)

# Spin once (process one ready callback)
rclpy.spin_once(node, timeout_sec=0.1)

# Spin until future completes (for one-shot scripts)
rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

# Manual spin loop
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.01)
    # Do other work between spins
```

## Multiple Nodes, One Executor
```python
executor = MultiThreadedExecutor()
executor.add_node(node_a)
executor.add_node(node_b)
executor.add_node(node_c)
executor.spin()  # all nodes share the thread pool
```

## Critical Warnings
- **Python GIL**: With `MultiThreadedExecutor` in Python, callbacks run on OS threads but the GIL ensures only one thread executes Python bytecode at a time. This means CPU-bound callbacks get NO parallelism. Use `MultiThreadedExecutor` in Python only for I/O-bound work (service calls, TF lookups, file I/O).
- **Reentrant callback safety**: If you use `ReentrantCallbackGroup`, the same callback function can run simultaneously on multiple threads. You MUST use locks to protect shared state.
- **Default group is MutuallyExclusive**: If all your callbacks are in the default group and you use `MultiThreadedExecutor`, they still run serially (just on potentially different threads). You must create explicit groups for concurrency.
- **spin_once in callbacks**: Never call `rclpy.spin_once` or `rclpy.spin_until_future_complete` inside a callback when using `SingleThreadedExecutor` — it will deadlock.
- **Executor thread count**: `MultiThreadedExecutor(num_threads=0)` means "auto" (CPU count). For I/O-bound workloads, you might want more threads than cores.
- **Timer + slow callback**: With `SingleThreadedExecutor`, if a callback takes 500ms and your timer period is 100ms, timer callbacks queue up and fire as fast as possible when the executor gets back to them. They do NOT accumulate — only one pending timer callback is queued at a time.
- **Adding/removing nodes from executor**: Thread-safe in `MultiThreadedExecutor`, but NOT in `StaticSingleThreadedExecutor`.
