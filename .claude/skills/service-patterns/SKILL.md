<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: service-patterns
description: 'Implement ROS 2 service servers and clients. Use when creating request/response communication patterns.'
---

# Role
You are an expert in ROS 2 service server/client patterns. You guide correct synchronous and asynchronous service usage, avoiding deadlocks, in ROS 2 Jazzy/Rolling.

## Service Server (Python)
```python
from example_interfaces.srv import AddTwoInts

class AddServer(Node):
    def __init__(self):
        super().__init__('add_server')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.callback)

    def callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

## Service Client — Async (Python, PREFERRED)
```python
class AddClient(Node):
    def __init__(self):
        super().__init__('add_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available')
            return None

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client.call_async(request)
        future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Result: {result.sum}')
```

## Service Client — Spinning Until Done (one-shot scripts)
```python
def main():
    rclpy.init()
    node = rclpy.create_node('add_client')
    client = node.create_client(AddTwoInts, 'add_two_ints')

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service not available')
        return

    request = AddTwoInts.Request()
    request.a = 3
    request.b = 5

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f'Result: {future.result().sum}')

    node.destroy_node()
    rclpy.shutdown()
```

## C++ Service Server
```cpp
#include "example_interfaces/srv/add_two_ints.hpp"

class AddServer : public rclcpp::Node {
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
public:
  AddServer() : Node("add_server") {
    service_ = create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints",
      [this](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> req,
             std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> res) {
        res->sum = req->a + req->b;
        RCLCPP_INFO(get_logger(), "%ld + %ld = %ld", req->a, req->b, res->sum);
      });
  }
};
```

## C++ Service Client (Async)
```cpp
auto client = create_client<AddTwoInts>("add_two_ints");
if (!client->wait_for_service(std::chrono::seconds(5))) {
  RCLCPP_ERROR(get_logger(), "Service not available");
  return;
}
auto request = std::make_shared<AddTwoInts::Request>();
request->a = 3; request->b = 5;

auto future = client->async_send_request(request,
  [this](rclcpp::Client<AddTwoInts>::SharedFuture result) {
    RCLCPP_INFO(get_logger(), "Sum: %ld", result.get()->sum);
  });
```

## Calling a Service from Within a Callback
```python
# CORRECT: Use call_async + add_done_callback
def timer_callback(self):
    if self.client.service_is_ready():
        future = self.client.call_async(request)
        future.add_done_callback(self.service_done)

def service_done(self, future):
    result = future.result()
    # Process result

# ALSO CORRECT: Use a separate callback group + different executor thread
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self._cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(
            AddTwoInts, 'add', callback_group=self._cb_group)
        self.timer = self.create_timer(
            1.0, self.timer_cb, callback_group=MutuallyExclusiveCallbackGroup())

    def timer_cb(self):
        # With MultiThreadedExecutor and separate callback groups,
        # spin_until_future_complete won't deadlock
        future = self.client.call_async(request)
        # But still prefer async pattern over blocking
```

## When to Use Services vs Topics vs Actions
| Pattern | Use Case | Characteristics |
|---------|----------|-----------------|
| **Topic** | Continuous data streams (sensors, commands) | Pub/sub, async, fire-and-forget |
| **Service** | Quick request/reply (get state, set config) | Synchronous semantics, blocking client |
| **Action** | Long-running tasks (navigate, dock) | Goal + feedback + result + cancel |

### Decision Guide:
- Latency < 1 second and no progress tracking → **Service**
- Continuous streaming → **Topic**
- Takes > 1 second OR needs feedback/cancelation → **Action**
- Configuration changes → **Service** or **Parameters**

## Critical Warnings
- **NEVER call `client.call()` synchronously inside a callback on the same executor thread**. This WILL deadlock because the executor is blocked waiting for the service response, but it can't process the response because it's blocked. This is the #1 service bug in ROS 2.
- **`call_async` is always safe**: Use `call_async` + callbacks or `spin_until_future_complete` from a non-callback context.
- **wait_for_service timeout**: Always pass a timeout to `wait_for_service()`. Without it, the call blocks forever if the server isn't available.
- **Service QoS**: Services use `RELIABLE` + `VOLATILE` internally. You generally cannot change service QoS.
- **No multicast**: A service request goes to exactly one server. If multiple servers exist on the same service name, behavior is undefined. Use topics for broadcast patterns.
- **Single-threaded deadlock**: Even with `call_async`, if you use `rclpy.spin_until_future_complete` inside a callback on a `SingleThreadedExecutor`, you deadlock. The executor is already spinning.
- **Service server callback must return quickly**: The callback runs in the executor thread. Long-running callbacks block other callbacks. For heavy work, dispatch to a separate thread.
