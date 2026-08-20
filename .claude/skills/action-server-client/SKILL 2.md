<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: action-server-client
description: 'Implement ROS 2 action servers and clients. Use when creating long-running tasks with feedback and cancellation support.'
---

# Role
You are an expert in ROS 2 action servers and clients. You guide correct implementation of long-running tasks with goal tracking, feedback, cancellation, and preemption in ROS 2 Jazzy/Rolling.

## Action Definition (.action)
```
# action/Patrol.action
# Goal
geometry_msgs/PoseStamped[] waypoints
float64 speed
---
# Result
bool success
uint32 waypoints_completed
float64 total_distance
---
# Feedback
uint32 current_waypoint_index
geometry_msgs/PoseStamped current_pose
float64 distance_remaining
```

## Python Action Server
```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from my_interfaces.action import Patrol
import time

class PatrolServer(Node):
    def __init__(self):
        super().__init__('patrol_server')
        self._action_server = ActionServer(
            self,
            Patrol,
            'patrol',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
        )

    def goal_callback(self, goal_request):
        """Decide whether to accept or reject the goal."""
        self.get_logger().info(f'Received goal with {len(goal_request.waypoints)} waypoints')
        if len(goal_request.waypoints) == 0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Decide whether to accept or reject the cancel request."""
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """Called when goal is accepted. Start execution in a thread."""
        goal_handle.execute()  # runs execute_callback

    async def execute_callback(self, goal_handle):
        """Main execution loop — runs in executor thread."""
        self.get_logger().info('Executing patrol...')
        feedback = Patrol.Feedback()
        result = Patrol.Result()
        waypoints = goal_handle.request.waypoints

        for i, wp in enumerate(waypoints):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.waypoints_completed = i
                self.get_logger().info('Goal canceled')
                return result

            # Simulate navigation to waypoint
            feedback.current_waypoint_index = i
            feedback.current_pose = wp
            feedback.distance_remaining = float(len(waypoints) - i)
            goal_handle.publish_feedback(feedback)

            # Do actual work here (navigate, etc.)
            await asyncio.sleep(1.0)  # or use blocking call

        goal_handle.succeed()
        result.success = True
        result.waypoints_completed = len(waypoints)
        return result
```

## Python Action Client
```python
from rclpy.action import ActionClient

class PatrolClient(Node):
    def __init__(self):
        super().__init__('patrol_client')
        self._client = ActionClient(self, Patrol, 'patrol')

    def send_goal(self, waypoints):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return

        goal = Patrol.Goal()
        goal.waypoints = waypoints
        goal.speed = 0.5

        future = self._client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Waypoint {fb.current_waypoint_index}, '
                                f'dist remaining: {fb.distance_remaining:.1f}')

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Done: success={result.success}, '
                                f'completed={result.waypoints_completed}')

    def cancel_goal(self, goal_handle):
        """Cancel a running goal."""
        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal cancel accepted')
```

## C++ Action Server (Key Pattern)
```cpp
#include "rclcpp_action/rclcpp_action.hpp"

using Patrol = my_interfaces::action::Patrol;
using GoalHandle = rclcpp_action::ServerGoalHandle<Patrol>;

rclcpp_action::Server<Patrol>::SharedPtr server_;

server_ = rclcpp_action::create_server<Patrol>(
  this, "patrol",
  // handle_goal
  [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Patrol::Goal> goal) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  },
  // handle_cancel
  [this](const std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  },
  // handle_accepted — start execution in a new thread
  [this](const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  });

void execute(const std::shared_ptr<GoalHandle> goal_handle) {
  auto feedback = std::make_shared<Patrol::Feedback>();
  auto result = std::make_shared<Patrol::Result>();

  for (size_t i = 0; i < goal_handle->get_goal()->waypoints.size(); ++i) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }
    feedback->current_waypoint_index = i;
    goal_handle->publish_feedback(feedback);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  result->success = true;
  goal_handle->succeed(result);
}
```

## Preemption Pattern
ROS 2 actions do NOT auto-preempt. To preempt an old goal with a new one:
```python
def goal_callback(self, goal_request):
    if self._current_goal_handle is not None and self._current_goal_handle.is_active:
        self.get_logger().info('Preempting current goal')
        self._current_goal_handle.abort()
    return GoalResponse.ACCEPT
```

## Critical Warnings
- **send_goal_async, not send_goal**: Always use `send_goal_async()`. Synchronous `send_goal()` blocks the executor and can deadlock.
- **Feedback is optional**: You don't have to publish feedback, but clients expecting it will time out.
- **Goal handle lifecycle**: Store the goal handle if you need to cancel later. It's only valid while the goal is active.
- **Thread safety in execute**: The execute callback may run in a different thread. Protect shared state with locks.
- **Timeout handling**: Clients should implement timeouts. Use `asyncio.wait_for` or check timestamps in feedback callbacks to detect stalled servers.
- **No auto-preemption**: Unlike ROS 1 `actionlib`, ROS 2 does NOT automatically preempt running goals. You must implement preemption logic in `goal_callback` or `handle_accepted_callback`.
- **Action names**: The action name creates three hidden topics (`_action/send_goal`, `_action/cancel_goal`, `_action/get_result`) and a feedback topic. Namespace them appropriately.
