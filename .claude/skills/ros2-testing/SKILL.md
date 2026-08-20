<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: ros2-testing
description: 'Write tests for ROS 2 nodes. Use when creating unit tests with pytest, gtest, or launch_testing integration tests.'
---

# Role
You are an expert in ROS 2 testing with launch_testing, pytest, and ament build system integration. You guide correct test design for unit tests, integration tests, and launch-based system tests in ROS 2 Jazzy/Rolling.

## Unit Testing with pytest (Python Nodes)
```python
# test/test_my_module.py
import pytest
from my_package.my_module import compute_distance, validate_waypoint

def test_compute_distance():
    assert compute_distance(0, 0, 3, 4) == pytest.approx(5.0)

def test_compute_distance_same_point():
    assert compute_distance(1, 1, 1, 1) == 0.0

def test_validate_waypoint_valid():
    assert validate_waypoint({'x': 1.0, 'y': 2.0}) is True

def test_validate_waypoint_missing_field():
    assert validate_waypoint({'x': 1.0}) is False
```

### setup.cfg for pytest
```ini
[tool:pytest]
testpaths = test
```

### package.xml test dependencies
```xml
<test_depend>ament_cmake_pytest</test_depend>
<test_depend>python3-pytest</test_depend>
```

## Integration Testing with launch_testing
```python
# test/test_node_integration.py
import unittest
import launch
import launch_ros
import launch_testing
import launch_testing.actions
import rclpy
from std_msgs.msg import String

def generate_test_description():
    """Launch the node under test."""
    node = launch_ros.actions.Node(
        package='my_package',
        executable='my_node',
        name='test_node',
        parameters=[{'rate': 10.0}],
        output='screen',
    )
    return (
        launch.LaunchDescription([
            node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'node': node}
    )

class TestMyNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_helper')

    def tearDown(self):
        self.node.destroy_node()

    def test_node_publishes(self):
        """Verify the node publishes on expected topic."""
        received = []

        sub = self.node.create_subscription(
            String, 'output', lambda msg: received.append(msg), 10)

        # Spin for up to 5 seconds waiting for messages
        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=5)
        while len(received) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.node.get_clock().now() > end_time:
                break

        self.assertGreater(len(received), 0, 'No messages received on /output')

    def test_parameter_exists(self):
        """Check that the node has expected parameters."""
        # Use a parameter client
        from rcl_interfaces.srv import ListParameters
        client = self.node.create_client(
            ListParameters, 'test_node/list_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))

@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        """Verify clean shutdown."""
        launch_testing.asserts.assertExitCodes(proc_info)
```

## CMakeLists.txt Test Integration
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Unit tests
  ament_add_pytest_test(test_my_module test/test_my_module.py)

  # Integration tests
  add_launch_test(test/test_node_integration.py
    TIMEOUT 60
  )
endif()
```

## C++ Unit Testing with gtest
```cpp
// test/test_utils.cpp
#include <gtest/gtest.h>
#include "my_package/utils.hpp"

TEST(UtilsTest, ComputeDistance) {
  EXPECT_DOUBLE_EQ(compute_distance(0, 0, 3, 4), 5.0);
}

TEST(UtilsTest, ComputeDistanceSamePoint) {
  EXPECT_DOUBLE_EQ(compute_distance(1, 1, 1, 1), 0.0);
}
```

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_utils test/test_utils.cpp)
  target_link_libraries(test_utils my_library)
  ament_target_dependencies(test_utils rclcpp)
endif()
```

## Testing Node Existence and Topics
```python
def test_node_is_running(self):
    """Check that the node appears in the graph."""
    node_names = self.node.get_node_names()
    self.assertIn('test_node', node_names)

def test_topic_exists(self):
    """Check that expected topics are advertised."""
    topics = self.node.get_topic_names_and_types()
    topic_names = [t[0] for t in topics]
    self.assertIn('/output', topic_names)
```

## Running Tests
```bash
# Build with tests
colcon build --symlink-install

# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select my_package

# Show test results
colcon test-result --verbose

# Run pytest directly for faster iteration
cd src/my_package
python -m pytest test/ -v
```

## Test Fixtures Pattern
```python
@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def test_node(ros_context):
    node = rclpy.create_node('test_helper')
    yield node
    node.destroy_node()

def test_something(test_node):
    assert test_node.get_name() == 'test_helper'
```

## Code Coverage
```bash
# Python coverage
colcon test --packages-select my_package \
    --pytest-args --cov=my_package --cov-report=html

# C++ coverage (with lcov)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_C_FLAGS="--coverage"
colcon test
lcov --capture --directory build/ --output-file coverage.info
genhtml coverage.info --output-directory coverage_html
```

## Critical Warnings
- **Timing-dependent tests are flaky**: Tests that rely on "wait N seconds for messages" are inherently fragile. Use polling loops with timeouts instead of fixed-duration sleeps.
- **rclpy.init() in tests**: Call `rclpy.init()` in `setUpClass` and `rclpy.shutdown()` in `tearDownClass`. Calling them per-test method causes "already initialized" errors.
- **launch_testing ReadyToTest**: The `ReadyToTest()` action must be in the launch description. Without it, tests start before nodes are ready.
- **Test isolation**: Each test should be independent. Don't rely on message state from a previous test.
- **Colcon test timeout**: Default timeout is 60 seconds. Increase with `TIMEOUT` in `add_launch_test` for slow integration tests.
- **Don't test ROS internals**: Test YOUR code's behavior (publishes correct messages, handles errors), not that rclcpp/rclpy work correctly.
- **Sim time in tests**: If your node uses `use_sim_time`, you must either publish a `/clock` topic in the test or set `use_sim_time=false` for testing.
