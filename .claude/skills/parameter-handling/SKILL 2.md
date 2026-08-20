<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: parameter-handling
description: 'Declare and use ROS 2 parameters. Use when adding declare_parameter, parameter callbacks, or dynamic reconfigure.'
---

# Role
You are an expert in ROS 2 parameter handling. You guide correct parameter declaration, dynamic reconfiguration, YAML loading, and cross-node parameter monitoring in ROS 2 Jazzy/Rolling.

## Parameter Declaration (Python)
```python
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Simple declaration
        self.declare_parameter('robot_name', 'sigyn')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('enable_debug', False)

        # With descriptor and range constraints
        self.declare_parameter('rate', 10.0, ParameterDescriptor(
            description='Update rate in Hz',
            floating_point_range=[FloatingPointRange(
                from_value=0.1, to_value=100.0, step=0.0
            )]
        ))

        # Read-only parameter (cannot be changed at runtime)
        self.declare_parameter('serial_port', '/dev/ttyUSB0', ParameterDescriptor(
            description='Hardware serial port',
            read_only=True
        ))

        # Integer with range
        self.declare_parameter('retry_count', 3, ParameterDescriptor(
            integer_range=[IntegerRange(from_value=0, to_value=10, step=1)]
        ))

        # Access values
        name = self.get_parameter('robot_name').get_parameter_value().string_value
        speed = self.get_parameter('max_speed').value  # shorthand
```

## Dynamic Parameter Callback (Python)
```python
from rcl_interfaces.msg import SetParametersResult

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.declare_parameter('gain', 1.0)
        self.declare_parameter('enabled', True)

        # Register callback for runtime parameter changes
        self.add_on_set_parameters_callback(self._on_param_change)

    def _on_param_change(self, params):
        for param in params:
            if param.name == 'gain':
                if param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='Gain must be non-negative'
                    )
                self._gain = param.value
            elif param.name == 'enabled':
                self._enabled = param.value
        return SetParametersResult(successful=True)
```

## Parameter Declaration (C++)
```cpp
// Simple
this->declare_parameter<double>("max_speed", 1.0);
this->declare_parameter<std::string>("frame_id", "base_link");
this->declare_parameter<std::vector<double>>("waypoints_x", {0.0, 1.0, 2.0});

// With descriptor
rcl_interfaces::msg::ParameterDescriptor desc;
desc.description = "Control gain";
desc.floating_point_range.resize(1);
desc.floating_point_range[0].from_value = 0.0;
desc.floating_point_range[0].to_value = 100.0;
this->declare_parameter("gain", 1.0, desc);

// Read-only
desc.read_only = true;
this->declare_parameter("hardware_id", "motor_01", desc);

// Access
double speed = this->get_parameter("max_speed").as_double();
```

## Dynamic Parameter Callback (C++)
```cpp
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class MyNode : public rclcpp::Node {
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_handle_;

  MyNode() : Node("my_node") {
    this->declare_parameter("gain", 1.0);

    cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &p : params) {
          if (p.get_name() == "gain") {
            if (p.as_double() < 0.0) {
              result.successful = false;
              result.reason = "Gain must be non-negative";
              return result;
            }
            gain_ = p.as_double();
          }
        }
        return result;
      });
  }
};
```

## Loading Parameters from YAML
```yaml
# config/params.yaml
my_node:
  ros__parameters:
    max_speed: 1.5
    robot_name: "sigyn"
    enable_debug: true
    waypoints:
      x: [0.0, 1.0, 2.0]
      y: [0.0, 0.5, 1.0]

# Wildcard namespace applies to any node
/**:
  ros__parameters:
    use_sim_time: false
```

### From Launch File
```python
Node(
    package='my_pkg', executable='my_node', name='my_node',
    parameters=[
        os.path.join(pkg_share, 'config', 'params.yaml'),
        {'override_param': 42}  # dict overrides YAML values
    ]
)
```

### From CLI
```bash
ros2 run my_pkg my_node --ros-args -p max_speed:=2.0 -p robot_name:=atlas
ros2 run my_pkg my_node --ros-args --params-file config/params.yaml
```

## Parameter Types
| Python Type | ROS Parameter Type |
|-------------|-------------------|
| `bool` | PARAMETER_BOOL |
| `int` | PARAMETER_INTEGER |
| `float` | PARAMETER_DOUBLE |
| `str` | PARAMETER_STRING |
| `list[bool]` | PARAMETER_BOOL_ARRAY |
| `list[int]` | PARAMETER_INTEGER_ARRAY |
| `list[float]` | PARAMETER_DOUBLE_ARRAY |
| `list[str]` | PARAMETER_STRING_ARRAY |
| `bytes` | PARAMETER_BYTE_ARRAY |

## ParameterEventHandler (Cross-Node Monitoring)
```python
from rclpy.parameter_event_handler import ParameterEventHandler

class Monitor(Node):
    def __init__(self):
        super().__init__('monitor')
        self._handler = ParameterEventHandler(self)
        # Watch a parameter on another node
        self._cb_handle = self._handler.add_parameter_callback(
            'gain', '/other_node', self._on_gain_changed
        )

    def _on_gain_changed(self, param):
        self.get_logger().info(f'Other node gain changed to {param.value}')
```

## Runtime Parameter Interaction
```bash
# List all parameters
ros2 param list /my_node

# Get a parameter
ros2 param get /my_node max_speed

# Set a parameter
ros2 param set /my_node gain 2.5

# Dump all parameters to YAML
ros2 param dump /my_node --output-dir .
```

## Critical Warnings
- **Undeclared parameters**: By default, accessing an undeclared parameter throws `ParameterNotDeclaredException`. Always `declare_parameter` before `get_parameter`.
- **allow_undeclared_parameters**: Setting `NodeOptions().allow_undeclared_parameters(True)` bypasses declaration. Avoid this — it hides bugs and makes YAML typos silently ignored.
- **YAML node name must match**: The top-level key in the YAML file must match the node name. If the node is remapped or namespaced, the YAML key must match the fully qualified name.
- **Parameter types are fixed**: Once declared, a parameter's type cannot change. Trying to set a string parameter to an int will fail.
- **Callback return value**: You MUST return `SetParametersResult`. If your callback raises an exception, the parameter change fails silently.
- **read_only timing**: A `read_only` parameter can be set during declaration and via YAML at startup, but NOT changed at runtime via `ros2 param set`.
- **Nested YAML keys**: `waypoints.x` in YAML becomes parameter name `waypoints.x` (the dot is literal). ROS 2 does NOT have hierarchical parameters — the dot is part of the name string.
