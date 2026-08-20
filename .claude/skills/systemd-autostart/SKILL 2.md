<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: systemd-autostart
description: 'Auto-start ROS 2 with systemd. Use when creating service files for robot boot, managing dependencies, or viewing logs.'
---

# Auto-Starting ROS 2 Nodes on Boot with systemd

## Basic Service File

Create `/etc/systemd/system/my-robot.service`:

```ini
[Unit]
Description=Robot ROS 2 Bringup
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=robot
Group=robot

# Source ROS 2 and workspace, then launch
ExecStart=/bin/bash -c '\
  source /opt/ros/jazzy/setup.bash && \
  source /home/robot/ros2_ws/install/setup.bash && \
  ros2 launch my_robot_bringup sigyn.launch.py'

# Environment variables
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
Environment="ROS_LOG_DIR=/home/robot/.ros/log"

# Restart policy
Restart=on-failure
RestartSec=5
StartLimitIntervalSec=60
StartLimitBurst=5

# Resource limits
LimitNOFILE=65536

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=my-robot

[Install]
WantedBy=multi-user.target
```

## Managing the Service

```bash
# Reload after editing any .service file
sudo systemctl daemon-reload

# Enable to start on boot
sudo systemctl enable my-robot.service

# Manual start/stop/restart
sudo systemctl start my-robot.service
sudo systemctl stop my-robot.service
sudo systemctl restart my-robot.service

# Check status
sudo systemctl status my-robot.service

# View logs (follow mode)
journalctl -u my-robot.service -f

# View logs since last boot
journalctl -u my-robot.service -b

# View last 100 lines
journalctl -u my-robot.service -n 100
```

## Multi-Service Architecture

Split the robot into ordered services for reliability. If hardware drivers crash, only they restart—navigation stays up if topics are still published.

### Hardware Drivers (starts first)

`/etc/systemd/system/sigyn-hardware.service`:

```ini
[Unit]
Description=the robot Hardware Drivers (Teensy, LIDAR, IMU)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=robot
ExecStart=/bin/bash -c '\
  source /opt/ros/jazzy/setup.bash && \
  source /home/robot/ros2_ws/install/setup.bash && \
  ros2 launch my_robot_bringup sub_launch/hardware.launch.py'

Environment="ROS_DOMAIN_ID=0"
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

### Navigation (depends on hardware)

`/etc/systemd/system/sigyn-navigation.service`:

```ini
[Unit]
Description=the robot Navigation Stack
After=sigyn-hardware.service
Requires=sigyn-hardware.service

[Service]
Type=simple
User=robot
ExecStart=/bin/bash -c '\
  source /opt/ros/jazzy/setup.bash && \
  source /home/robot/ros2_ws/install/setup.bash && \
  ros2 launch my_robot_bringup sub_launch/navigation.launch.py'

Environment="ROS_DOMAIN_ID=0"

# Give navigation stack time to initialize before considering it failed
TimeoutStartSec=30
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

### Perception (independent of navigation)

`/etc/systemd/system/sigyn-perception.service`:

```ini
[Unit]
Description=the robot Perception (OAK-D, Object Detection)
After=sigyn-hardware.service
Wants=sigyn-hardware.service

[Service]
Type=simple
User=robot
ExecStart=/bin/bash -c '\
  source /opt/ros/jazzy/setup.bash && \
  source /home/robot/ros2_ws/install/setup.bash && \
  ros2 launch oakd_detector detector.launch.py'

Environment="ROS_DOMAIN_ID=0"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

## Using an Environment File

Instead of inline `Environment=` lines, centralize variables:

`/etc/sigyn/robot.env`:

```bash
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_LOG_DIR=/home/robot/.ros/log
RCUTILS_COLORIZED_OUTPUT=0
```

Reference it in the service:

```ini
[Service]
EnvironmentFile=/etc/sigyn/robot.env
```

## Graceful Shutdown

ROS 2 lifecycle nodes need clean shutdown signals. systemd sends SIGTERM by default, which `rclpy` and `rclcpp` handle correctly. For extra safety:

```ini
[Service]
# Send SIGTERM, wait 15 seconds, then SIGKILL
TimeoutStopSec=15
KillMode=mixed
KillSignal=SIGTERM

# Optional: run a shutdown script
ExecStop=/bin/bash -c '\
  source /opt/ros/jazzy/setup.bash && \
  source /home/robot/ros2_ws/install/setup.bash && \
  ros2 lifecycle set /nav2_controller shutdown'
```

## Pre-Start Checks

Run a hardware check before launching:

```ini
[Service]
ExecStartPre=/bin/bash -c '\
  source /opt/ros/jazzy/setup.bash && \
  source /home/robot/ros2_ws/install/setup.bash && \
  ros2 launch my_robot_bringup precheck.launch.py'
ExecStart=/bin/bash -c '\
  source /opt/ros/jazzy/setup.bash && \
  source /home/robot/ros2_ws/install/setup.bash && \
  ros2 launch my_robot_bringup sigyn.launch.py'
```

## Common Issues and Fixes

### Missing hardware permissions

```bash
# Add the service user to required groups
sudo usermod -aG dialout sigyn   # Serial ports (Teensy, LIDAR)
sudo usermod -aG video sigyn     # Cameras
sudo usermod -aG i2c sigyn       # I2C sensors
sudo usermod -aG gpio sigyn      # GPIO pins (Raspberry Pi)
```

### Service starts before devices are ready

```ini
[Unit]
# Wait for specific udev device
After=dev-ttyACM0.device
Requires=dev-ttyACM0.device
```

Or use a startup delay:

```ini
[Service]
ExecStartPre=/bin/sleep 5
```

### No display server for GUI nodes

If a node needs DISPLAY (e.g., debugging with RViz):

```ini
[Service]
Environment="DISPLAY=:0"
Environment="XAUTHORITY=/home/robot/.Xauthority"
```

This is generally not needed for headless robot operation.

### Logging fills disk

```bash
# Limit journal size
sudo journalctl --vacuum-size=500M

# Or in /etc/systemd/journald.conf:
# SystemMaxUse=500M
```

## Debugging a Failing Service

```bash
# See why it failed
systemctl status my-robot.service
journalctl -u my-robot.service --no-pager -n 50

# Test the ExecStart command manually as the service user
sudo -u sigyn bash -c '\
  source /opt/ros/jazzy/setup.bash && \
  source /home/robot/ros2_ws/install/setup.bash && \
  ros2 launch my_robot_bringup sigyn.launch.py'

# Check environment resolution
systemctl show my-robot.service --property=Environment
```

## Enable/Start All Robot Services at Once

```bash
sudo systemctl enable sigyn-hardware sigyn-navigation sigyn-perception
sudo systemctl start sigyn-hardware sigyn-navigation sigyn-perception

# Or create a target that groups them:
# /etc/systemd/system/my-robot.target
# [Unit]
# Description=the robot Robot Full Stack
# Requires=sigyn-hardware.service sigyn-navigation.service sigyn-perception.service
# After=sigyn-hardware.service
# [Install]
# WantedBy=multi-user.target
```
