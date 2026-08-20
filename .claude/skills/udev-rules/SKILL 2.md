<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: udev-rules
description: 'Write udev rules for persistent USB naming. Use when creating /dev/ symlinks for lidar, motor controller, or Teensy serial ports.'
---

# Writing udev Rules for Persistent USB Device Naming

## The Problem

Linux assigns serial device names (`/dev/ttyACM0`, `/dev/ttyUSB0`) dynamically based on enumeration order. If you have a Teensy, a LIDAR, and a motor controller, their assignments shuffle on every reboot or reconnection. Your launch file hardcodes `/dev/ttyACM0` for the Teensy — but today that's the LIDAR.

## Solution: udev Symlinks

udev rules create persistent symlinks based on device attributes. Instead of `/dev/ttyACM0`, you get `/dev/teensy_main`, `/dev/lidar`, `/dev/roboclaw`.

## Finding Device Attributes

Plug in one device at a time. Find its current device node:

```bash
# List recent USB serial devices
ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
dmesg | tail -20   # see what just connected
```

Then query its full attribute chain:

```bash
udevadm info -a -n /dev/ttyACM0
```

Key attributes to look for:

```
ATTRS{idVendor}=="16c0"         # Teensy vendor ID
ATTRS{idProduct}=="0483"        # Teensy product ID
ATTRS{serial}=="12345678"       # unique serial number (if available)
ATTRS{manufacturer}=="Teensyduino"
```

For FTDI-based devices (many USB-serial adapters):
```
ATTRS{idVendor}=="0403"
ATTRS{idProduct}=="6001"
ATTRS{serial}=="AB0CDEFG"       # FTDI serial is usually unique
```

For devices without unique serials (e.g., cheap CH340 adapters), use the physical USB port:
```bash
udevadm info -a -n /dev/ttyUSB0 | grep KERNELS
# KERNELS=="1-2.3:1.0"         # physical USB port path
```

## Writing Rules

### Rule File Location

```bash
sudo nano /etc/udev/rules.d/99-robot-devices.rules
```

Rules are processed in alphabetical order. Using `99-` ensures your rules run after all system defaults.

### Rule Syntax

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", ATTRS{serial}=="ZZZZ", SYMLINK+="device_name", MODE="0666"
```

- `SUBSYSTEM=="tty"`: Match only serial/tty devices
- `ATTRS{...}`: Match against device attributes (walk up the device tree)
- `SYMLINK+="name"`: Create `/dev/name` symlink (the `+=` appends, doesn't replace)
- `MODE="0666"`: Read/write for all users (avoids permission issues)
- `GROUP="dialout"`: Alternative to MODE — add your user to the `dialout` group

### Example Rules for a Typical Robot

```udev
# Teensy 4.1 - Main board (identified by serial number)
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="14026180", SYMLINK+="teensy_main", MODE="0666"

# Teensy 4.1 - Sensor board (different serial number)
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ATTRS{serial}=="14026181", SYMLINK+="teensy_sensors", MODE="0666"

# LDROBOT LD19 LIDAR (CP2102 USB-serial chip)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0666"

# RoboClaw motor controller (FTDI chip)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", ATTRS{serial}=="RC_001", SYMLINK+="roboclaw", MODE="0666"

# RPLidar A2 (CP2102)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="RP0001", SYMLINK+="rplidar", MODE="0666"

# OAK-D camera (USB device, non-tty)
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
```

## Handling Multiple Identical Devices

When two devices have the same vendor/product ID and no unique serial (e.g., two CH340 adapters), use the physical USB port path:

```bash
# Find the port path for each device
udevadm info -a -n /dev/ttyUSB0 | grep KERNELS
# look for: KERNELS=="1-1.2:1.0"

udevadm info -a -n /dev/ttyUSB1 | grep KERNELS
# look for: KERNELS=="1-1.3:1.0"
```

```udev
# CH340 on USB port 1-1.2 → left motor controller
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", KERNELS=="1-1.2", SYMLINK+="motor_left", MODE="0666"

# CH340 on USB port 1-1.3 → right motor controller
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", KERNELS=="1-1.3", SYMLINK+="motor_right", MODE="0666"
```

**Warning**: Port-based rules break if you move devices to different USB ports. Label the ports physically and document the assignments.

## Applying Rules

```bash
# Reload rules
sudo udevadm control --reload-rules

# Trigger rules for existing devices (without unplug/replug)
sudo udevadm trigger

# Verify symlinks
ls -la /dev/teensy_main /dev/lidar /dev/roboclaw
# lrwxrwxrwx 1 root root 7 ... /dev/teensy_main -> ttyACM0
# lrwxrwxrwx 1 root root 7 ... /dev/lidar -> ttyUSB0
# lrwxrwxrwx 1 root root 7 ... /dev/roboclaw -> ttyUSB1
```

## Using Symlinks in ROS 2 Launch Files

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            parameters=[{'serial_port': '/dev/lidar'}],
        ),
        Node(
            package='motor_driver',
            executable='motor_driver_node',
            parameters=[{'port': '/dev/roboclaw', 'baud': 115200}],
        ),
    ])
```

## Debugging udev Rules

```bash
# Test a rule against a device without applying
udevadm test $(udevadm info -q path -n /dev/ttyACM0)

# Monitor udev events in real-time (plug/unplug devices to see events)
udevadm monitor --property

# Check if a specific attribute is available
udevadm info -a -n /dev/ttyACM0 | grep -i serial

# Verify rule syntax (look for parse errors in syslog)
journalctl -u systemd-udevd -f
```

## Common Pitfalls

- **ATTRS vs ATTR**: `ATTRS` walks up the device tree (matches parent attributes). `ATTR` matches only the current device node. For serial devices, you almost always need `ATTRS` because vendor/product IDs are on the parent USB device, not the tty child.
- **Multiple ATTRS from different parents**: A single rule cannot match ATTRS from two different parent levels. If you need both `ATTRS{serial}` (from USB device) and `KERNELS` (from USB port), you may need two rules or use `ENV{}` variables.
- **Docker containers**: udev rules run on the host. Inside Docker, you see the symlinks only if you bind-mount `/dev` or the specific device path. Use `--device=/dev/teensy_main:/dev/teensy_main` in `docker run`.
- **Permissions after rule change**: If you change MODE or GROUP, unplug and replug the device (or run `udevadm trigger`). Existing device nodes keep their old permissions.
