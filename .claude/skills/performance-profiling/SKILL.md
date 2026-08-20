<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: performance-profiling
description: 'Profile ROS 2 system performance. Use when measuring topic rates, callback timing, CPU usage, or end-to-end latency.'
---

# Profiling ROS 2 System Performance

## Topic-Level Monitoring

```bash
# Publication rate (Hz)
ros2 topic hz /scan
# average rate: 10.02 Hz, min: 0.091s, max: 0.108s, std dev: 0.004s

# Bandwidth consumption
ros2 topic bw /scan
# average: 245.31KB/s, min: 24.12KB, max: 24.68KB

# End-to-end latency (requires message header stamp)
ros2 topic delay /scan
# average delay: 0.012s, min: 0.008s, max: 0.023s

# Check all active topics and their types
ros2 topic list -t

# Message count (useful for debugging "is it publishing?")
ros2 topic echo /scan --once
```

## System Health Check

```bash
# ROS 2 built-in diagnostics
ros2 doctor
ros2 doctor --report

# DDS multicast verification
ros2 multicast receive &
ros2 multicast send
# Should see "Received from ..." if multicast works

# Node introspection
ros2 node list
ros2 node info /controller_server  # shows subs, pubs, services, actions
```

## System-Level Profiling

```bash
# CPU and memory per process
htop  # interactive
ps aux | grep -E "ros2|nav2|controller|planner|costmap" | sort -k3 -rn

# Disk I/O (important during rosbag recording)
iotop -o

# GPU usage (for depth processing, ML inference)
nvidia-smi -l 1
# or for continuous logging:
nvidia-smi --query-gpu=timestamp,utilization.gpu,memory.used --format=csv -l 1

# Network traffic (DDS uses UDP multicast)
iftop -i wlan0
ss -u -a | grep -c ESTAB  # UDP socket count
```

## Callback Timing

```python
#!/usr/bin/env python3
"""Monitor callback execution times and warn if exceeding budget."""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class CallbackProfiler(Node):
    def __init__(self):
        super().__init__("callback_profiler")
        self.scan_times: list[float] = []
        self.budget_ms = 20.0  # 50 Hz budget

        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_timer(5.0, self.report)

    def scan_callback(self, msg: LaserScan):
        start = time.monotonic()
        # -- actual processing would go here --
        elapsed_ms = (time.monotonic() - start) * 1000
        self.scan_times.append(elapsed_ms)

        if elapsed_ms > self.budget_ms:
            self.get_logger().warn(
                f"Scan callback took {elapsed_ms:.1f}ms (budget: {self.budget_ms}ms)"
            )

    def report(self):
        if not self.scan_times:
            return
        import statistics
        times = self.scan_times[-100:]  # last 100 samples
        self.get_logger().info(
            f"Callback timing (last {len(times)}): "
            f"mean={statistics.mean(times):.2f}ms, "
            f"p95={sorted(times)[int(len(times)*0.95)]:.2f}ms, "
            f"max={max(times):.2f}ms"
        )
        self.scan_times.clear()
```

## Comprehensive Diagnostic Script

```python
#!/usr/bin/env python3
"""Monitor key performance metrics for a ROS 2 navigation stack."""

import subprocess
import time
import re
import psutil


def get_ros_process_stats() -> list[dict]:
    """Get CPU/memory for all ROS-related processes."""
    results = []
    for proc in psutil.process_iter(["pid", "name", "cmdline", "cpu_percent", "memory_info"]):
        try:
            cmdline = " ".join(proc.info["cmdline"] or [])
            if any(k in cmdline for k in ["ros2", "nav2", "controller_server",
                                           "planner_server", "bt_navigator",
                                           "costmap", "amcl", "slam"]):
                results.append({
                    "pid": proc.info["pid"],
                    "name": proc.info["name"],
                    "cmd": cmdline[:80],
                    "cpu": proc.info["cpu_percent"],
                    "rss_mb": proc.info["memory_info"].rss / 1024 / 1024,
                })
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return sorted(results, key=lambda x: x["cpu"], reverse=True)


def get_topic_hz(topic: str, duration: float = 3.0) -> float | None:
    """Measure topic publication rate."""
    try:
        result = subprocess.run(
            ["ros2", "topic", "hz", topic, "--window", "20"],
            capture_output=True, text=True, timeout=duration,
        )
        match = re.search(r"average rate:\s+([\d.]+)", result.stdout)
        return float(match.group(1)) if match else None
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None


def monitor_loop():
    """Print a periodic performance summary."""
    critical_topics = {
        "/scan": 10.0,        # expected Hz
        "/odom": 50.0,
        "/cmd_vel": 20.0,
        "/battery_state": 1.0,
    }

    print("=" * 70)
    print("ROS 2 Performance Monitor")
    print("=" * 70)

    # System overview
    cpu_total = psutil.cpu_percent(interval=1)
    mem = psutil.virtual_memory()
    print(f"\nSystem: CPU {cpu_total:.0f}% | RAM {mem.percent:.0f}% "
          f"({mem.used/1024**3:.1f}/{mem.total/1024**3:.1f} GB)")

    # Per-process stats
    print(f"\n{'PID':>7} {'CPU%':>6} {'RSS MB':>8}  Command")
    print("-" * 70)
    for p in get_ros_process_stats()[:15]:
        print(f"{p['pid']:>7} {p['cpu']:>6.1f} {p['rss_mb']:>8.1f}  {p['cmd']}")

    # Topic rates
    print(f"\n{'Topic':<30} {'Rate':>8} {'Expected':>10} {'Status':>8}")
    print("-" * 60)
    for topic, expected_hz in critical_topics.items():
        hz = get_topic_hz(topic)
        if hz is None:
            status = "NO DATA"
        elif hz < expected_hz * 0.8:
            status = "SLOW"
        elif hz > expected_hz * 1.2:
            status = "FAST"
        else:
            status = "OK"
        hz_str = f"{hz:.1f} Hz" if hz else "N/A"
        print(f"{topic:<30} {hz_str:>8} {expected_hz:>8.0f} Hz {status:>8}")


if __name__ == "__main__":
    monitor_loop()
```

## Nav2-Specific Performance

| Component | Typical CPU Concern | Tuning |
|-----------|-------------------|--------|
| `controller_server` | Loop rate 20Hz must be achievable | Lower rate or simpler controller |
| `global_costmap` | High with large maps | Reduce resolution (0.1m → 0.15m) |
| `local_costmap` | Updates each controller tick | Shrink rolling window size |
| `planner_server` | Spikes on replan | Increase replan interval |
| `amcl` | Particle filter with many particles | Reduce `max_particles` |
| MPPI controller | GPU helps; CPU-heavy otherwise | Reduce `batch_size`, `time_steps` |

## DDS Tuning

If discovery is slow or messages drop, configure FastDDS:

```xml
<!-- fastdds_profile.xml -->
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="default_participant" is_default_profile="true">
    <rtps>
      <builtin>
        <discovery_config>
          <leaseDuration>
            <sec>10</sec>
          </leaseDuration>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_profile.xml
```

## Memory Leak Detection

Track resident set size (RSS) over time. A steady climb indicates a leak:

```bash
# Log RSS every 30s for a node
while true; do
    ps -p $(pgrep -f controller_server) -o pid,rss,vsz --no-headers 2>/dev/null
    sleep 30
done | ts '[%Y-%m-%d %H:%M:%S]' >> rss_log.txt
```

## End-to-End Latency

Measure sensor-to-actuator delay: stamp a message at sensor callback entry, measure time at motor command publication. For Nav2, the chain is: scan → costmap update → controller → cmd_vel. Typical healthy latency is < 100ms for the full loop at 10Hz scan rate.
