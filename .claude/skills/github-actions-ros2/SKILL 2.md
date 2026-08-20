<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: github-actions-ros2
description: 'Set up GitHub Actions CI for ROS 2. Use when configuring ros-tooling/action-ros-ci, matrix builds, linting, or hardware-in-loop testing.'
---

# CI/CD with GitHub Actions for ROS 2 Packages

## Basic CI Workflow

Create `.github/workflows/ros2_ci.yml`:

```yaml
name: ROS 2 CI

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

jobs:
  build-and-test:
    runs-on: ubuntu-24.04
    strategy:
      fail-fast: false
      matrix:
        ros_distro: [jazzy, rolling]

    container:
      image: ros:${{ matrix.ros_distro }}

    steps:
      - name: Install build tools
        run: |
          apt-get update && apt-get install -y \
            python3-colcon-common-extensions \
            python3-rosdep \
            python3-vcstool \
            git

      - name: Checkout repository
        uses: actions/checkout@v4
        with:
          path: src/my_robot

      - name: Initialize rosdep
        run: |
          rosdep init || true
          rosdep update

      - name: Install dependencies
        run: |
          source /opt/ros/${{ matrix.ros_distro }}/setup.bash
          rosdep install --from-paths src --ignore-src -r -y

      - name: Build
        run: |
          source /opt/ros/${{ matrix.ros_distro }}/setup.bash
          colcon build \
            --symlink-install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            --event-handlers console_direct+

      - name: Test
        run: |
          source /opt/ros/${{ matrix.ros_distro }}/setup.bash
          source install/setup.bash
          colcon test \
            --event-handlers console_direct+ \
            --return-code-on-test-failure

      - name: Upload test results
        if: always()
        uses: actions/upload-artifact@v4
        with:
          name: test-results-${{ matrix.ros_distro }}
          path: |
            build/*/test_results/
            log/
```

## Using ros-tooling/action-ros-ci

The `action-ros-ci` action wraps colcon build+test with built-in rosdep, caching, and multi-repo support.

```yaml
name: ROS 2 CI (ros-tooling)

on:
  push:
    branches: [main]
  pull_request:

jobs:
  test:
    runs-on: ubuntu-24.04
    strategy:
      matrix:
        ros_distro: [jazzy, rolling]
    steps:
      - uses: actions/checkout@v4

      - uses: ros-tooling/setup-ros@v0.7
        with:
          required-ros-distributions: ${{ matrix.ros_distro }}

      - uses: ros-tooling/action-ros-ci@v0.3
        with:
          target-ros2-distro: ${{ matrix.ros_distro }}
          package-name: my_robot_bringup my_robot_nav_goals my_robot_interfaces
          colcon-defaults: |
            {
              "build": {
                "cmake-args": ["-DCMAKE_BUILD_TYPE=Release"]
              }
            }
          # Import additional repos via .repos file
          vcs-repo-file-url: dependencies.repos
```

## Linting Workflow

```yaml
name: ROS 2 Linting

on: [push, pull_request]

jobs:
  lint:
    runs-on: ubuntu-24.04
    container:
      image: ros:jazzy

    steps:
      - uses: actions/checkout@v4

      - name: Install linters
        run: |
          apt-get update && apt-get install -y \
            ros-jazzy-ament-lint-auto \
            ros-jazzy-ament-lint-common \
            ros-jazzy-ament-cmake-lint-cmake \
            ros-jazzy-ament-cmake-cppcheck \
            ros-jazzy-ament-cmake-cpplint \
            ros-jazzy-ament-cmake-flake8 \
            ros-jazzy-ament-cmake-pep257 \
            ros-jazzy-ament-cmake-xmllint

      - name: Run ament_lint_auto
        run: |
          source /opt/ros/jazzy/setup.bash
          colcon build --symlink-install
          colcon test --packages-select my_robot_bringup \
            --event-handlers console_direct+ \
            --return-code-on-test-failure
          colcon test-result --verbose
```

## Code Coverage

```yaml
  coverage:
    runs-on: ubuntu-24.04
    container:
      image: ros:jazzy

    steps:
      - uses: actions/checkout@v4
        with:
          path: src/my_robot

      - name: Install deps
        run: |
          apt-get update && apt-get install -y \
            python3-colcon-common-extensions \
            python3-rosdep lcov python3-pytest-cov
          rosdep init || true && rosdep update
          source /opt/ros/jazzy/setup.bash
          rosdep install --from-paths src --ignore-src -r -y

      # C++ coverage with lcov
      - name: Build with coverage flags
        run: |
          source /opt/ros/jazzy/setup.bash
          colcon build --symlink-install \
            --cmake-args \
              -DCMAKE_BUILD_TYPE=Debug \
              -DCMAKE_CXX_FLAGS="--coverage" \
              -DCMAKE_C_FLAGS="--coverage"

      - name: Run tests
        run: |
          source /opt/ros/jazzy/setup.bash
          source install/setup.bash
          colcon test --event-handlers console_direct+

      - name: Generate C++ coverage report
        run: |
          lcov --capture --directory build/ --output-file coverage.info
          lcov --remove coverage.info '/opt/*' '/usr/*' '*/test/*' \
            --output-file coverage_filtered.info
          lcov --list coverage_filtered.info

      - name: Upload coverage
        uses: codecov/codecov-action@v4
        with:
          files: coverage_filtered.info
          token: ${{ secrets.CODECOV_TOKEN }}
```

## Caching Dependencies

Speed up builds by caching apt packages and rosdep installations:

```yaml
      - name: Cache apt packages
        uses: actions/cache@v4
        with:
          path: /var/cache/apt/archives
          key: apt-${{ matrix.ros_distro }}-${{ hashFiles('**/package.xml') }}
          restore-keys: |
            apt-${{ matrix.ros_distro }}-

      - name: Cache rosdep
        uses: actions/cache@v4
        with:
          path: ~/.ros/rosdep
          key: rosdep-${{ matrix.ros_distro }}-${{ hashFiles('**/package.xml') }}
```

## Integration Tests with Gazebo (Headless)

```yaml
  integration:
    runs-on: ubuntu-24.04
    container:
      image: ros:jazzy-desktop

    steps:
      - uses: actions/checkout@v4
        with:
          path: src/my_robot

      - name: Install Gazebo and test deps
        run: |
          apt-get update && apt-get install -y \
            ros-jazzy-ros-gz \
            python3-colcon-common-extensions python3-rosdep
          rosdep init || true && rosdep update
          source /opt/ros/jazzy/setup.bash
          rosdep install --from-paths src --ignore-src -r -y

      - name: Build
        run: |
          source /opt/ros/jazzy/setup.bash
          colcon build --symlink-install

      - name: Run integration tests (headless)
        env:
          DISPLAY: ""
          GZ_SIM_HEADLESS_RENDERING: 1
        run: |
          source /opt/ros/jazzy/setup.bash
          source install/setup.bash
          colcon test --packages-select my_robot_bringup \
            --event-handlers console_direct+ \
            --return-code-on-test-failure
```

## Pre-Commit Hooks

`.pre-commit-config.yaml`:

```yaml
repos:
  - repo: https://github.com/pre-commit/pre-commit-hooks
    rev: v4.6.0
    hooks:
      - id: trailing-whitespace
      - id: end-of-file-fixer
      - id: check-xml           # catches malformed package.xml
      - id: check-yaml

  - repo: https://github.com/psf/black
    rev: 24.4.2
    hooks:
      - id: black

  - repo: https://github.com/PyCQA/flake8
    rev: 7.1.0
    hooks:
      - id: flake8
        args: [--max-line-length=99]

  - repo: https://github.com/cpplint/cpplint
    rev: 1.6.1
    hooks:
      - id: cpplint
        args: [--filter=-whitespace/braces,-build/include_order]
```

```bash
pip install pre-commit
pre-commit install
# Runs automatically on git commit
```

## Self-Hosted Runner for Hardware-in-the-Loop

For testing on the actual robot hardware:

```bash
# On the robot:
mkdir ~/actions-runner && cd ~/actions-runner
curl -o actions-runner-linux-arm64-2.319.0.tar.gz -L \
  https://github.com/actions/runner/releases/download/v2.319.0/actions-runner-linux-arm64-2.319.0.tar.gz
tar xzf ./actions-runner-linux-arm64-2.319.0.tar.gz
./config.sh --url https://github.com/your_org/my_robot --token <TOKEN>
sudo ./svc.sh install && sudo ./svc.sh start
```

```yaml
  hardware-test:
    runs-on: self-hosted
    if: github.event_name == 'push' && github.ref == 'refs/heads/main'
    needs: build-and-test
    steps:
      - uses: actions/checkout@v4
      - name: Deploy and test on robot
        run: |
          cd ~/ros2_ws/src/my_robot && git pull
          source /opt/ros/jazzy/setup.bash
          cd ~/ros2_ws
          colcon build --symlink-install --packages-select my_robot_bringup
          source install/setup.bash
          ros2 launch my_robot_bringup precheck.launch.py
```
