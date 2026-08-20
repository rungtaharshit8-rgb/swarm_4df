# Swarm_4DF — Swarm Robotics Platform

A multi-robot (swarm) platform built on ROS, where each unit is equipped with LiDAR, a camera, and an IMU for autonomous navigation and mapping.

Repository: https://github.com/rungtaharshit8-rgb/swarm_4df

## Project Status

### ✅ Working
- **LiDAR** — sensor is integrated and streaming data on each robot.
- **Camera** — sensor is integrated and streaming data on each robot.
- **IMU** — sensor is integrated and streaming data on each robot.
- **teleop_twist_keyboard** — each car/robot in the swarm can be driven individually using teleop_twist_keyboard.

### 🚧 In Progress
- **LiDAR integration** across the full swarm (beyond per-robot sensing) is being prepared.
- **Map merging** — combining individual robots' maps into a single shared/global map is being prepared.

## Hardware Overview

Each robot ("car") in the swarm is equipped with:
| Sensor | Purpose | Status |
|---|---|---|
| LiDAR | Obstacle detection / mapping | Working |
| Camera | Visual perception | Working |
| IMU | Orientation / motion tracking | Working |

## Usage

### Teleoperation (per robot)
Each car can currently be controlled independently via teleop_twist_keyboard:

```bash
# example — adjust to match your actual launch/package names
roslaunch swarm_4df teleop_keyboard.launch robot:=<robot_name>
```

Use the standard teleop keys (commonly `i`/`,`/`j`/`l`/`k`, or arrow keys depending on the teleop node) to move the selected robot forward, backward, left, and right.

> Update the exact launch command/package name above to match what's in this repo.

## Roadmap

- [x] Bring up LiDAR, camera, and IMU on each robot
- [x] Keyboard teleop control per robot
- [ ] Swarm-wide LiDAR integration
- [ ] Map merging across robots into a unified map
- [ ] Autonomous swarm navigation

## Contributing

Contributions are welcome. Please open an issue or pull request for bugs, feature requests, or improvements.
