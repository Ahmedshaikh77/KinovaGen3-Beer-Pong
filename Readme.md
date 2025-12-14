# KinovaGen3 Beer Pong Robot

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.12-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-orange.svg)](LICENSE)

An autonomous beer pong throwing system powered by a Kinova Gen3 Lite collaborative robotic arm. This project demonstrates precision throwing mechanics using ROS2, MoveIt2, and advanced motion planning to hit target cups in a traditional beer pong formation.

https://github.com/user-attachments/assets/74468b8c-d71d-403c-bb11-c0e4a3d168cd

## Overview

This project uses a 6-DOF Kinova Gen3 Lite robotic arm with a 2-finger gripper to autonomously pick up ping pong balls and throw them at beer pong cups with high speed and precision. The system achieves throwing distances of **3.5-4.0 meters** using optimized joint trajectories and carefully timed gripper release.

### Key Highlights
- Autonomous ball pickup and throwing
- High-speed precision trajectories (4.5 rad/s)
- Multi-cup sequential targeting
- Intelligent release timing based on joint angle monitoring
- Open-loop control with consistent accuracy

## Demo

<p align="center">
  <img src="docs/ Robot Picking Up the Ball.jpeg" alt="Robot Picking Up Ball" width="350"/>
  <img src="docs/Ball Landing in the Cup After the Throw.jpeg" alt="Ball Landing in Cup" width="350"/>
</p>

**Video Demonstrations:**
- [Kinova Gen3 Lite Beer Pong Demo](docs/Kinova%20Gen3%20Lite%20Beer%20Pong%20Demo.mp4)
- [Open-Loop Beer Pong with Kinova Gen3](docs/Open-Loop%20Beer%20Pong%20with%20a%20Kinova%20Gen3.mp4)

## Technology Stack

| Component | Technology |
|-----------|-----------|
| **Middleware** | ROS2 (Robot Operating System 2) |
| **Language** | Python 3.12 |
| **Motion Planning** | MoveIt2 (via `pymoveit2`) |
| **Robot Hardware** | Kinova Gen3 Lite (6-DOF collaborative arm) |
| **Operating System** | Ubuntu/Linux |

## Project Structure

```
KinovaGen3-Beer-Pong/
├── Readme.md
├── docs/                                  # Videos and photos
│   ├── Kinova Gen3 Lite Beer Pong Demo.mp4
│   ├── Open-Loop Beer Pong with a Kinova Gen3.mp4
│   └── *.jpeg                             # Action shots
└── ros2_ws/                               # ROS2 workspace
    └── src/beer_pong/                     # Main package
        └── beer_pong/
            ├── beer_pong_throw.py         # Single ball throw module
            ├── beer_pong_throwall.py      # 6-cup sequential throws
            ├── direct_gripper_throw.py    # Alternative gripper implementation
            ├── gripper_node.py            # Gripper control node
            ├── joint_limits.yaml          # Robot joint constraints
            └── Hold_My_Beer.md            # Quick reference guide
```

## Features

### Single Throw Mode
**Module:** [beer_pong_throw.py](ros2_ws/src/beer_pong/beer_pong/beer_pong_throw.py)

- Executes one optimized throw at maximum robot speed (4.5 rad/s)
- Three-phase motion: pickup → wind-up → throw
- Intelligent release timing based on elbow joint angle monitoring
- Expected throwing distance: **3.5-4.0 meters**

### Multi-Cup Mode
**Module:** [beer_pong_throwall.py](ros2_ws/src/beer_pong/beer_pong/beer_pong_throwall.py)

- Sequential throws at 6 cups in standard beer pong triangle formation
- Automatic targeting using base joint rotation
- Configurable pause between throws (default: 5 seconds)
- Collision object management for consistent motion planning

### Cup Formation
| Parameter | Value |
|-----------|-------|
| Layout | Equilateral triangle (3-2-1 formation) |
| Cup diameter | 9cm |
| Cup height | 11.5cm |
| Center-to-center spacing | 9cm |
| Default distance to front cups | 2.0m (configurable) |

## Installation

### Prerequisites
- ROS2 (Jazzy)
- MoveIt2
- Kinova Gen3 Lite robot with ROS2 drivers
- Python 3.12+

### Setup

**1. Clone this repository:**
```bash
cd ~/
git clone <repository-url> KinovaGen3-Beer-Pong
cd KinovaGen3-Beer-Pong/ros2_ws
```

**2. Build the ROS2 workspace:**
```bash
cd ros2_ws
colcon build
source install/setup.bash
```

**3. Ensure Kinova robot drivers are running:**
```bash
# Follow Kinova Gen3 ROS2 setup instructions
```

## Usage

### Single Throw
Execute one optimized throw:
```bash
ros2 run beer_pong beer_pong_throw --ros-args -p task:=throw_ball
```

### Multi-Cup Throws

**Throw at all 6 cups** (5-second pause between throws):
```bash
ros2 run beer_pong beer_pong_throwall --ros-args -p task:=throw_ball
```

**Throw 3 balls only:**
```bash
ros2 run beer_pong beer_pong_throwall --ros-args -p task:=throw_ball -p num_shots:=3
```

**Custom pause duration** (2 seconds):
```bash
ros2 run beer_pong beer_pong_throwall --ros-args -p task:=throw_ball -p pause_between:=2.0
```

**Custom cup distance** (2.5 meters):
```bash
ros2 run beer_pong beer_pong_throwall --ros-args -p task:=throw_ball -p cup_distance:=2.5
```

### Standalone Gripper Control
Run the gripper node separately:
```bash
ros2 run beer_pong gripper_node
```

## Technical Details

### Motion Parameters
| Parameter | Value |
|-----------|-------|
| **Pickup velocity** | 3.0 rad/s |
| **Throw velocity** | 4.5 rad/s (hardware maximum) |
| **Acceleration** | 3.0-5.0 rad/s² |

### Throw Mechanics

**Wind-up Position:**
| Joint | Angle |
|-------|-------|
| Shoulder | 35° back |
| Elbow | -150° (extreme flexion) |
| Wrist | -80° (cocked) |

**Release Position:**
| Joint | Angle |
|-------|-------|
| Shoulder | -45° (deep forward) |
| Elbow | 150° (extreme extension) |
| Wrist | 70° (snap) |

**Release Timing:**
- Triggered at **-30° elbow angle** for optimal ball trajectory
- Multi-threaded joint monitoring for precise timing

### Execution Flow
1. Initialize robot and MoveIt2 interface
2. Add collision objects (ball and cups)
3. Open gripper and move to pickup position
4. Close gripper to grab ball
5. Move to wind-up position
6. Execute throw at maximum speed
7. Monitor joint angles and release ball at optimal timing
8. Return to home position
9. Repeat for multi-cup mode

## Key Files

| File | Description | Lines |
|------|-------------|-------|
| [beer_pong_throw.py](ros2_ws/src/beer_pong/beer_pong/beer_pong_throw.py) | Single throw implementation | 372 |
| [beer_pong_throwall.py](ros2_ws/src/beer_pong/beer_pong/beer_pong_throwall.py) | Multi-cup sequential throws | 484 |
| [direct_gripper_throw.py](ros2_ws/src/beer_pong/beer_pong/direct_gripper_throw.py) | Alternative gripper action client | 296 |
| [gripper_node.py](ros2_ws/src/beer_pong/beer_pong/gripper_node.py) | Standalone gripper control | 102 |
| [joint_limits.yaml](ros2_ws/src/beer_pong/beer_pong/joint_limits.yaml) | Robot joint constraints | 67 |
| [Hold_My_Beer.md](ros2_ws/src/beer_pong/beer_pong/Hold_My_Beer.md) | Quick reference guide | - |

### Joint Limits Configuration
**File:** [joint_limits.yaml](ros2_ws/src/beer_pong/beer_pong/joint_limits.yaml)

This YAML file configures the robot's motion dynamics and constraints:

**Velocity and Acceleration Scaling:**
- `default_velocity_scaling_factor: 1.0` - Maximum speed (100%)
- `default_acceleration_scaling_factor: 1.0` - Maximum acceleration (100%)

**Joint Limits (all 6 joints + gripper):**
- Max velocity: 3.5 rad/s (joints 1-6), 5.0 rad/s (gripper fingers)
- Max acceleration: 5.0 rad/s² (all joints)
- Joint 3 position limits: -2.65 to 2.66 radians
- Gripper joints have effort limits enabled

This configuration ensures safe operation while allowing maximum performance for the throwing motion.

## Dependencies

- `rclpy` - ROS2 Python client library
- `pymoveit2` - Python bindings for MoveIt2
- `geometry_msgs` - ROS2 geometry message types
- `std_msgs` - ROS2 standard message types
- `moveit_msgs` - MoveIt2 message types
- Kinova Gen3 ROS2 drivers and gripper interface

## Troubleshooting

### Gripper Not Responding
The gripper logic is hardware-inverted:
- `gripper.close()` → physically opens
- `gripper.open()` → physically closes

**Solution:** Try the `direct_gripper_throw` module which uses raw action clients.

### Ball Not Reaching Cups
**Problem:** Ball doesn't reach the cups at default distance.

**Solution:** Adjust the throw velocity or cup distance parameter. The default configuration is optimized for 2.0m distance.

### Motion Planning Failures
**Problem:** Robot fails to plan motion.

**Solution:** Ensure collision objects are properly configured and the robot has sufficient workspace clearance.

---

## Contributing

This is a research/educational project. Feel free to fork and experiment with different throwing strategies, trajectory optimizations, or vision-based targeting systems.

## License

Apache License 2.0

## Version

**v0.0.0** - Active development

## Acknowledgments

Built with the Kinova Gen3 Lite collaborative robot and the ROS2/MoveIt2 ecosystem.
