# KinovaGen3 Beer Pong Robot

An autonomous beer pong throwing system powered by a Kinova Gen3 Lite collaborative robotic arm. This project demonstrates precision throwing mechanics using ROS2, MoveIt2, and advanced motion planning to hit target cups in a traditional beer pong formation.

## Overview

This project uses a 6-DOF Kinova Gen3 Lite robotic arm with a 2-finger gripper to autonomously pick up ping pong balls and throw them at beer pong cups with high speed and precision. The system achieves throwing distances of 3.5-4.0 meters using optimized joint trajectories and carefully timed gripper release.

## Demo

Check out the demo videos in the [docs/](docs/) folder:
- Open-loop beer pong throwing demonstrations
- Multi-cup sequential targeting
- Action photos showing the complete throw sequence

## Technology Stack

- **ROS2**: Robot Operating System 2 middleware
- **Python 3.12**: Primary programming language
- **MoveIt2**: Motion planning and collision detection (via `pymoveit2`)
- **Kinova Gen3 Lite**: 7-DOF collaborative robot arm
- **Ubuntu/Linux**: Operating system

## Project Structure

```
KinovaGen3-Beer-Pong/
Readme.md
docs/                                  # Videos and photos
ros2_ws/                               # ROS2 workspace
    src/beer_pong/                     # Main package
        beer_pong/
            beer_pong_throw.py         # Single ball throw module
            beer_pong_throwall.py      # 6-cup sequential throws
            direct_gripper_throw.py    # Alternative gripper implementation
            gripper_node.py            # Gripper control node
            joint_limits.yaml          # Robot joint constraints
            Hold_My_Beer.md            # Quick reference guide
```

## Features

### Single Throw Mode ([beer_pong_throw.py](ros2_ws/src/beer_pong/beer_pong/beer_pong_throw.py))
- Executes one optimized throw at maximum robot speed (4.5 rad/s)
- Three-phase motion: pickup � wind-up � throw
- Intelligent release timing based on elbow joint angle monitoring
- Expected throwing distance: 3.5-4.0 meters

### Multi-Cup Mode ([beer_pong_throwall.py](ros2_ws/src/beer_pong/beer_pong/beer_pong_throwall.py))
- Sequential throws at 6 cups in standard beer pong triangle formation
- Automatic targeting using base joint rotation
- Configurable pause between throws (default: 5 seconds)
- Collision object management for consistent motion planning

### Cup Formation
- Layout: Equilateral triangle (3-2-1 formation)
- Cup diameter: 9cm
- Cup height: 11.5cm
- Center-to-center spacing: 9cm
- Default distance to front cups: 2.0m (configurable)

## Installation

### Prerequisites
- ROS2 (Humble or later)
- MoveIt2
- Kinova Gen3 Lite robot with ROS2 drivers
- Python 3.12+

### Setup

1. Clone this repository:
```bash
cd ~/
git clone <repository-url> KinovaGen3-Beer-Pong
cd KinovaGen3-Beer-Pong/ros2_ws
```

2. Build the ROS2 workspace:
```bash
cd ros2_ws
colcon build
source install/setup.bash
```

3. Ensure Kinova robot drivers are running:
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
Throw at all 6 cups with 5-second pause between throws:
```bash
ros2 run beer_pong beer_pong_throwall --ros-args -p task:=throw_ball
```

Throw 3 balls only:
```bash
ros2 run beer_pong beer_pong_throwall --ros-args -p task:=throw_ball -p num_shots:=3
```

Custom pause duration (2 seconds):
```bash
ros2 run beer_pong beer_pong_throwall --ros-args -p task:=throw_ball -p pause_between:=2.0
```

Custom cup distance (2.5 meters):
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
- **Pickup velocity**: 3.0 rad/s
- **Throw velocity**: 4.5 rad/s (hardware maximum)
- **Acceleration**: 3.0-5.0 rad/s�

### Throw Mechanics
**Wind-up Position:**
- Shoulder: 35� back
- Elbow: -150� (extreme flexion)
- Wrist: -80� (cocked)

**Release Position:**
- Shoulder: -45� (deep forward)
- Elbow: 150� (extreme extension)
- Wrist: 70� (snap)

**Release Timing:**
- Triggered at -30� elbow angle for optimal ball trajectory
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

- [beer_pong_throw.py](ros2_ws/src/beer_pong/beer_pong/beer_pong_throw.py) - Single throw implementation (372 lines)
- [beer_pong_throwall.py](ros2_ws/src/beer_pong/beer_pong/beer_pong_throwall.py) - Multi-cup sequential throws (484 lines)
- [direct_gripper_throw.py](ros2_ws/src/beer_pong/beer_pong/direct_gripper_throw.py) - Alternative gripper action client (296 lines)
- [gripper_node.py](ros2_ws/src/beer_pong/beer_pong/gripper_node.py) - Standalone gripper control (102 lines)
- [joint_limits.yaml](ros2_ws/src/beer_pong/beer_pong/joint_limits.yaml) - Robot joint constraints
- [Hold_My_Beer.md](ros2_ws/src/beer_pong/beer_pong/Hold_My_Beer.md) - Quick reference guide

## Dependencies

- `rclpy`: ROS2 Python client library
- `pymoveit2`: Python bindings for MoveIt2
- `geometry_msgs`: ROS2 geometry message types
- `std_msgs`: ROS2 standard message types
- `moveit_msgs`: MoveIt2 message types
- Kinova Gen3 ROS2 drivers and gripper interface

## License

Apache License 2.0

## Development Status

Version 0.0.0 - Active development

## Troubleshooting

### Gripper Not Responding
The gripper logic is hardware-inverted:
- `gripper.close()` � physically opens
- `gripper.open()` � physically closes

If experiencing issues, try the `direct_gripper_throw` module which uses raw action clients.

### Ball Not Reaching Cups
Adjust the throw velocity or cup distance parameter. The default configuration is optimized for 2.0m distance.

### Motion Planning Failures
Ensure collision objects are properly configured and the robot has sufficient workspace clearance.

## Contributing

This is a research/educational project. Feel free to fork and experiment with different throwing strategies, trajectory optimizations, or vision-based targeting systems.

## Acknowledgments

Built with the Kinova Gen3 Lite collaborative robot and the ROS2/MoveIt2 ecosystem.
