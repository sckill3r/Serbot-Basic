# Serbot Autonomous ROS Robot

This repository contains the code and configuration for Serbot, an autonomous mobile robot platform based on ROS2. Serbot integrates Arduino, Raspberry Pi, LIDAR (customizable), and an Ubuntu VM for advanced navigation, mapping, and visualization using RViz and SLAM.

## Hardware Overview

- **Arduino**: Handles low-level motor control and encoder feedback (tested with Arduino Uno).
- **Raspberry Pi**: Runs ROS2 nodes for serial communication, sensor integration, robot control, and URDF robot description (tested with Raspberry Pi 5).
- **LIDAR**: Provides 2D laser scans for mapping and navigation. Default setup uses YDLIDAR, but you can use any ROS2-compatible LIDAR (see below).
- **Ubuntu VM**: Used for running ROS2 visualization tools (RViz), navigation stack, and SLAM algorithms.

## Directory Structure

```
Serbot_Project/
├── For_Arduino/         # Arduino code for motor drivers and encoders
├── For_Raspberrypi/    # ROS2 nodes for serial, LIDAR, URDF (serbot_description), and robot control
├── For_Vm/             # ROS2 navigation, SLAM, and visualization packages
├── README.md           # Project documentation
```

- **serbot_description** (in For_Raspberrypi): Contains the URDF robot model used for visualization and simulation in ROS2.

## System Architecture

### Data Flow

1. **Teleoperation/Navigation Commands** (from Ubuntu VM or remote):
    - Sent as `/cmd_vel` ROS2 messages.
2. **Raspberry Pi**:
    - Receives `/cmd_vel`, sends velocity commands to Arduino via serial.
    - Publishes feedback from Arduino (encoder speeds) as `/speed`.
    - Runs LIDAR drivers; publishes `/scan` topic.
    - Publishes robot description (URDF) via `serbot_description` package.
3. **Arduino**:
    - Receives velocity commands, drives motors, reads encoders.
    - Sends actual wheel speeds back to Raspberry Pi.
4. **Ubuntu VM**:
    - Runs RViz for visualization, navigation, and SLAM (using LIDAR and odometry).

### ROS2 Nodes
- **serial_control**: Bridges `/cmd_vel` (from navigation/teleop) to Arduino via serial, publishes `/speed` feedback.
- **serbot_controller**: Computes odometry from wheel speeds, publishes `/odom`.
- **ydlidar_ros2_driver**: Publishes LIDAR scans to `/scan` (see below for other LIDARs).
- **serbot_slam**: Launches SLAM Toolbox for mapping.
- **serbot_navigation**: Runs Nav2 stack for autonomous navigation.
- **serbot_description**: Publishes the robot's URDF model.

## Hardware Setup

- Connect motor driver and encoders to Arduino as per `motor_encoders_vel.ino` pin definitions.
- Connect Arduino to Raspberry Pi via USB.
- Connect LIDAR to Raspberry Pi via USB/serial.
- Ensure network connectivity between Raspberry Pi and Ubuntu VM (for ROS2 communication).

## Software Setup

### Prerequisites
- ROS2 Humble or later (tested on ROS2 Jazzy)
- Python 3.8 or later
- Required ROS2 packages (Nav2, SLAM Toolbox, RViz, etc.)
- Arduino IDE (for uploading firmware)

### Installation

1. Create a ROS2 workspace (on both Raspberry Pi and Ubuntu VM):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Copy or clone the provided `src` folder contents into your workspace `src` directory.

3. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the workspace:
```bash
colcon build
```

5. Source the workspace:
```bash
source install/setup.bash
```

## Usage Guide

### 1. Flash Arduino
- Upload `For_Arduino/motor_encoders_vel/motor_encoders_vel.ino` to Arduino using Arduino IDE.

### 2. Launch Robot Software (on Raspberry Pi)
- Start serial and controller nodes to bridge Arduino and ROS2.
- Launch LIDAR driver (default: YDLIDAR, see below for other LIDARs).
- Launch `serbot_description` to publish the URDF robot model.

### 3. Run SLAM or Navigation (on Ubuntu VM)
- Launch SLAM:
```bash
ros2 launch serbot_slam serbot_slam.launch.py
```
- Drive the robot to create a map. Save the map when done.
- Launch Navigation:
```bash
ros2 launch serbot_navigation serbot_navigation.launch.py
```
- Use RViz to set initial pose and navigation goals.

### Using a Different LIDAR
If you have a different LIDAR, replace the `ydlidar_ros2_driver` node with your LIDAR's ROS2 driver. Make sure the driver publishes to the `/scan` topic, or remap the topic as needed. Update your launch files accordingly. Most ROS2 LIDAR drivers (e.g., RPLIDAR, Hokuyo, Slamtec) can be used with minimal changes.

## Configuration Files

- SLAM: `For_Vm/src/serbot_slam/config/slam_params.yaml`
- Navigation: `For_Vm/src/serbot_navigation/config/controller_params_fast.yaml`, `advanced_amcl_params.yaml`

## Troubleshooting

- **Serial Issues**: Check USB connections and device names.
- **Transform Errors**: Ensure static transforms are running.
- **Navigation/SLAM Issues**: Verify sensor data in RViz, adjust parameters as needed.

## Tested Platforms

- ROS2 Jazzy (recommended)
- Raspberry Pi 5
- Arduino Uno

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contact

For support or queries, contact:
- Email: chawkigh2250@gmail.com
