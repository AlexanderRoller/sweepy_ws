# Sweepy Bot

Sweepy Bot is an autonomous mobile robot designed for sweeping tasks. This README provides instructions on how to set up and run the robot stack, SLAM, navigation, joystick control, and controlling the brushes through a relay control.

## Getting Started

### Prerequisites

Ensure you have the following installed:
- ROS 2
- Relevant ROS 2 packages and dependencies

### Repository Setup

Clone the repository and navigate to the workspace:

```sh
git clone https://github.com/AlexanderRoller/sweepy_ws.git
cd sweepy_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Launching the Robot

### 1. Robot Stack

Launch the robot stack which includes drivers and sensor nodes:

```sh
ros2 launch sweeper_bot sweepy_launch.py
```

### 2. SLAM

To perform simultaneous localization and mapping (SLAM), use the following command:

```sh
ros2 launch sweeper_bot online_async_launch.py
```

### 3. Navigation

For autonomous navigation, launch the Nav2 stack:

```sh
ros2 launch sweeper_bot navigation_launch.py
```

### 4. Joystick Controller

If you want to control the robot manually using a joystick:

```sh
ros2 launch sweeper_bot joystick_launch.py
```

### 5. Turning the brushes ON 

Launch the relay controller node that establishes a serial communication between NVIDIA Jetson Orin Nano and Adafruit Huzzah ESP 32 Feather board to turn ON/OFF the main and side brushes:

```sh
ros2 launch sweeper_bot relay_controller_launch.py
```

## Configuration Files

### `sweepy_launch.py`
Contains configurations for initializing the robot stack, including sensors and drivers.

### `slam_toolbox-params.yaml`
SLAM setup for creating and updating the map in real-time.

### `nav2_params.yaml`
Nav2 setup for path planning and autonomous navigation.

### `joystick.yaml`
Joystick controller setup for manual robot control.

### `relay_controller_launch.py`
Custom node configuration for turning ON the main and side brushes.

## Usage

### Running the Full Stack

To launch the entire robot stack, SLAM, and navigation individually:

```sh
ros2 launch sweeper_bot sweepy_launch.py
ros2 launch sweeper_bot online_async_launch.py
ros2 launch sweeper_bot navigation_launch.py
```

To launch everything for nav2 and joystick control

```sh
ros2 launch sweeper_bot lex_dev_launch.py
```

To launch robot stack and relay control

```sh
ros2 launch sweeper_bot vednshee_dev_launch.py
```

### Manual Control

For manual control with a joystick, use the joystick launch file:

```sh
ros2 launch sweeper_bot joystick_launch.py
```

## Dependencies/Help 

Project could not have been done without these people

- [Roboclaw Hardware Interface](https://github.com/dumbotics/roboclaw_hardware_interface)
- [Roboclaw Serial](https://github.com/dumbotics/roboclaw_serial)
- [Nav2](https://github.com/ros-navigation/navigation2)
- [Articubot One](https://github.com/joshnewans/articubot_one)
- [ROS2 Control](https://github.com/ros-controls/ros2_control)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Sick Scan](https://github.com/SICKAG/sick_scan_xd)
- [Robot Localization](https://github.com/cra-ros-pkg/robot_localization)
- lex/vedanshee 😎

## Troubleshooting

### Common Issues

- **Connection Issues**: Ensure all hardware is properly connected and configured.
- **Dependency Errors**: Make sure all dependencies are installed and sourced.
