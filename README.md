# Sweepy Bot

Sweepy Bot is an autonomous mobile robot designed for sweeping tasks. This README provides instructions on how to set up and run the robot stack, SLAM, navigation, joystick control, and controlling the brushes through relay control.

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

Launch the robot stack, which includes drivers, sensor nodes, and relay control:

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

## Configuration Files

### `sweepy_launch.py`
Contains configurations for initializing the robot stack, including sensors, drivers, and relay control.

### `slam_toolbox-params.yaml`
SLAM setup for creating and updating the map in real-time.

### `nav2_params.yaml`
Nav2 setup for path planning and autonomous navigation.

### `joystick.yaml`
Joystick controller setup for manual robot control.

## Usage

### Running the Full Stack

To launch the entire robot stack, SLAM, and navigation individually:

```sh
ros2 launch sweeper_bot sweepy_launch.py
ros2 launch sweeper_bot online_async_launch.py
ros2 launch sweeper_bot navigation_launch.py
```

To launch everything for Nav2 and SLAM:

```sh
ros2 launch sweeper_bot lex_dev_launch.py
```


### Joystick Control Details

When using a Bluetooth controller:
- The **left joystick button** controls the movement of the robot.
- An **enable button** must also be held down.
- On a PS4 controller:
  - The **left trigger** enables half speed.
  - The **right trigger** enables full speed.
  - The **bumpers** control the turning on and off of the brushes.

## Dependencies/Help

This project could not have been done without these people and resources:

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

Feel free to reach out for further assistance or inquiries.
