# MPU9250 Driver for ROS2

This repository contains a ROS2 package that interfaces with an MPU9250 sensor over I2C. The accelerometer and gyroscope are calibrated on node startup (the sensor should be placed on a flat surface with the z-axis pointing up and should remain stationary during calibration). Calibration can be turned off in the parameters file. The current implementation allows for customization of the gyroscope and accelerometer ranges, digital low pass filter (DLPF) settings, and sensor offsets.

## Dependencies
- Python 3
- ROS 2 Humble
- `smbus2` Python library (for I2C communication with the MPU9250 sensor)

You can install the `smbus2` library using pip:

```bash
pip install smbus2
```

## Setup

Parameters can be configured in the `params/mpu9250.yaml` file. The following parameters are available:

```yaml
calibrate: True
gyro_range: 0
accel_range: 0
dlpf_bandwidth: 2
gyro_x_offset: 0.0 # [deg/s]
gyro_y_offset: 0.0 # [deg/s]
gyro_z_offset: 0.0 # [deg/s]
accel_x_offset: 0.0 # [m/s²]
accel_y_offset: 0.0 # [m/s²]
accel_z_offset: 0.0 # [m/s²]
frequency: 100 # [Hz]
```

### Build the Package

To build the package in your ROS 2 workspace:

```bash
colcon build --packages-select mpu9250driver
```

### Source the Workspace

After building the package, source the setup script:

```bash
. install/setup.bash
```

### Launch the Node

To launch the node with the configured parameters:

```bash
ros2 launch mpu9250driver mpu9250driver_launch.py
```

## Disclaimer

This project is still a **work in progress**. While I will continue to make improvements as time allows, please understand that not everything may work perfectly. Contributions and bug fixes are welcome! Feel free to create a pull request if you'd like to contribute.

