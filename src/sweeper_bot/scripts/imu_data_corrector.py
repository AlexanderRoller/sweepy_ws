#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu

class IMUDataCorrector(Node):
    def __init__(self):
        super().__init__('imu_data_corrector')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.accel_subscription = self.create_subscription(
            Imu,
            '/camera/realsense2_camera/accel/sample',
            self.accel_callback,
            qos_profile
        )
        self.gyro_subscription = self.create_subscription(
            Imu,
            '/camera/realsense2_camera/gyro/sample',
            self.gyro_callback,
            qos_profile
        )

        self.accel_publisher = self.create_publisher(Imu, '/camera/realsense2_camera/accel/corrected', qos_profile)
        self.gyro_publisher = self.create_publisher(Imu, '/camera/realsense2_camera/gyro/corrected', qos_profile)

    def accel_callback(self, data):
        corrected_data = Imu()

        # Copy the header
        corrected_data.header = data.header

        # Swap angular_velocity to linear_acceleration with coordinate transform
        corrected_data.linear_acceleration.x = data.angular_velocity.x  # Adjust according to your coordinate system
        corrected_data.linear_acceleration.y = data.angular_velocity.y  # Adjust according to your coordinate system
        corrected_data.linear_acceleration.z = data.angular_velocity.z  # Adjust according to your coordinate system

        corrected_data.orientation = data.orientation
        corrected_data.orientation_covariance = data.orientation_covariance
        corrected_data.angular_velocity_covariance = data.angular_velocity_covariance
        corrected_data.linear_acceleration_covariance = data.linear_acceleration_covariance

        self.accel_publisher.publish(corrected_data)

    def gyro_callback(self, data):
        corrected_data = Imu()

        # Copy the header
        corrected_data.header = data.header

        # Swap linear_acceleration to angular_velocity with coordinate transform
        corrected_data.angular_velocity.x = data.linear_acceleration.x  # Adjust according to your coordinate system
        corrected_data.angular_velocity.y = data.linear_acceleration.y  # Adjust according to your coordinate system
        corrected_data.angular_velocity.z = data.linear_acceleration.z  # Adjust according to your coordinate system

        corrected_data.orientation = data.orientation
        corrected_data.orientation_covariance = data.orientation_covariance
        corrected_data.angular_velocity_covariance = data.angular_velocity_covariance
        corrected_data.linear_acceleration_covariance = data.linear_acceleration_covariance

        self.gyro_publisher.publish(corrected_data)

def main(args=None):
    rclpy.init(args=args)
    imu_data_corrector = IMUDataCorrector()
    rclpy.spin(imu_data_corrector)
    imu_data_corrector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
