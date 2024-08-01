#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class IMUDataNode(Node):
    def __init__(self):
        super().__init__('imu_data_node')
        self.subscription = self.create_subscription(
            String,
            'imu_data',
            self.imu_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def imu_callback(self, msg):
        imu_data = msg.data
        gyro_data = self.parse_data(imu_data, 'Angular velocity')
        accel_data = self.parse_data(imu_data, 'Linear acceleration')
        
        if gyro_data and accel_data:
            self.get_logger().info(
                f"IMU Data:\n"
                f"  Angular velocity:\n"
                f"    x: {gyro_data.get('x', '0.00')}\n"
                f"    y: {gyro_data.get('y', '0.00')}\n"
                f"    z: {gyro_data.get('z', '0.00')}\n"
                f"  Linear acceleration:\n"
                f"    x: {accel_data.get('x', '0.00')}\n"
                f"    y: {accel_data.get('y', '0.00')}\n"
                f"    z: {accel_data.get('z', '0.00')}\n"
            )

    def parse_data(self, imu_data, data_type):
        try:
            start_index = imu_data.find(f"{data_type}:")
            if start_index == -1:
                return None
            data_lines = imu_data[start_index:].split("\n")[1:]
            data_dict = {}
            for line in data_lines:
                if line.strip() == "":
                    break
                if ": " in line:
                    axis, value = line.split(": ")
                    data_dict[axis.strip()] = float(value.strip())
            return data_dict
        except Exception as e:
            self.get_logger().error(f"Error parsing {data_type} data: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    imu_data_node = IMUDataNode()
    rclpy.spin(imu_data_node)
    imu_data_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

