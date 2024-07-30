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
        self.subscription

    def imu_callback(self, msg):
        imu_data = msg.data
        self.get_logger().info(f"IMU Data received: {imu_data}")

def main(args=None):
    rclpy.init(args=args)
    imu_data_node = IMUDataNode()
    rclpy.spin(imu_data_node)
    imu_data_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()