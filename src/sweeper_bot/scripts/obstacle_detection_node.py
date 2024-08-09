#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('object_detection_node')
        self.get_logger().info('Initializing spin robot node...')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Node initialized and ready to spin the robot.')

    def timer_callback(self):
        move_cmd = Twist()
        move_cmd.angular.z = 0.5  # Adjust this value for faster or slower spinning
        move_cmd.linear.x = 0.0   # Ensure no forward/backward movement

        # Publish the command
        self.publisher_.publish(move_cmd)
        self.get_logger().info('Publishing spin command...')

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()