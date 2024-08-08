#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleDetectionNode(Node):

    def __init__(self):
        super().__init__('obstacle_detection_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Node initialized and subscriptions created.')

    def listener_callback(self, msg):
        ranges = msg.ranges
        num_values = len(ranges)
        left_side = ranges[:num_values//3]
        right_side = ranges[-num_values//3:]
        
        min_left = min(left_side)
        min_right = min(right_side)

        self.get_logger().info(f'Left side min range: {min_left}')
        self.get_logger().info(f'Right side min range: {min_right}')

        move_cmd = Twist()
        
        # Check for obstacles
        if min_left < 0.5:
            # Turn right
            self.get_logger().info('Obstacle detected on the left, turning right.')
            move_cmd.angular.z = -0.5
            move_cmd.linear.x = 0.0
        elif min_right < 0.5:
            # Turn left
            self.get_logger().info('Obstacle detected on the right, turning left.')
            move_cmd.angular.z = 0.5
            move_cmd.linear.x = 0.0
        else:
            # Go straight
            self.get_logger().info('No obstacles detected, going straight.')
            move_cmd.angular.z = 0.0
            move_cmd.linear.x = 0.5

        # Publish the command
        self.publisher_.publish(move_cmd)
        self.get_logger().info('Movement command published.')

def main(args=None):
    rclpy.init(args=args)
    obstacle_detection_node= ObstacleDetectionNode()
    rclpy.spin(obstacle_detection_node)
    obstacle_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
