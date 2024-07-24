#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import logging
import threading
import time 

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # Set up logging
        logging.basicConfig(filename='robot_log.txt', level=logging.INFO, format='%(asctime)s - %(message)s')
        self.logger = logging.getLogger()

        # Set up periodic timer for obstacles
        self.obstacle_check_timer = self.create_timer(0.25, self.check_for_obstacles)

        # Placeholder for the latest LiDAR scan data and heading data 
        self.latest_scan_data = None

        # Placeholder for the previous command 
        self.prev_cmd = Twist()
        
        # Storing initial counts 
        self.left_count = 0 
        self.right_count = 0 
        self.front_count = 0 

    def lidar_callback(self, msg):
        # Store the latest scan data 
        self.latest_scan_data = msg 

    def check_for_obstacles(self):
        if self.latest_scan_data is None:
            return

        ranges = np.array(self.latest_scan_data.ranges)
        ranges[ranges == 0] = np.inf

        # Split the ranges into left right and front parts
        # 
        left_indices = slice(0, len(ranges) // 3)
        front_indices = slice(len(ranges) // 3, 2 * len(ranges) // 3) 
        right_indices = slice(2 * len(ranges) // 3, len(ranges))
        left_ranges = ranges[left_indices]
        front_ranges = ranges[front_indices]
        right_ranges = ranges[right_indices]

        obstacles = {'left': False, 'front': False, 'right': False}

        min_left = np.min(left_ranges)
        min_front = np.min(front_ranges)
        min_right = np.min(right_ranges)

        if min_left < 0.5: # here 0.5 is the obstacle threshold 
            obstacles['left'] = True 
            self.get_logger().info(f'Obstacle detected in left region with range {min_left} meters.')

        if min_right < 0.5:
            obstacles['right'] = True 
            self.get_logger().info(f'Obstacle detected in right region with range {min_right} meters.')
        if min_front < 0.5:
            obstacles['front'] = True 
            self.get_logger().info(f'Obstacle detected in front region with range {min_front} meters.')

        if obstacles['front']:
            self.front_count += 1
            self.right_count = 0 
            self.left_count = 0 
            if self.front_count >= 3:
                self.get_logger().info('Obstacle detected on the front, stopping and deciding turn direction.')
                self.logger.info('Obstacle detected on the front, stopping and deciding turning direction.')
                self.stop_and_decide_turn()
            
        elif obstacles['right']:
            self.right_count += 1 
            self.left_count = 0 
            self.front_count = 0 
            if self.right_count >= 3:
                self.get_logger().info('Obstacle detected on the right, turning left.')
                self.logger.info('Obstacle detected on the right, turning left.')
                self.turn_left()
            
        elif obstacles['left']:
            self.left_count += 1 
            self.right_count = 0 
            self.front_count = 0 
            if self.left_count >= 3:
                self.get_logger().info('Obstacle detected on left, turning right')
                self.logger.info('Obstacles detected on both left, turning right')
                self.turn_right()
            
        else:
            self.front_count = 0 
            self.left_count = 0 
            self.right_count = 0 
            self.get_logger().info('No obstacles detected, moving forward.')
            self.logger.info('No obstacles detected, moving forward.')
            self.move_forward()

    def move_forward(self):
        cmd = Twist()
        cmd.linear.x = 0.6
        cmd.angular.z = 0.0
        self.pub_cmd_vel.publish(cmd)
        self.get_logger().info('Command: Move Forward')
        self.logger.info('Command: Move forward')

    def turn_left(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 1.0
        self.pub_cmd_vel.publish(cmd)
        self.get_logger().info('Command: Turn Left')
        self.logger.info('Command: Turn left')
        self.left_count = 0 
        self.right_count = 0 
        self.front_count = 0 
       
    def turn_right(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -1.0
        self.pub_cmd_vel.publish(cmd)
        self.get_logger().info('Command: Turn Right')
        self.logger.info('Command: Turn right')
        self.left_count = 0 
        self.right_count = 0 
        self.front_count = 0 
       
    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd_vel.publish(cmd)
        self.get_logger().info('Command: Stop')
        self.logger.info('Command: Stop')

    def move_backward(self):
        cmd = Twist()
        cmd.linear.x = -0.6
        cmd.angular.z = 0.0
        self.pub_cmd_vel.publish(cmd)
        self.get_logger().info('Command: Move Backward')
        self.logger.info('Command: Move Backward')

    def spot_turn_right(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -2.0
        self.pub_cmd_vel.publish(cmd)
        self.get_logger().info('Command: Spot turning to right')
        self.logger.info('Command: Spot turning right')

    def spot_turn_left(self):
        cmd = Twist()
        cmd.linear.x = 0.0 
        cmd.angular.z = 2.0
        self.pub_cmd_vel.publish(cmd)
        self.get_logger().info('Command: Spot turning to left')
        self.logger.info('Command: Spot turning to left')

    def stop_and_decide_turn(self):
        self.stop()
        threading.Timer(0.3, self.move_backward_and_decide_turn).start()
        #self.stop()
        #threading.Timer(0.3, self.move_backward).start()
        #threading.Timer(1.0, self.spot_turn_right).start()
        #threading.Timer(3.5, self.move_forward).start()
        #self.front_count = 0 
        #self.left_count = 0 
        #self.right_count = 0 
    def move_backward_and_decide_turn(self):
        self.move_backward()
        threading.Timer(1.0, self.decide_turn).start()
        
    def decide_turn(self):
        if self.latest_scan_data is None:
            self.stop()
            return
        ranges = np.array(self.latest_scan_data.ranges)
        ranges[ranges==0] = np.inf
        left_ranges = ranges[:len(ranges) // 3]
        right_ranges = ranges[2 * len(ranges) // 3:]
        min_left = np.min(left_ranges)
        min_right = np.min(right_ranges)
        if min_right >= 0.5:
            self.spot_turn_left()
        elif min_left >= 0.5:
            self.spot_turn_left()
        else:
            self.get_logger().info('Obstacles on both sides after backing up, stopping.')
            self.logger.info('Obstacles on both sides after backing up, stopping')
            self.stop()
            return 
        threading.Timer(3.5, self.move_forward).start()

    def publish_cmd(self, cmd):
        if cmd.linear.x != self.prev_cmd.linear.x or cmd.angular.z != self.prev_cmd.angular.z:
            self.prev_cmd = cmd 
            self.pub_cmd_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
