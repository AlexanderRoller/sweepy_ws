#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

class NavigationDemo(Node):
    def __init__(self):
        super().__init__('navigation_demo')
        self.navigator = BasicNavigator()
        self.initial_pose_received = False

        # Subscriber to the /amcl_pose topic
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        if not self.initial_pose_received:
            self.get_logger().info('Received initial pose from /amcl_pose')
            self.set_initial_pose(msg)
            self.initial_pose_received = True

    def set_initial_pose(self, pose_msg):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose = pose_msg.pose  # Use the pose from /amcl_pose

        # Set the initial pose using the navigator
        self.navigator.setInitialPose(initial_pose)

        # Wait until the navigation stack is fully activated
        self.navigator.waitUntilNav2Active()

    def run_demo(self):
        # Go to our demo's first goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = -1.5314358510374584
        goal_pose.pose.position.y = -0.9838490079174844
        goal_pose.pose.orientation.w = 0.6468758287712141
        goal_pose.pose.orientation.z = -0.7625953462692746

        # Go to the pose
        self.navigator.goToPose(goal_pose)

        i = 0
        while not self.navigator.isTaskComplete():
            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    goal_pose.pose.position.x = 0.0
                    goal_pose.pose.position.y = 0.0
                    self.navigator.goToPose(goal_pose)

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        self.navigator.lifecycleShutdown()

def main():
    rclpy.init()
    navigation_demo = NavigationDemo()
    rclpy.spin(navigation_demo)
    navigation_demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
