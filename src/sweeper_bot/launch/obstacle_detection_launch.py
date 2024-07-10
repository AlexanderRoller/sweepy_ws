import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    obstacle_detection_node = Node(
        package='sweeper_bot',
        executable='obstacle_detection_node.py',
        name='obstacle_detection_node',
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        obstacle_detection_node
    ])
