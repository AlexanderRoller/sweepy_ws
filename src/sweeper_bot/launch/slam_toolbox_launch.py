from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_toolbox_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_file),
            launch_arguments={
                'slam_params_file': os.path.join(get_package_share_directory('sweeper_bot'), 'config', 'slam_toolbox_params.yaml'),
                'use_sim_time': 'false'
            }.items()
        ),          
    ])
