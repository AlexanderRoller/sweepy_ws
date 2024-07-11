import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    
    sweepy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('sweeper_bot'),'launch','sweepy_launch.py'
        )])
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('sweeper_bot'),'launch','joystick_launch.py'
        )])
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('sweeper_bot'),'launch','online_async_launch.py'
        )])
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('sweeper_bot'),'launch','navigation_launch.py'
        )])
    )

    return LaunchDescription([
        sweepy,
        joystick,
        slam,
        nav2,
    ])