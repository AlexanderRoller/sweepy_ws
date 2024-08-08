import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

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
        )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('sweeper_bot'),'launch','navigation_launch.py'
        )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    return LaunchDescription([
        sweepy,
        joystick,
        slam,
        nav2, 
    ])
