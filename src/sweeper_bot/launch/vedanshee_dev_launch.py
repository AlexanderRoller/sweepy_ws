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

    #obs_det = IncludeLaunchDescription(
     #   PythonLaunchDescriptionSource([os.path.join(
      #  get_package_share_directory('sweeper_bot'),'launch','obstacle_detection_launch.py'
       # )])
    #)

    relay_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('sweeper_bot'), 'launch', 'relay_controller_launch.py'
        )])
    )

    return LaunchDescription([
        sweepy,
        joystick,
        #obs_det,
        relay_controller,
    ])