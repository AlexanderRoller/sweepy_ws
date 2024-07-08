from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_toolbox_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
    laserscan_params = os.path.join(get_package_share_directory('sweeper_bot'), 'config', 'laserscan_params.yaml')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_file),
            launch_arguments={
                'slam_params_file': os.path.join(get_package_share_directory('sweeper_bot'), 'config', 'slam_toolbox_params.yaml'),
                'use_sim_time': 'false'
            }.items()
        ),
    
        # Node(
        #     package='pointcloud_to_laserscan',
        #     executable='pointcloud_to_laserscan_node',
        #     name='pointcloud_to_laserscan',
        #     output='screen',
        #     parameters=[{
        #         'range_min': 0.1,
        #         'range_max': 10.0,
        #         'angle_min': -1.5708,  # -90 degrees
        #         'angle_max': 1.5708,   # 90 degrees
        #         'angle_increment': 0.0174533,  # 1 degree
        #         'scan_time': 0.1,
        #         'min_height': 0.4,  # Adjust based on your sensor height
        #         'max_height': 0.6,  # Adjust based on your sensor height
        #         'target_frame': 'camera_link',
        #         'transform_tolerance': 0.1,
        #         'use_inf': True,
        #     }],
        #     remappings=[
        #         ('cloud_in', '/camera/realsense2_camera/depth/color/points'),
        #         ('scan', '/scan')
        #     ]
        # ),

        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[('depth', '/camera/realsense2_camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/camera/realsense2_camera/depth/camera_info')],
            parameters=[laserscan_params]
            ) 
            
    ])
