import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory

def generate_launch_description():
    pkg_share = FindPackageShare(package='sweeper_bot').find('sweeper_bot')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'urdf_config.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', LaunchConfiguration('model')]),
            'use_sim_time': True
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[default_model_path],
        output='screen'
    )
    """
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{
            'use_sim_time': True
        }],
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    """
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'sweeper', '-topic', 'robot_description'],
        output='screen'
    )

    gazebo_params_file = os.path.join(get_package_share_directory('sweeper_bot'),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )
    
    realsense_params_file = os.path.join(get_package_share_directory('sweeper_bot'),'config','realsense_params.yaml')

    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        parameters=[realsense_params_file, {'pointcloud.enable': True}, 
                    {'depth_module.global_time_enabled': True}]

    )
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    nav2_params_file = os.path.join(get_package_share_directory('sweeper_bot'), 'config', 'costmap_params.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_file),
        launch_arguments={'params_file': nav2_params_file}.items()
    )

    return LaunchDescription([
        #DeclareLaunchArgument(name='gui', default_value='True',
        #                      description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                              description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        #joint_state_publisher_gui_node,
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        rviz_node,
        realsense,
        #nav2
    ])
