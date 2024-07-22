import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory

def generate_launch_description():
    pkg_share = FindPackageShare(package='sweeper_bot').find('sweeper_bot')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'urdf_config.rviz')
    realsense_params_file = os.path.join(get_package_share_directory('sweeper_bot'), 'config', 'realsense_params.yaml')
    ekf_params_file = os.path.join(get_package_share_directory('sweeper_bot'), 'config', 'ekf_params.yaml')

    robot_description_content = Command(['xacro ', LaunchConfiguration('model')])
    robot_description = {'robot_description': robot_description_content}

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sweeper_bot'),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}],
        arguments=[default_model_path],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        parameters=[realsense_params_file, {'pointcloud.enable': True}, {'depth_module.global_time_enabled': True}],
        remappings=[
            ('/camera/realsense2_camera/gyro/sample', '/camera/realsense2_camera/accel/sample'),
            ('/camera/realsense2_camera/accel/sample', '/camera/realsense2_camera/gyro/sample')
        ]
    )

    imu_corrector_node = Node(
        package='sweeper_bot',
        executable='imu_data_corrector.py',
        name='imu_data_corrector',
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file]
    )

    robot_controllers_path = PathJoinSubstitution([
        FindPackageShare("sweeper_bot"),
        "config",
        "roboclaw_controllers.yaml"
    ])

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers_path],
        output="both",
        #arguments=['--ros-args', '--log-level', 'debug'],
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.5,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="both",
            )
        ]
    )

    robot_controller_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
                output="both",
            )
        ]
    )
    
    twist_mux_params = os.path.join(get_package_share_directory('sweeper_bot'),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
        )
    
    laserscan_params = os.path.join(get_package_share_directory('sweeper_bot'), 'config', 'laserscan_params.yaml')
    laser_scan =Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[('depth', '/camera/realsense2_camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/camera/realsense2_camera/depth/camera_info')],
            parameters=[laserscan_params]
        ) 
    
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    tim_launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/sick_tim_7xx.launch')
    
    sick_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output= 'screen',
        arguments=[
            tim_launch_file_path,
            'frame_id:=sick_lidar_frame',
            'tf_base_frame_id:=sick_lidar_frame',
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        #joint_state_publisher_node,
        rsp,
        rviz_node,
        #realsense_node,
        ekf_node,
        #imu_corrector_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        twist_mux,
        #laser_scan,
        sick_node,
    ])
