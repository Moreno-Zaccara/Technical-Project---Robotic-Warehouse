import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    # ---  CONFIGURAZIONE NAV2  ---
    fra2mo_dir = FindPackageShare('ros2_fra2mo')
    nav2_bringup_launch_file_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    
    # Parametri Navigazione
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([fra2mo_dir, 'maps', 'map.yaml']),
        description='Full path to map yaml file to load',
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([fra2mo_dir, 'config', 'navigation.yaml']),
        description='Full path to the ROS2 parameters file'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'
    )

    # Launch Nav2 (Map Server, AMCL, Controller, Planner, BT Navigator)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch_file_dir]),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    

    pose_yaml = PathJoinSubstitution([
        FindPackageShare('final_project'),
        'config',
        'iiwa_poses.yaml'
    ])

    aruco_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'marker_id': 0,       
            'marker_size': 0.15,    
            'reference_frame': 'base_footprint',
            'camera_frame':    'fra2mo/base_footprint/fra2mo_camera',
            'marker_frame': 'aruco_marker_frame',
            'image_is_rectified': True,
            'use_sim_time': True,
            'min_marker_size': 0.005
        }],
        remappings=[
            ('/camera_info', '/fra2mo/camera_info'),
            ('/image', '/fra2mo/camera')
        ]
    )

    iiwa_pick_and_place_node = Node(
        package='final_project',
        executable='iiwa_pick_and_place_node',
        name='iiwa_pick_and_place_node',
        output='screen',
        parameters=[{'pose_file': pose_yaml}]
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    
    # rqt_view_fra2mo = Node(
    #     package='rqt_image_view',
    #     executable='rqt_image_view',
    #     name='rqt_image_view',
    #     arguments=['/aruco_single/result'],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    # )
    # rqt_view_iiwa = Node(
    #     package='rqt_image_view',
    #     executable='rqt_image_view',
    #     name='rqt_image_view',
    #     arguments=['/iiwa_camera'],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    # )

    return LaunchDescription([
        # Args
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        
        # Nav2
        nav2_bringup_launch,
        
        # Aruco & IIWA
        aruco_node,
        gripper_spawner,
        iiwa_pick_and_place_node
        
    ])