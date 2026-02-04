import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    
    # --- PATHS ---
    pkg_project = get_package_share_directory('final_project')
    pkg_iiwa = get_package_share_directory('iiwa_description')
    pkg_fra2mo = get_package_share_directory('ros2_fra2mo')

    models_path = os.path.join(pkg_project, 'models')

    # ====================================================
    # 1. FIX RESOURCE PATH 
    # ====================================================
    install_dir = os.path.dirname(os.path.dirname(os.path.dirname(pkg_project)))
    workspace_share = os.path.join(install_dir, 'share')
    
    gz_resource_path = f"{models_path}:{workspace_share}:{os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')}"

    print(f"\n--- DEBUG RESOURCE PATH ---\n{gz_resource_path}\n---------------------------\n")

    set_gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gz_resource_path
    )

    # -------------------
    # GAZEBO WORLD
    # -------------------
    world = PathJoinSubstitution([
        FindPackageShare('final_project'),
        'worlds',
        'warehouse.world'
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    # ====================================================
    # ROBOT 1 :  IIWA 
    # ====================================================
    
    iiwa_desc_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([pkg_iiwa, 'config', 'iiwa.config.xacro']),
        ' ', 'use_sim:=true',
        ' ', 'use_fake_hardware:=true',
        ' ', 'base_frame_file:=base_frame.yaml'
    ])
    
    iiwa_robot_desc = {'robot_description': ParameterValue(iiwa_desc_content, value_type=str)}

    # STATE PUBLISHER
    iiwa_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher', 
        output='screen',
        parameters=[iiwa_robot_desc],
    )

    # SPAWN IIWA
    spawn_iiwa = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'iiwa',
            '-world', 'warehouse_world',
            '-allow_renaming', 'true'
        ],
    )

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    
    traj_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['iiwa_arm_trajectory_controller', '--controller-manager', '/controller_manager'],
    )

    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_iiwa, on_exit=[jsb_spawner])
    )
    delay_traj = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=jsb_spawner, on_exit=[traj_spawner])
    )

    # ====================================================
    # ROBOT 2 :  FRA2MO 
    # ====================================================

    fra2mo_desc_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([pkg_fra2mo, 'urdf', 'fra2mo.urdf.xacro']),
    ])

    fra2mo_robot_desc = {'robot_description': ParameterValue(fra2mo_desc_content, value_type=str)}

    fra2mo_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='fra2mo_state_publisher',
        namespace='fra2mo', 
        output='screen',
        parameters=[fra2mo_robot_desc, {'use_sim_time': True}],
    )

    #SPAWN FRA2MO
    spawn_fra2mo = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/fra2mo/robot_description', 
            '-name', 'fra2mo',
            '-world', 'warehouse_world',
            '-allow_renaming', 'true',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
    )

    odom_tf = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        parameters=[{"use_sim_time": True}]
    )

    # ====================================================
    # BRIDGE
    # ====================================================
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            
            # IIWA
            "/iiwa/grasp/attach@std_msgs/msg/Empty@ignition.msgs.Empty",
            "/iiwa/grasp/detach@std_msgs/msg/Empty@ignition.msgs.Empty",
            "/iiwa_camera@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/iiwa_camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",

            # FRA2MO
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
            "/model/fra2mo/pose@geometry_msgs/msg/Pose@ignition.msgs.Pose",
            "/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
            "/fra2mo/camera@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/fra2mo/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            "/fra2mo/grasp/fix_package@std_msgs/msg/Empty@ignition.msgs.Empty",
            "/fra2mo/grasp/release_package@std_msgs/msg/Empty@ignition.msgs.Empty",
        ],
        output="screen",
        parameters=[{'use_sim_time': True}],
        remappings=[
        ('/model/fra2mo/tf', '/tf'),
        ]
    )

    # --- FIX TF: PONTE TRA GAZEBO E URDF ---
    tf_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_fix',
        # Argomenti: x y z yaw pitch roll frame_padre frame_figlio
        arguments=['0', '0', '0', '0', '0', '0', 'fra2mo/base_footprint', 'base_footprint'],
        parameters=[{'use_sim_time': True}]
    )

    # --- FIX CAMERA TF: Collega il nome logico (URDF) al nome reale (Gazebo) ---
    tf_camera_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_camera_fix',
        # Collega il frame ottico del robot al frame che arriva dall'immagine
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link_optical', 'fra2mo/base_footprint/fra2mo_camera'],
        parameters=[{'use_sim_time': True}]
    )

    # 1. Bridge per CREARE oggetti (Spawn)
    spawn_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="spawn_bridge",
        arguments=[
            # Mappa il servizio Gazebo -> Servizio ROS
            "/world/warehouse_world/create@ros_gz_interfaces/srv/SpawnEntity"
        ],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    # 2. Bridge per RIMUOVERE oggetti (Delete)
    delete_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="delete_bridge",
        arguments=[
            # Mappa il servizio Gazebo -> Servizio ROS
            "/world/warehouse_world/remove@ros_gz_interfaces/srv/DeleteEntity"
        ],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        iiwa_state_publisher,
        spawn_iiwa,
        delay_jsb,
        delay_traj,
        fra2mo_state_publisher,
        spawn_fra2mo,
        odom_tf,
        bridge,
        tf_fix,
        tf_camera_fix,
        spawn_bridge,
        delete_bridge
    ])