import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --------------------------------------------------------------------------
    # 1. Configuration & Arguments
    # --------------------------------------------------------------------------
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulation = LaunchConfiguration("use_simulation")
    spacenav = LaunchConfiguration('spacenav')
    use_actuator_interface = LaunchConfiguration("use_actuator_interface")
    can_port = LaunchConfiguration("can_port")
    host_id = LaunchConfiguration("host_id")
    use_poc2 = LaunchConfiguration("use_POC2")

    declared_arguments = [
        DeclareLaunchArgument(
            "gui", 
            default_value="false", 
            description="Start RViz2 and Gazebo GUI automatically with this launch file."
        ),
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false', 
            description='If true, use simulated clock'
        ),
        DeclareLaunchArgument(
            "use_simulation", 
            default_value="false", 
            description="Launch in simulation mode with Gazebo"
        ),
        DeclareLaunchArgument(
            "use_actuator_interface", 
            default_value="true", 
            description="Use VESCInterface to control the robot. Set to false for simulation"
        ),
        DeclareLaunchArgument(
            "can_port", 
            default_value="can0", 
            description="CAN Port for VESC Communication"
        ),
        DeclareLaunchArgument(
            "host_id", 
            default_value="45", 
            description="Host CAN ID for VESC Communication"
        ),
        DeclareLaunchArgument(
            "use_POC2", 
            default_value="true", 
            description="Use POC2 urdf"
        ),
        DeclareLaunchArgument(
            'spacenav', 
            default_value='True', 
            description='If the spacenav 3D mouse is used'
        ),
        DeclareLaunchArgument(
            'folder_path',
            default_value='',
            description='Path to folder containing videos/images for offline_media_publisher'
        ),
        DeclareLaunchArgument(
            'use_depth',
            default_value='false',
            description='Enable RGB+depth fusion for metric 3D landmarks'
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/aligned_depth_to_color/image_raw',
            description='Depth image topic aligned with RGB image_topic'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='CameraInfo topic used for depth projection'
        ),
        DeclareLaunchArgument(
            'depth_time_tolerance_ms',
            default_value='10.0',
            description='Maximum allowed RGB/depth timestamp mismatch (ms)'
        ),
        DeclareLaunchArgument(
            'depth_min_m',
            default_value='0.05',
            description='Minimum valid depth in meters'
        ),
        DeclareLaunchArgument(
            'depth_max_m',
            default_value='2.0',
            description='Maximum valid depth in meters'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='empty_world_headless.world',
            description='Gazebo world file to load'
        )
    ]

    # --------------------------------------------------------------------------
    # 2. File Paths & Substitutions
    # --------------------------------------------------------------------------
    pkg_share = FindPackageShare("explorer_description")
    
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([pkg_share, "urdf", "explorer.urdf.xacro"]), " ",
        "use_ignition:=", use_simulation,
        " use_actuator_interface:=", use_actuator_interface,
        " can_port:=", can_port,
        " host_id:=", host_id,
        " use_POC2:=", use_poc2,
    ])
    robot_description = {"robot_description": robot_description_content}
    
    # Config Files for fractional teleoperation controller
    fractional_teleop_config = PathJoinSubstitution([
        FindPackageShare("fractional_teleoperation"), "config", "explorer_params.yaml"
    ])

    # --------------------------------------------------------------------------
    # 3. Robot Launch Includes (Simulation or Hardware)
    # --------------------------------------------------------------------------
    robot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("explorer_bringup"), "/launch/simulation_base.launch.py"]),
        launch_arguments={
            'use_POC2': use_poc2,
            'gui': gui,
            'use_sim_time': use_sim_time,
            'rviz_delay': '0.0',
            'world_file': LaunchConfiguration('world_file'),
            'extra_controllers_config': fractional_teleop_config
        }.items(),
        condition=IfCondition(use_simulation)
    )

    robot_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("explorer_bringup"), "/launch/hardware_base.launch.py"]),
        launch_arguments={
            'gui': gui,
            'use_sim_time': use_sim_time,
            'use_actuator_interface': 'True',
            'can_port': can_port,
            'host_id': host_id,
            'use_POC2': use_poc2,
            'rviz_delay': '5.0', 
            'extra_controllers_config': fractional_teleop_config
        }.items(),
        condition=UnlessCondition(use_simulation) 
    )

    # --------------------------------------------------------------------------
    # 4. Controller Spawners
    # --------------------------------------------------------------------------
    spawner_qontrol = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["qontrol_explorer", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawner_fractional_teleop_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fractional_teleoperation_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # --------------------------------------------------------------------------
    # 5. Teleoperation Nodes (Hand Joystick Interface with MediaPipe)
    # --------------------------------------------------------------------------
    # Offline media publisher for video input
    offline_media_share_dir = get_package_share_directory('offline_media_publisher')
    video_config = os.path.join(offline_media_share_dir, 'config', 'video_publisher.yaml')

    offline_media_node = Node(
        package='offline_media_publisher',
        executable='video_publisher',
        name='video_publisher',
        output='screen',
        parameters=[
            video_config,
            {'folder_path': LaunchConfiguration('folder_path')}
        ]
    )

    # MediaPipe hand landmarks configuration
    mediapipe_share_dir = get_package_share_directory('mediapipe_mocap')
    hand_landmarks_config = os.path.join(mediapipe_share_dir, 'config', 'hand_landmarks_node.yaml')

    hand_landmarks_node = Node(
        package='mediapipe_mocap',
        executable='hand_landmarks_node',
        name='hand_landmarks_node',
        output='screen',
        parameters=[
            hand_landmarks_config,
            {
                'use_depth': LaunchConfiguration('use_depth'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'depth_time_tolerance_ms': LaunchConfiguration('depth_time_tolerance_ms'),
                'depth_min_m': LaunchConfiguration('depth_min_m'),
                'depth_max_m': LaunchConfiguration('depth_max_m'),
            }
        ]
    )

    # Hand joystick configuration
    hand_joystick_config_file = PathJoinSubstitution([
        FindPackageShare("hand_joystick_interfaces"), "config", "default_parameters.yaml"
    ])

    hand_joystick_node = Node(
        package='hand_joystick_interfaces',
        executable='hand_joystick_node',
        name='hand_joystick_interface',
        output='screen',
        parameters=[hand_joystick_config_file],
    )

    # --------------------------------------------------------------------------
    # 6. Event Handlers
    # --------------------------------------------------------------------------
    # Start fractional teleoperation controller after qontrol is loaded
    start_fractional_teleop_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_qontrol,
            on_exit=[spawner_fractional_teleop_controller]
        )
    )

    # --------------------------------------------------------------------------
    # 7. Launch Description
    # --------------------------------------------------------------------------
    nodes_to_start = [
        robot_simulation,
        robot_hardware,
        offline_media_node,
        hand_landmarks_node,
        hand_joystick_node,
        spawner_qontrol,
        start_fractional_teleop_event,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
