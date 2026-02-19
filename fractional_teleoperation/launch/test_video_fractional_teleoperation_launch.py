from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    hand_joystick_share_dir = get_package_share_directory('hand_joystick_interfaces')
    mediapipe_share_dir = get_package_share_directory('mediapipe_mocap')
    offline_media_share_dir = get_package_share_directory('offline_media_publisher')
    fractional_tele_share_dir = get_package_share_directory('fractional_teleoperation')
    
    # Config files
    hand_joystick_config = os.path.join(hand_joystick_share_dir, 'config', '3d_parameters.yaml')
    hand_landmarks_config = os.path.join(mediapipe_share_dir, 'config', 'hand_landmarks_node_depth_inference.yaml')
    video_config = os.path.join(offline_media_share_dir, 'config', 'video_publisher.yaml')
    fractional_tele_config = os.path.join(fractional_tele_share_dir, 'config', 'franka_params.yaml')

    # Launch arguments
    folder_path_arg = DeclareLaunchArgument(
        'folder_path',
        default_value='',
        description='Path to folder containing videos (required)'
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Publishing rate in Hz (overrides native video FPS)'
    )
    
    hand_joystick_config_arg = DeclareLaunchArgument(
        'hand_joystick_config',
        default_value=hand_joystick_config,
        description='Path to hand joystick 3D configuration file'
    )

    # Auto-generate robot_description from explorer.urdf.xacro
    explorer_description_dir = get_package_share_directory('explorer_description')
    explorer_urdf_path = os.path.join(explorer_description_dir, 'urdf', 'explorer.urdf.xacro')
    
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            explorer_urdf_path,
        ]),
        value_type=str
    )

    controller_config_arg = DeclareLaunchArgument(
        'controller_config',
        default_value=fractional_tele_config,
        description='Path to fractional teleoperation controller configuration file'
    )

    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='If true, use simulation (Gazebo), if false use real hardware'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='If true, use simulated clock'
    )

    # Offline video publisher node
    offline_video_node = Node(
        package='offline_media_publisher',
        executable='video_publisher',
        name='video_publisher',
        output='screen',
        parameters=[
            video_config,
            {
                'folder_path': LaunchConfiguration('folder_path'),
                'fps': LaunchConfiguration('fps'),
            }
        ]
    )
    
    # Hand landmarks node with depth inference (MediaPipe)
    hand_landmarks_node = Node(
        package='mediapipe_mocap',
        executable='hand_landmarks_node_depth_inference',
        name='hand_landmarks_depth_inference_node',
        output='screen',
        parameters=[hand_landmarks_config]
    )

    # Hand joystick 3D inference interface node
    hand_joystick_3d_node = Node(
        package='hand_joystick_interfaces',
        executable='hand_joystick_3d_node',
        name='hand_joystick_3d_interface',
        output='screen',
        parameters=[LaunchConfiguration('hand_joystick_config')],
        remappings=[
            ('hand_landmarks_depth', '/hand_landmarks_depth'),
            ('cmd_vel', '/cmd_vel')
        ]
    )

    # Controller manager node for ros2_control
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            LaunchConfiguration('controller_config'),
            {'robot_description': robot_description_content},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Spawner for fractional teleoperation controller
    fractional_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'fractional_teleoperation_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen',
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Viewer node (optional - visualize hand landmarks)
    viewer_node = Node(
        package='mediapipe_mocap',
        executable='viewer_node',
        name='hand_landmarks_viewer',
        output='screen',
        parameters=[
            {
                'image_topic': '/camera/color/image_raw',
                'landmarks_topic': '/hand_landmarks_depth',
                'window_name': 'Hand Landmarks 3D Viewer',
            }
        ]
    )

    nodes = [
        offline_video_node,
        hand_landmarks_node,
        hand_joystick_3d_node,
        robot_state_publisher_node,
        controller_manager_node,
        fractional_controller_spawner,
        viewer_node
    ]

    return LaunchDescription([
        folder_path_arg,
        fps_arg,
        hand_joystick_config_arg,
        controller_config_arg,
        simulation_arg,
        use_sim_time_arg,
    ] + nodes)
