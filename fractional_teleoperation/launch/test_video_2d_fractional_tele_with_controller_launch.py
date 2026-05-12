from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    hand_joystick_share_dir = get_package_share_directory('hand_joystick_interfaces')
    mediapipe_share_dir = get_package_share_directory('mediapipe_mocap')
    offline_media_share_dir = get_package_share_directory('offline_media_publisher')
    fractional_tele_share_dir = get_package_share_directory('fractional_teleoperation')
    
    # Config files
    hand_joystick_config = os.path.join(hand_joystick_share_dir, 'config', 'explorer_hand_parameters.yaml')
    hand_landmarks_config = os.path.join(mediapipe_share_dir, 'config', 'hand_landmarks_node.yaml')
    video_config = os.path.join(offline_media_share_dir, 'config', 'video_publisher.yaml')
    
    # Test configs
    fractional_tele_config = os.path.join(fractional_tele_share_dir, 'config', 'test_params.yaml')
    test_urdf_path = os.path.join(fractional_tele_share_dir, 'config', 'test_robot.urdf')

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
        description='Path to hand joystick 2D configuration file'
    )

    use_depth_arg = DeclareLaunchArgument(
        'use_depth',
        default_value='false',
        description='Enable RGB+depth fusion for metric 3D landmarks'
    )

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/aligned_depth_to_color/image_raw',
        description='Depth image topic aligned with RGB image_topic'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/color/camera_info',
        description='CameraInfo topic used for depth projection'
    )

    depth_time_tolerance_ms_arg = DeclareLaunchArgument(
        'depth_time_tolerance_ms',
        default_value='10.0',
        description='Maximum allowed RGB/depth timestamp mismatch (ms)'
    )

    depth_min_m_arg = DeclareLaunchArgument(
        'depth_min_m',
        default_value='0.05',
        description='Minimum valid depth in meters'
    )

    depth_max_m_arg = DeclareLaunchArgument(
        'depth_max_m',
        default_value='2.0',
        description='Maximum valid depth in meters'
    )

    controller_config_arg = DeclareLaunchArgument(
        'controller_config',
        default_value=fractional_tele_config,
        description='Path to fractional teleoperation controller test configuration file'
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
    
    # Hand landmarks node with 2D inference (MediaPipe)
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

    # Hand joystick 2D interface node
    hand_joystick_node = Node(
        package='hand_joystick_interfaces',
        executable='hand_joystick_node',
        name='hand_joystick_interface',
        output='screen',
        parameters=[LaunchConfiguration('hand_joystick_config')],
        remappings=[
            ('hand_landmarks', '/hand_landmarks'),
            ('teleop_cmd', '/teleop_cmd')
        ]
    )

    # Controller manager node for ros2_control (with minimal test URDF)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            LaunchConfiguration('controller_config'),
            {
                'robot_description': open(test_urdf_path).read(),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
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
            {'robot_description': open(test_urdf_path).read()},
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
                'landmarks_topic': '/hand_landmarks',
                'window_name': 'Hand Landmarks Viewer',
            }
        ]
    )

    nodes = [
        offline_video_node,
        hand_landmarks_node,
        hand_joystick_node,
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
        use_sim_time_arg,
        use_depth_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        depth_time_tolerance_ms_arg,
        depth_min_m_arg,
        depth_max_m_arg,
    ] + nodes)
