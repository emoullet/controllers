import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --------------------------------------------------------------------------
    # 1. Configuration & Arguments
    # --------------------------------------------------------------------------
    declared_arguments = [
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
    ]

    # --------------------------------------------------------------------------
    # 2. rosbridge websocket
    # --------------------------------------------------------------------------
    rosbridge_websocket_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            ])
        )
    )

    # --------------------------------------------------------------------------
    # 3. image_transport republish compressed -> raw
    # --------------------------------------------------------------------------
    republish_node = Node(
        package='image_transport',
        executable='republish',
        name='image_republish_compressed_to_raw',
        output='screen',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/image_raw'),
            ('out', '/image_raw_uncompressed'),
        ],
    )

    # --------------------------------------------------------------------------
    # 4. MediaPipe Hand Landmarks Node
    # --------------------------------------------------------------------------
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
                'image_topic': '/image_raw_uncompressed',
                'use_depth': LaunchConfiguration('use_depth'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'depth_time_tolerance_ms': LaunchConfiguration('depth_time_tolerance_ms'),
                'depth_min_m': LaunchConfiguration('depth_min_m'),
                'depth_max_m': LaunchConfiguration('depth_max_m'),
            }
        ]
    )

    # --------------------------------------------------------------------------
    # 5. Hand Joystick Interface Node
    # --------------------------------------------------------------------------
    hand_joystick_config_file = PathJoinSubstitution([
        FindPackageShare('hand_joystick_interfaces'), 'config', 'default_parameters.yaml'
    ])

    hand_joystick_node = Node(
        package='hand_joystick_interfaces',
        executable='hand_joystick_node',
        name='hand_joystick_interface',
        output='screen',
        parameters=[hand_joystick_config_file],
    )

    # --------------------------------------------------------------------------
    # 6. Fractional Teleoperation Node
    # --------------------------------------------------------------------------
    fractional_teleop_config = PathJoinSubstitution([
        FindPackageShare('fractional_teleoperation'), 'config', 'node_params.yaml'
    ])

    fractional_teleoperation_node = Node(
        package='fractional_teleoperation',
        executable='fractional_teleoperation_node',
        name='fractional_teleoperation_node',
        output='screen',
        parameters=[fractional_teleop_config],
    )

    # --------------------------------------------------------------------------
    # 7. Launch Description
    # --------------------------------------------------------------------------
    nodes_to_start = [
        rosbridge_websocket_launch,
        republish_node,
        hand_landmarks_node,
        hand_joystick_node,
        fractional_teleoperation_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
