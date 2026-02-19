import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # --------------------------------------------------------------------------
    # 1. Configuration & Arguments
    # --------------------------------------------------------------------------
    folder_path = LaunchConfiguration('folder_path')

    declared_arguments = [
        DeclareLaunchArgument(
            'folder_path',
            default_value='',
            description='Path to folder containing videos/images for offline_media_publisher'
        ),
    ]

    # --------------------------------------------------------------------------
    # 2. Offline Media Publisher Node
    # --------------------------------------------------------------------------
    offline_media_share_dir = get_package_share_directory('offline_media_publisher')
    video_config = os.path.join(offline_media_share_dir, 'config', 'video_publisher.yaml')

    offline_media_node = Node(
        package='offline_media_publisher',
        executable='video_publisher',
        name='video_publisher',
        output='screen',
        parameters=[
            video_config,
            {'folder_path': folder_path}
        ]
    )

    # --------------------------------------------------------------------------
    # 3. MediaPipe Hand Landmarks Node
    # --------------------------------------------------------------------------
    mediapipe_share_dir = get_package_share_directory('mediapipe_mocap')
    hand_landmarks_config = os.path.join(mediapipe_share_dir, 'config', 'hand_landmarks_node.yaml')

    hand_landmarks_node = Node(
        package='mediapipe_mocap',
        executable='hand_landmarks_node',
        name='hand_landmarks_node',
        output='screen',
        parameters=[hand_landmarks_config]
    )

    # --------------------------------------------------------------------------
    # 4. Hand Joystick Interface Node
    # --------------------------------------------------------------------------
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
    # 5. Fractional Teleoperation Node
    # --------------------------------------------------------------------------
    fractional_teleop_config = PathJoinSubstitution([
        FindPackageShare("fractional_teleoperation"), "config", "node_params.yaml"
    ])

    fractional_teleoperation_node = Node(
        package='fractional_teleoperation',
        executable='fractional_teleoperation_node',
        name='fractional_teleoperation_node',
        output='screen',
        parameters=[fractional_teleop_config],
    )

    # --------------------------------------------------------------------------
    # 6. Launch Description
    # --------------------------------------------------------------------------
    nodes_to_start = [
        offline_media_node,
        hand_landmarks_node,
        hand_joystick_node,
        fractional_teleoperation_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
