from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    hand_joystick_share_dir = get_package_share_directory('hand_joystick_interfaces')
    mediapipe_share_dir = get_package_share_directory('mediapipe_mocap')
    offline_media_share_dir = get_package_share_directory('offline_media_publisher')
    
    # Config files
    hand_joystick_config = os.path.join(hand_joystick_share_dir, 'config', 'default_parameters.yaml')
    hand_landmarks_config = os.path.join(mediapipe_share_dir, 'config', 'hand_landmarks_node.yaml')
    video_config = os.path.join(offline_media_share_dir, 'config', 'video_publisher.yaml')

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
        parameters=[hand_landmarks_config]
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
        viewer_node
    ]

    return LaunchDescription([
        folder_path_arg,
        fps_arg,
        hand_joystick_config_arg,
    ] + nodes)
