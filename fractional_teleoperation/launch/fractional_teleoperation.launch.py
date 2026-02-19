from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value='',
        description='Robot description URDF/XACRO content'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('fractional_teleoperation'),
            'config',
            'franka_params.yaml'
        ]),
        description='Path to controller configuration file'
    )

    # Controller manager node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {'robot_description': LaunchConfiguration('robot_description')}
        ],
        output='screen',
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

    return LaunchDescription([
        robot_description_arg,
        config_file_arg,
        controller_manager,
        fractional_controller_spawner,
    ])
