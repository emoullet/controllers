from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --------------------------------------------------------------------------
    # 1. Configuration & Arguments
    # --------------------------------------------------------------------------
    declared_arguments = [
        DeclareLaunchArgument(
            'mouse_host',
            default_value='127.0.0.1',
            description='HTTP bind address for mouse joystick web page'
        ),
        DeclareLaunchArgument(
            'mouse_port',
            default_value='8765',
            description='HTTP bind port for mouse joystick web page'
        ),
        DeclareLaunchArgument(
            'auto_open_browser',
            default_value='true',
            description='Open the mouse joystick page automatically'
        ),
    ]

    # --------------------------------------------------------------------------
    # 2. Mouse Joystick Interface Node
    # --------------------------------------------------------------------------
    mouse_joystick_node = Node(
        package='mouse_joystick_interface',
        executable='mouse_joystick_server.py',
        name='mouse_joystick_interface',
        output='screen',
        parameters=[
            {
                'host': LaunchConfiguration('mouse_host'),
                'port': LaunchConfiguration('mouse_port'),
                'auto_open_browser': LaunchConfiguration('auto_open_browser'),
                'teleop_topic': 'teleop_cmd',
            }
        ],
    )

    # --------------------------------------------------------------------------
    # 3. Fractional Teleoperation Node
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
    # 4. Launch Description
    # --------------------------------------------------------------------------
    nodes_to_start = [
        mouse_joystick_node,
        fractional_teleoperation_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
