from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression


def generate_launch_description():
    # --------------------------------------------------------------------------
    # Configuration & Arguments
    # --------------------------------------------------------------------------
    gui = LaunchConfiguration("gui")
    use_simulation = LaunchConfiguration("use_simulation")

    use_actuator_interface = PythonExpression([
            "'false' if '", use_simulation, "' == 'true' else 'true'"
        ])
    declared_arguments = [
        DeclareLaunchArgument(
            "gui", 
            default_value="true", 
            description="Start RViz2 automatically with this launch file."
        ),
        DeclareLaunchArgument(
            "use_simulation", 
            default_value="false", 
            description="Whether to launch the Gazebo simulation environment"
        ),
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
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for the fractional teleoperation node (debug, info, warn, error)'
        ),
    ]

    # --------------------------------------------------------------------------
    # File Paths & Substitutions
    # --------------------------------------------------------------------------
    # Config Files   
    velocity_config = PathJoinSubstitution([
        FindPackageShare("fractional_teleoperation"), "config", "explorer_params.yaml"
    ])

    robot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("explorer_bringup"), "/launch/simulation_base.launch.py"]),
        launch_arguments={
            'use_POC2': "true",
            'gui': gui,
            'use_sim_time': use_simulation,
            'rviz_delay': '3.0',
            'extra_controllers_config': velocity_config, 
            'use_custom_controllers': "true"
        }.items(),
        condition=IfCondition(use_simulation)
    )

    robot_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("explorer_bringup"), "/launch/hardware_base.launch.py"]),
        launch_arguments={
            'gui': gui,
            'use_sim_time': use_simulation,
            'use_actuator_interface': use_actuator_interface,
            'can_port': "can0",
            'host_id': "45",
            'use_POC2': "true",
            'rviz_delay': '3.0', 
            'extra_controllers_config': velocity_config,
            'use_custom_controllers': "true"

        }.items(),
        condition=UnlessCondition(use_simulation) 
    )

    # --------------------------------------------------------------------------
    # Controllers spawner
    # --------------------------------------------------------------------------
    spawner_qontrol = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["qontrol_explorer", "--controller-manager", "/controller_manager"],
    )

    spawner_sandbox_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fractional_teleoperation_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawner_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # --------------------------------------------------------------------------
    # Other Nodes
    # --------------------------------------------------------------------------
    
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
    # Event Handlers
    # --------------------------------------------------------------------------
    delayed_spawner_qontrol = TimerAction(
        period=2.0,
        actions=[spawner_qontrol]
    )
    start_sandbox_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_qontrol,
            on_exit=[ spawner_sandbox_controller, spawner_gripper_controller]
        )
    )

    # --------------------------------------------------------------------------
    # Launch Description
    # --------------------------------------------------------------------------
    nodes_to_start = [
        robot_simulation,
        robot_hardware,
        delayed_spawner_qontrol,
        start_sandbox_event,
        # mouse_joystick_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)