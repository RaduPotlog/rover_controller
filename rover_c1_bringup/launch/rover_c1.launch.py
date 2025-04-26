from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rover_c1_description"),
                    "urdf", 
                    "rover_c1.urdf.xacro"
        	]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rover_c1_bringup"),
            "config",
            "rover_c1_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rover_c1_description"), "rviz", "rover_c1.rviz"]
    )

    joy_config_file = PathJoinSubstitution(
        [FindPackageShare("rover_c1_teleop"), "config", "rover_c1_joystick.yaml"]
    )

    teleop_config_file = PathJoinSubstitution(
        [FindPackageShare("rover_c1_teleop"), "config", "rover_c1_teleop.yaml"]
    )

    joy_node = Node(
	    package='joy',
	    executable='joy_node',
	    parameters=[joy_config_file],
    )

    teleop_node = Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
	    parameters=[teleop_config_file],
	)
    
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        remappings=[
            ("/cmd_vel_in", "/cmd_vel"),
        ]
    )
                  
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/rover_c1_base_controller/cmd_vel", "/cmd_vel_out"),
        ],
    )
    
    robot_ardupilot_controller_node = Node(
        package="rover_c1_ardupilot_controller",
        executable="rover_c1_ardupilot_controller",
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file", robot_controllers],
    )
    
    pid_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pid_controller_left_wheel_joint", "pid_controller_right_wheel_joint", "--param-file", robot_controllers],
    )
    
    rover_c1_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover_c1_base_controller", "--param-file", robot_controllers],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_robot_base_after_pid_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pid_controllers_spawner,
            on_exit=[rover_c1_base_controller_spawner],
        )
    )
    
    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rover_c1_base_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    nodes = [
        #joy_node,
    	#teleop_node,
    	#twist_stamper,
        control_node,
        robot_state_pub_node,
        pid_controllers_spawner,
        #rover_c1_base_controller_spawner,
        #robot_ardupilot_controller_node,
        delay_robot_base_after_pid_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
