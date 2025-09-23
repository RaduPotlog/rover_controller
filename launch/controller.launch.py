#!/usr/bin/env python3

# Copyright 2020 ros2_control Development Team
# Copyright 2025 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():
    
    common_dir_path = LaunchConfiguration("common_dir_path")
    declare_common_dir_path_arg = DeclareLaunchArgument(
        "common_dir_path",
        default_value="",
        description="Path to the common configuration directory.",
    )
    
    rover_controller_common_dir = PythonExpression(
        [
            "'",
            common_dir_path,
            "/rover_controller' if '",
            common_dir_path,
            "' else '",
            FindPackageShare("rover_controller"),
            "'",
        ]
    )

    robot_model = LaunchConfiguration("robot_model")
    description_pkg = FindPackageShare("rover_description")
    description_common_dir = PythonExpression(
        [
            "'",
            common_dir_path,
            "/rover_description",
            "' if '",
            common_dir_path,
            "' else '",
            description_pkg,
            "'",
        ]
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="rover_a1"),
        description="Specify robot model",
        choices=["rover_a1", "rover_a2"],
    )

    components_config_path = LaunchConfiguration("components_config_path")
    declare_components_config_path_arg = DeclareLaunchArgument(
        "components_config_path",
        default_value=PathJoinSubstitution([description_common_dir, "config", "components.yaml"]),
        description=(
            "Additional components configuration file. Components described in this file."
        ),
    )

    wheel_type = LaunchConfiguration("wheel_type")
    controller_config_path = LaunchConfiguration("controller_config_path")
    declare_controller_config_path_arg = DeclareLaunchArgument(
        "controller_config_path",
        default_value=PathJoinSubstitution(
            [
                rover_controller_common_dir,
                "config",
                PythonExpression(["'", wheel_type, "_controller.yaml'"]),
            ]
        ),
        description=(
            "Path to controller configuration file."
        ),
    )

    log_level = LaunchConfiguration("log_level")
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "FATAL"],
        description="Logging level",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    publish_robot_state = LaunchConfiguration("publish_robot_state")
    declare_publish_robot_state_arg = DeclareLaunchArgument(
        "publish_robot_state",
        default_value="False",
        description=(
            "Whether to launch the robot_state_publisher node."
            "When set to False, users should publish their own robot description."
        ),
        choices=["True", "true", "False", "false"],
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
        choices=["True", "true", "False", "false"],
    )

    wheel_config_path = LaunchConfiguration("wheel_config_path")
    declare_wheel_config_path_arg = DeclareLaunchArgument(
        "wheel_config_path",
        default_value=PathJoinSubstitution(
            [description_pkg, "config", PythonExpression(["'", wheel_type, ".yaml'"])]
        ),
        description=(
            "Path to wheel configuration file."
        ),
    )

    default_wheel_type = {"rover_a1": "wheel_01", "rover_a2": "wheel_01"}
    declare_wheel_type_arg = DeclareLaunchArgument(
        "wheel_type",
        default_value=PythonExpression([f"{default_wheel_type}['", robot_model, "']"]),
        description=(
            "Specify the wheel type."
        ),
        choices=["wheel_01", "custom"],
    )

    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    ns_controller_config_path = ReplaceString(controller_config_path, {"<namespace>/": ns})

    urdf_file = PythonExpression(["'", robot_model, ".urdf.xacro'"])
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_pkg, "urdf", urdf_file]),
            " use_sim:=",
            use_sim,
            " wheel_config_file:=",
            wheel_config_path,
            " controller_config_file:=",
            ns_controller_config_path,
            " namespace:=",
            namespace,
            " components_config_path:=",
            components_config_path,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    joint_state_broadcaster_log_unit = PythonExpression(
        [
            "'",
            namespace,
            "' + '.joint_state_broadcaster' if '",
            namespace,
            "' else 'joint_state_broadcaster'",
        ]
    )
    controller_manager_log_unit = PythonExpression(
        [
            "'",
            namespace,
            "' + '.controller_manager' if '",
            namespace,
            "' else 'controller_manager'",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ns_controller_config_path],
        namespace=namespace,
        remappings=[
            ("drive_controller/cmd_vel_unstamped", "cmd_vel"),
            ("drive_controller/odom", "odometry/wheels"),
            ("drive_controller/transition_event", "_drive_controller/transition_event"),
            # ("hardware_controller/robot_driver_state", "hardware/robot_driver_state"),
            # ("hardware_controller/motor_torque_enable", "hardware/motor_torque_enable"),
            (
                "joint_state_broadcaster/transition_event",
                "_joint_state_broadcaster/transition_event",
            ),
        ],
        condition=UnlessCondition(use_sim),
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    namespace_ext = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=["--ros-args", "--disable-stdout-logs"],
        parameters=[robot_description, {"frame_prefix": namespace_ext}],
        namespace=namespace,
        condition=IfCondition(publish_robot_state),
    )

    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "drive_controller",
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            "10",
        ],
        namespace=namespace,
        emulate_tty=True,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            "10",
        ],
        namespace=namespace,
        emulate_tty=True,
    )

    delay_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[drive_controller_spawner],
        )
    )

    actions = [
        declare_common_dir_path_arg,
        declare_robot_model_arg,
        declare_wheel_type_arg,
        declare_components_config_path_arg,
        declare_controller_config_path_arg,
        declare_namespace_arg,
        declare_publish_robot_state_arg,
        declare_use_sim_arg,
        declare_wheel_config_path_arg,
        declare_log_level_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_drive_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(actions)