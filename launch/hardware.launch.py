#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for robot communication'
    )
    
    # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command(
            [
                "xacro",
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("so100_bidirectional"), "config", "so100_bidirectional.urdf.xacro"]
                ),
                " serial_port:=", LaunchConfiguration("serial_port"),
            ]
        ),
        value_type=str
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Hardware interface
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare("so100_bidirectional"),
                "config",
                "hardware_config.yaml"
            ])
        ],
        output="both",
    )

    return LaunchDescription([
        serial_port_arg,
        robot_state_publisher_node,
        controller_manager_node,
    ])
