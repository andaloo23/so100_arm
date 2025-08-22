#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_hardware",
            default_value="true",
            description="Start hardware interface (set to false for simulation only)"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port for robot communication"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware for testing without physical robot"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "auto_start_controllers",
            default_value="true",
            description="Automatically start controllers on launch (set to true to enable robot movement)"
        )
    )

    # Package directories
    so100_pkg = FindPackageShare("so100_arm")

    # Robot description with hardware parameters
    robot_description_content = ParameterValue(
        Command(
            [
                "xacro",
                " ",
                PathJoinSubstitution([so100_pkg, "config", "so100_arm.urdf.xacro"]),
                " serial_port:=", LaunchConfiguration("serial_port"),
                " use_fake_hardware:=", LaunchConfiguration("use_fake_hardware"),
            ]
        ),
        value_type=str
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    # Robot semantic description (SRDF file - read as string)
    robot_description_semantic_content = ParameterValue(
        Command(['cat ', PathJoinSubstitution([so100_pkg, "config", "so100_arm.srdf"])]),
        value_type=str
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Debug log messages
    log_use_rviz = LogInfo(msg=["use_rviz: ", LaunchConfiguration("use_rviz")])
    log_use_hardware = LogInfo(msg=["use_hardware: ", LaunchConfiguration("use_hardware")])
    log_auto_start_controllers = LogInfo(msg=["auto_start_controllers: ", LaunchConfiguration("auto_start_controllers")])
    
    # Core nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([so100_pkg, "config", "ros2_controllers.yaml"]),
        ],
        output="both",
        remappings=[
            ('/joint_states', '/joint_states'),
        ],
    )

    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("auto_start_controllers")),
    )
    
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("auto_start_controllers")),
    )
    
    # The new explicit planning pipeline configuration
    planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "planning_adapters": [
                "default_planner_request_adapters/AddTimeOptimalParameterization",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStatePathConstraints",
            ],
        },
    }

    # MoveIt node - now with the correct parameter passing
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            PathJoinSubstitution([so100_pkg, "config", "kinematics.yaml"]),
            PathJoinSubstitution([so100_pkg, "config", "joint_limits.yaml"]),
            PathJoinSubstitution([so100_pkg, "config", "ompl_planning.yaml"]),
            PathJoinSubstitution([so100_pkg, "config", "moveit_controllers.yaml"]),
            {"use_sim_time": False},
            planning_pipeline_config, # Explicitly add the planning pipeline config
        ],
    )

    # RViz node
    rviz_config_file = PathJoinSubstitution([so100_pkg, "config", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            PathJoinSubstitution([so100_pkg, "config", "kinematics.yaml"]),
            PathJoinSubstitution([so100_pkg, "config", "joint_limits.yaml"]),
            PathJoinSubstitution([so100_pkg, "config", "ompl_planning.yaml"]),
            PathJoinSubstitution([so100_pkg, "config", "moveit_controllers.yaml"]),
            planning_pipeline_config, # Explicitly add the planning pipeline config
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(declared_arguments + [
        log_use_rviz,
        log_use_hardware,
        log_auto_start_controllers,
        
        robot_state_publisher_node,
        ros2_control_node,
        
        TimerAction(
            period=2.0,
            actions=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                gripper_controller_spawner,
            ]
        ),
        
        TimerAction(
            period=8.0,
            actions=[
                move_group_node,
                rviz_node,
            ]
        )
    ])
