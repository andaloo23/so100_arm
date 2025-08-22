#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Declare arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )
    
    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='true',
        description='Start hardware interface (set to false for simulation only)'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for robot communication'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use fake hardware for testing without physical robot'
    )
    
    auto_start_controllers_arg = DeclareLaunchArgument(
        'auto_start_controllers',
        default_value='false',
        description='Automatically start controllers on launch (set to true to enable robot movement)'
    )
    
    # Package directories
    so100_pkg = FindPackageShare("so100_bidirectional")
    
    # Robot description with hardware parameters
    robot_description_content = ParameterValue(
        Command(
            [
                "xacro",
                " ",
                PathJoinSubstitution([so100_pkg, "config", "so100_bidirectional.urdf.xacro"]),
                " serial_port:=", LaunchConfiguration("serial_port"),
                " use_fake_hardware:=", LaunchConfiguration("use_fake_hardware"),
            ]
        ),
        value_type=str
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    # Robot semantic description (SRDF file - read as string)
    robot_description_semantic_content = ParameterValue(
        Command(['cat ', PathJoinSubstitution([so100_pkg, "config", "so100_bidirectional.srdf"])]),
        value_type=str
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    
    # MoveIt configuration parameters
    kinematics_yaml = PathJoinSubstitution([so100_pkg, "config", "kinematics.yaml"])
    joint_limits_yaml = PathJoinSubstitution([so100_pkg, "config", "joint_limits.yaml"])
    moveit_controllers_yaml = PathJoinSubstitution([so100_pkg, "config", "moveit_controllers.yaml"])
    ompl_planning_yaml = PathJoinSubstitution([so100_pkg, "config", "ompl_planning.yaml"])
    planning_pipeline_yaml = PathJoinSubstitution([so100_pkg, "config", "planning_pipeline.yaml"])
    

    

    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Hardware interface node (conditional)
    hardware_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([so100_pkg, "config", "hardware_config.yaml"]),
            PathJoinSubstitution([so100_pkg, "config", "ros2_controllers.yaml"]),
        ],
        output="both",
        condition=IfCondition(LaunchConfiguration("use_hardware")),
    )
    
    # Joint state broadcaster (always needed for visualization)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_hardware")),
    )
    
    # Arm controller (conditional - only if auto_start_controllers is true)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("auto_start_controllers")),
    )
    
    # Gripper controller (conditional - only if auto_start_controllers is true)
    gripper_controller_spawner = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("auto_start_controllers")),
    )
    
    # MoveIt move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group", 
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            moveit_controllers_yaml,
            ompl_planning_yaml,
            planning_pipeline_yaml,
            {"use_sim_time": False},
        ],
    )
    
    # RViz node
    rviz_config_file = PathJoinSubstitution([so100_pkg, "config", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "info"],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_planning_yaml,
            planning_pipeline_yaml,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    
    # Position visualizer script (conditional - only with hardware)
    visualizer_node = Node(
        package="so100_bidirectional",
        executable="rviz_moveit_visualizer.py",
        name="rviz_moveit_visualizer",
        output="screen",
        parameters=[robot_description],
        condition=IfCondition(LaunchConfiguration("use_hardware")),
    )
    
    # Delay some nodes to ensure proper startup order
    delayed_move_group = TimerAction(
        period=3.0,
        actions=[move_group_node]
    )
    
    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )
    
    delayed_visualizer = TimerAction(
        period=6.0,
        actions=[visualizer_node]
    )
    
    # Debug log messages
    log_use_rviz = LogInfo(msg=["use_rviz: ", LaunchConfiguration("use_rviz")])
    log_use_hardware = LogInfo(msg=["use_hardware: ", LaunchConfiguration("use_hardware")])
    log_auto_start_controllers = LogInfo(msg=["auto_start_controllers: ", LaunchConfiguration("auto_start_controllers")])

    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        use_hardware_arg,
        serial_port_arg,
        use_fake_hardware_arg,
        auto_start_controllers_arg,
        
        # Debug log messages
        log_use_rviz,
        log_use_hardware, 
        log_auto_start_controllers,
        
        # Core nodes
        robot_state_publisher_node,
        hardware_node,
        
        # Controllers (with delays)
        TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=2.5, actions=[arm_controller_spawner]),
        TimerAction(period=2.7, actions=[gripper_controller_spawner]),
        
        # MoveIt and visualization
        delayed_move_group,
        delayed_rviz,
        delayed_visualizer,
    ])
