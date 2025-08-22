#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
import yaml
import math
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading
from ament_index_python.packages import get_package_share_directory
import os

class RVizMoveItVisualizer(Node):
    def __init__(self):
        super().__init__('rviz_moveit_visualizer')
        
        # Joint names matching the URDF
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
            'gripper_joint'
        ]
        
        # Load calibration data
        self.load_calibration_data()
        
        # Publishers for joint states
        self.joint_state_pub = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )
        
        # Publisher for displaying robot trajectory in RViz
        self.display_trajectory_pub = self.create_publisher(
            DisplayTrajectory,
            '/move_group/display_planned_path',
            10
        )
        
        # Service client to capture current arm position
        self.position_capture_client = self.create_client(Trigger, 'record_position')
        
        # Timer to periodically capture and publish current position (10 Hz)
        self.capture_timer = self.create_timer(0.1, self.capture_and_publish_position)
        
        # Flag to track if service is available
        self.service_available = False
        
        # Store current joint positions
        self.current_joint_positions = [0.0] * len(self.joint_names)
        
        self.get_logger().info("RViz MoveIt Visualizer started")
        self.get_logger().info("Waiting for position capture service...")
        
        # Wait for the service to be available
        if not self.position_capture_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().warn("Position capture service not available after 15 seconds")
            self.get_logger().warn("Make sure the hardware interface is running and controllers are started")
            self.service_available = False
        else:
            self.get_logger().info("Position capture service is available")
            self.service_available = True
    
    def load_calibration_data(self):
        """Load calibration data from the calibration.yaml file"""
        try:
            # Try to find the calibration file
            package_share = get_package_share_directory('so100_bidirectional')
            calib_file = os.path.join(package_share, 'config', 'calibration.yaml')
            
            # If not found in install, try source directory
            if not os.path.exists(calib_file):
                import rclpy
                from ament_index_python.packages import get_package_prefix
                pkg_prefix = get_package_prefix('so100_bidirectional')
                workspace_root = os.path.dirname(os.path.dirname(pkg_prefix))
                source_calib_file = os.path.join(
                    workspace_root, 'src', 'brukg_so100', 'so100_bidirectional', 'config', 'calibration.yaml'
                )
                if os.path.exists(source_calib_file):
                    calib_file = source_calib_file
            
            with open(calib_file, 'r') as f:
                self.calibration_data = yaml.safe_load(f)
                
            self.get_logger().info(f"Loaded calibration data from: {calib_file}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration data: {e}")
            self.calibration_data = None
    
    def ticks_to_radians(self, ticks, joint_name):
        """Convert servo ticks to radians using calibration data"""
        if self.calibration_data is None or 'joints' not in self.calibration_data:
            # Fallback: assume 4096 ticks per full rotation, centered at 2048
            return (ticks - 2048) * 2 * math.pi / 4096.0
        
        joint_calib = self.calibration_data['joints'].get(joint_name)
        if joint_calib is None:
            # Fallback for this joint
            return (ticks - 2048) * 2 * math.pi / 4096.0
        
        min_ticks = joint_calib['min']['ticks']
        max_ticks = joint_calib['max']['ticks']
        range_ticks = max_ticks - min_ticks
        
        if range_ticks == 0:
            return 0.0
        
        # Convert to normalized position first (0 to 1)
        normalized = (ticks - min_ticks) / range_ticks
        # Then convert to radians (-π to π)
        return (normalized * 2.0 - 1.0) * math.pi
    
    def capture_and_publish_position(self):
        """Capture current arm position and publish as joint states"""
        if not self.service_available or not self.position_capture_client.service_is_ready():
            # Try to reconnect to service if it wasn't available before
            if not self.service_available and self.position_capture_client.service_is_ready():
                self.service_available = True
                self.get_logger().info("Position capture service is now available")
            return
        
        # Call the service to get current position
        request = Trigger.Request()
        future = self.position_capture_client.call_async(request)
        
        # Use a callback to handle the response
        future.add_done_callback(self.position_capture_callback)
    
    def position_capture_callback(self, future):
        """Handle the response from position capture service"""
        try:
            response = future.result()
            if response.success:
                # Parse the YAML response to get joint positions
                joint_data = yaml.safe_load(response.message)
                
                # Convert ticks to radians for each joint
                for i, joint_name in enumerate(self.joint_names):
                    if joint_name in joint_data:
                        ticks = joint_data[joint_name]['ticks']
                        radians = self.ticks_to_radians(ticks, joint_name)
                        self.current_joint_positions[i] = radians
                
                # Publish joint states
                self.publish_joint_states()
                
        except Exception as e:
            self.get_logger().debug(f"Error in position capture: {e}")
    
    def publish_joint_states(self):
        """Publish current joint positions as JointState message"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions
        joint_state.velocity = [0.0] * len(self.joint_names)
        joint_state.effort = [0.0] * len(self.joint_names)
        
        self.joint_state_pub.publish(joint_state)
    
    def create_display_trajectory(self, waypoints, execution_time=5.0):
        """Create a DisplayTrajectory message for RViz visualization"""
        display_trajectory = DisplayTrajectory()
        display_trajectory.model_id = "so100_bidirectional"
        
        # Create the trajectory
        trajectory = RobotTrajectory()
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = self.joint_names
        
        # Add waypoints
        for i, waypoint in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.velocities = [0.0] * len(self.joint_names)
            point.accelerations = [0.0] * len(self.joint_names)
            point.time_from_start = Duration(sec=int(i * execution_time / len(waypoints)), 
                                           nanosec=int((i * execution_time / len(waypoints) % 1) * 1e9))
            joint_trajectory.points.append(point)
        
        trajectory.joint_trajectory = joint_trajectory
        display_trajectory.trajectory.append(trajectory)
        
        return display_trajectory
    
    def demonstrate_movement(self):
        """Demonstrate some pre-defined movements for visualization"""
        self.get_logger().info("Starting movement demonstration...")
        
        # Define some interesting poses
        home_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        raised_pose = [0.0, -1.0, 1.5, -0.5, 0.0, 0.5]
        side_pose = [1.57, -0.5, 1.0, -0.5, 1.57, 0.8]
        extended_pose = [0.0, -1.5, 2.0, 0.0, 0.0, 1.0]
        
        waypoints = [home_pose, raised_pose, side_pose, extended_pose, home_pose]
        
        # Create and publish display trajectory
        display_trajectory = self.create_display_trajectory(waypoints, execution_time=10.0)
        self.display_trajectory_pub.publish(display_trajectory)
        
        self.get_logger().info("Movement demonstration published to RViz")

def main():
    rclpy.init()
    
    visualizer = RVizMoveItVisualizer()
    
    # Create a multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(visualizer)
    
    # Start a thread to run the executor
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()
    
    try:
        # Wait a bit for initialization
        import time
        time.sleep(2.0)
        
        print("\n" + "="*60)
        print("RViz MoveIt Visualizer is running!")
        print("="*60)
        print("Features:")
        print("- Real-time arm position visualization in RViz")
        print("- MoveIt integration for motion planning")
        print("- Calibration-based position conversion")
        print("\nCommands:")
        print("- 'd' + Enter: Demonstrate sample movements")
        print("- 'q' + Enter: Quit")
        print("="*60)
        
        while rclpy.ok():
            try:
                user_input = input("\nEnter command (d for demo, q to quit): ").strip().lower()
                
                if user_input == 'q':
                    break
                elif user_input == 'd':
                    visualizer.demonstrate_movement()
                else:
                    print("Unknown command. Use 'd' for demo or 'q' to quit.")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
    
    finally:
        visualizer.get_logger().info("Shutting down...")
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main()
