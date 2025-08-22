#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import subprocess
import sys

class MoveItTestNode(Node):
    def __init__(self):
        super().__init__('moveit_test_node')
        self.get_logger().info('Starting MoveIt setup test...')
        
    def test_topics(self):
        """Test if required topics are available"""
        required_topics = [
            '/joint_states',
            '/robot_description', 
            '/tf',
            '/tf_static'
        ]
        
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
        available_topics = result.stdout.strip().split('\n')
        
        self.get_logger().info('Checking required topics...')
        for topic in required_topics:
            if topic in available_topics:
                self.get_logger().info(f'✓ {topic} - Available')
            else:
                self.get_logger().warn(f'✗ {topic} - Missing')
        
        return all(topic in available_topics for topic in required_topics)
    
    def test_controllers(self):
        """Test if controllers are running"""
        self.get_logger().info('Checking controllers...')
        
        try:
            result = subprocess.run(
                ['ros2', 'control', 'list_controllers'], 
                capture_output=True, 
                text=True, 
                timeout=10
            )
            
            if result.returncode == 0:
                self.get_logger().info('Controller manager is running')
                self.get_logger().info(f'Controllers status:\n{result.stdout}')
                return True
            else:
                self.get_logger().warn('Controller manager not responding')
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Controller manager service timeout')
            return False
    
    def test_moveit_services(self):
        """Test if MoveIt services are available"""
        moveit_services = [
            '/move_group/plan_kinematic_path',
            '/move_group/execute_trajectory'
        ]
        
        result = subprocess.run(['ros2', 'service', 'list'], capture_output=True, text=True)
        available_services = result.stdout.strip().split('\n')
        
        self.get_logger().info('Checking MoveIt services...')
        for service in moveit_services:
            if service in available_services:
                self.get_logger().info(f'✓ {service} - Available')
            else:
                self.get_logger().warn(f'✗ {service} - Missing')
        
        return any(service in available_services for service in moveit_services)

def main():
    rclpy.init()
    
    test_node = MoveItTestNode()
    
    # Wait a moment for system to initialize
    time.sleep(2)
    
    # Run tests
    topics_ok = test_node.test_topics()
    controllers_ok = test_node.test_controllers()
    moveit_ok = test_node.test_moveit_services()
    
    # Summary
    test_node.get_logger().info('=== Test Summary ===')
    test_node.get_logger().info(f'Topics: {"PASS" if topics_ok else "FAIL"}')
    test_node.get_logger().info(f'Controllers: {"PASS" if controllers_ok else "FAIL"}')
    test_node.get_logger().info(f'MoveIt: {"PASS" if moveit_ok else "FAIL"}')
    
    if all([topics_ok, controllers_ok, moveit_ok]):
        test_node.get_logger().info('🎉 All tests passed! MoveIt should work properly.')
    else:
        test_node.get_logger().warn('❌ Some tests failed. Check the issues above.')
        
        # Provide troubleshooting suggestions
        test_node.get_logger().info('Troubleshooting suggestions:')
        if not topics_ok:
            test_node.get_logger().info('- Check if robot_state_publisher is running')
        if not controllers_ok:
            test_node.get_logger().info('- Check if ros2_control_node is running')
            test_node.get_logger().info('- Verify hardware connection if using real robot')
        if not moveit_ok:
            test_node.get_logger().info('- Check if move_group node is running')
            test_node.get_logger().info('- Verify MoveIt configuration files')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
