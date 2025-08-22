#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStateMonitor(Node):
    def __init__(self):
        super().__init__('joint_state_monitor')
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info("Joint State Monitor started - listening for /joint_states")
        self.last_positions = None
        
    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        if len(msg.position) > 0:
            # Check if positions have changed significantly
            if self.last_positions is None or self.positions_changed(msg.position):
                self.get_logger().info(f"Joint States Update:")
                for i, (name, pos) in enumerate(zip(msg.name, msg.position)):
                    degrees = math.degrees(pos)
                    self.get_logger().info(f"  {name}: {pos:.3f} rad ({degrees:.1f}°)")
                self.last_positions = list(msg.position)
                self.get_logger().info("---")
        else:
            self.get_logger().warn("Received empty joint state message")
    
    def positions_changed(self, new_positions):
        """Check if positions have changed significantly"""
        if len(new_positions) != len(self.last_positions):
            return True
        
        for i, (new_pos, old_pos) in enumerate(zip(new_positions, self.last_positions)):
            if abs(new_pos - old_pos) > 0.01:  # 0.01 radian threshold
                return True
        return False

def main():
    rclpy.init()
    
    monitor = JointStateMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info("Shutting down joint state monitor")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
