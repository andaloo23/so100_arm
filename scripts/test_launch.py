#!/usr/bin/env python3

"""
Test script to verify the fixes for RViz and startup movement issues.
This script provides instructions for testing the launch files.
"""

import subprocess
import sys
import time

def print_header(text):
    print("\n" + "="*60)
    print(text)
    print("="*60)

def print_instructions():
    print_header("SO-100 Launch Test Instructions")
    
    print("\n🔧 FIXES APPLIED:")
    print("1. RViz output changed from 'log' to 'screen' for proper display")
    print("2. Added 'auto_start_controllers' parameter (default: false)")
    print("3. Controllers only start if auto_start_controllers=true")
    print("4. Improved visualizer script robustness")
    
    print("\n📋 TESTING PROCEDURE:")
    print("\n1. Test RViz without robot movement (default behavior):")
    print("   ros2 launch so100_arm rviz_moveit.launch.py")
    print("   Expected: RViz window should appear, robot should NOT move")
    
    print("\n2. Test with robot movement enabled:")
    print("   ros2 launch so100_arm rviz_moveit.launch.py auto_start_controllers:=true")
    print("   Expected: RViz window appears, controllers start, robot may move to hold position")
    
    print("\n3. Test visualization only (no hardware):")
    print("   ros2 launch so100_arm rviz_moveit.launch.py use_hardware:=false")
    print("   Expected: RViz window appears, no hardware interface")
    
    print("\n4. Test with fake hardware:")
    print("   ros2 launch so100_arm rviz_moveit.launch.py use_fake_hardware:=true")
    print("   Expected: RViz window appears, fake hardware simulation")
    
    print("\n✅ VERIFICATION CHECKLIST:")
    print("□ RViz window opens and displays properly")
    print("□ Robot model is visible in RViz")
    print("□ Robot does NOT move on startup (default)")
    print("□ MoveIt planning interface is available")
    print("□ Joint state visualization works")
    
    print("\n🚨 TROUBLESHOOTING:")
    print("- If RViz doesn't open: Check if display is available (DISPLAY env var)")
    print("- If robot still moves: Verify auto_start_controllers=false (default)")
    print("- If no robot model: Check URDF and robot_description topic")
    print("- If service errors: Ensure hardware interface is running")
    
    print("\n📝 USAGE EXAMPLES:")
    print("# Basic usage (safe, no movement):")
    print("ros2 launch so100_arm rviz_moveit.launch.py")
    print()
    print("# Enable controllers (may cause movement):")
    print("ros2 launch so100_arm rviz_moveit.launch.py auto_start_controllers:=true")
    print()
    print("# Visualization only:")
    print("ros2 launch so100_arm rviz_moveit.launch.py use_hardware:=false")

if __name__ == "__main__":
    print_instructions()
    
    print("\n" + "="*60)
    response = input("Would you like to test the launch file now? (y/n): ").strip().lower()
    
    if response == 'y':
        print("\nStarting test launch (safe mode - no robot movement)...")
        print("Press Ctrl+C to stop the launch when you're done testing.")
        time.sleep(2)
        
        try:
            subprocess.run([
                "ros2", "launch", "so100_arm", "rviz_moveit.launch.py"
            ])
        except KeyboardInterrupt:
            print("\nLaunch stopped by user.")
        except Exception as e:
            print(f"\nError running launch: {e}")
    else:
        print("\nTest instructions provided above. Run the commands manually to test.")
