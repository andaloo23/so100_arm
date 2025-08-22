#!/usr/bin/env python3

import sys
import os
import time
import serial
import yaml

# Add the SCServo_Linux path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'SCServo_Linux'))

try:
    from SMSBL import *
except ImportError as e:
    print(f"Error importing SMSBL: {e}")
    print("Make sure the SCServo_Linux library is properly compiled")
    sys.exit(1)

def test_servo_communication():
    """Test communication with each servo individually"""
    
    # Configuration
    port = '/dev/ttyACM0'
    baudrate = 1000000
    servo_ids = [1, 2, 3, 4, 5, 6]  # Expected servo IDs
    
    print(f"Testing servo communication on {port} at {baudrate} baud")
    print("=" * 60)
    
    # Initialize serial connection
    try:
        st3215 = SMSBL(port, baudrate)
        time.sleep(0.1)  # Allow connection to stabilize
        print(f"✓ Serial connection established")
    except Exception as e:
        print(f"✗ Failed to establish serial connection: {e}")
        return False
    
    working_servos = []
    failed_servos = []
    
    # Test each servo
    for servo_id in servo_ids:
        print(f"\nTesting Servo {servo_id}:")
        print("-" * 30)
        
        success = False
        for attempt in range(3):
            try:
                # Ping test
                result = st3215.Ping(servo_id)
                if result != -1:
                    print(f"  ✓ Ping successful (attempt {attempt + 1})")
                    success = True
                    break
                else:
                    print(f"  ✗ Ping failed (attempt {attempt + 1})")
                    time.sleep(0.1)
            except Exception as e:
                print(f"  ✗ Exception during ping (attempt {attempt + 1}): {e}")
                time.sleep(0.1)
        
        if success:
            working_servos.append(servo_id)
            
            # Try to read additional info
            try:
                # Read current position
                if st3215.FeedBack(servo_id) != -1:
                    pos = st3215.ReadPos(servo_id)
                    speed = st3215.ReadSpeed(servo_id)
                    load = st3215.ReadLoad(servo_id)
                    print(f"  ✓ Position: {pos} ticks")
                    print(f"  ✓ Speed: {speed}")
                    print(f"  ✓ Load: {load}")
                else:
                    print(f"  ⚠ Could not read servo feedback")
                    
            except Exception as e:
                print(f"  ⚠ Error reading servo details: {e}")
        else:
            failed_servos.append(servo_id)
            print(f"  ✗ Servo {servo_id} not responding")
    
    # Summary
    print("\n" + "=" * 60)
    print("SERVO COMMUNICATION SUMMARY")
    print("=" * 60)
    print(f"Working servos: {working_servos}")
    print(f"Failed servos: {failed_servos}")
    print(f"Success rate: {len(working_servos)}/{len(servo_ids)} ({len(working_servos)/len(servo_ids)*100:.1f}%)")
    
    if failed_servos:
        print("\nTROUBLESHOoting SUGGESTIONS:")
        print("- Check power supply to the robot")
        print("- Verify all servo connections are secure")
        print("- Check if failed servos have correct IDs (use servo configuration tool)")
        print("- Verify baudrate settings match servo configuration")
        print("- Test with lower baudrate (e.g., 115200)")
        print("- Check for loose or damaged cables")
        print("- Ensure servos are properly daisy-chained")
    
    return len(failed_servos) == 0

def test_different_baudrates():
    """Test communication at different baudrates"""
    
    port = '/dev/ttyACM0'
    baudrates = [1000000, 500000, 115200, 57600]
    servo_id = 1  # Test with servo 1 (known to work)
    
    print("\nTesting different baudrates:")
    print("=" * 40)
    
    for baudrate in baudrates:
        print(f"\nTesting {baudrate} baud...")
        try:
            st3215 = SMSBL(port, baudrate)
            time.sleep(0.2)
            
            result = st3215.Ping(servo_id)
            if result != -1:
                print(f"  ✓ Servo {servo_id} responds at {baudrate} baud")
            else:
                print(f"  ✗ No response at {baudrate} baud")
                
        except Exception as e:
            print(f"  ✗ Error at {baudrate} baud: {e}")

def check_serial_permissions():
    """Check if we have proper permissions for the serial port"""
    
    port = '/dev/ttyACM0'
    
    print("\nChecking serial port permissions:")
    print("=" * 40)
    
    if not os.path.exists(port):
        print(f"✗ Serial port {port} does not exist")
        print("  - Check if robot is connected via USB")
        print("  - Try 'ls /dev/ttyACM*' or 'ls /dev/ttyUSB*' to find correct port")
        return False
    
    # Check permissions
    try:
        with open(port, 'rb') as f:
            print(f"✓ Can read from {port}")
    except PermissionError:
        print(f"✗ Permission denied for {port}")
        print(f"  - Run: sudo chmod 666 {port}")
        print(f"  - Or add user to dialout group: sudo usermod -a -G dialout $USER")
        return False
    except Exception as e:
        print(f"⚠ Error accessing {port}: {e}")
        return False
    
    return True

if __name__ == "__main__":
    print("SO-100 Servo Communication Diagnostic Tool")
    print("=" * 60)
    
    # Check prerequisites
    if not check_serial_permissions():
        sys.exit(1)
    
    # Test servo communication
    success = test_servo_communication()
    
    # Test different baudrates if there are issues
    if not success:
        test_different_baudrates()
    
    print("\nDiagnostic complete.")
