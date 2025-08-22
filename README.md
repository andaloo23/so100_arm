# SO100 Bidirectional - Minimal Calibration Package

This is a minimal ROS2 package containing **only** the essential components needed for robot calibration. No visualization, no MoveIt, no extra features - just hardware interface and calibration.

## What's Included

- **Hardware Interface**: Minimal ROS2 Control interface for SO-100 robot
- **Calibration Script**: Python script for manual joint calibration
- **Essential Configs**: URDF, YAML configs for basic functionality
- **Feetech SDK**: SCServo library for servo communication

## Quick Start

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select so100_bidirectional
source install/setup.bash
```

### 2. Start Hardware Interface
```bash
ros2 launch so100_bidirectional hardware.launch.py
```

### 3. Run Calibration (in another terminal)
```bash
cd ~/ros2_ws
source install/setup.bash
python3 src/brukg_so100/so100_bidirectional/scripts/calibrate_arm.py
```

## Calibration Process

1. **Connect Robot**: Ensure robot is connected via USB (/dev/ttyUSB0)
2. **Start Hardware**: Launch the hardware interface
3. **Run Script**: Execute calibration script
4. **Manual Movement**: Script will disable torque for manual joint movement
5. **Record Positions**: For each joint, move to min/center/max and press Enter
6. **Save Data**: Calibration data automatically saved to `config/calibration.yaml`

## Files Structure

```
so100_bidirectional/
├── config/
│   ├── calibration.yaml          # Calibration data (auto-generated)
│   ├── hardware_config.yaml      # Hardware parameters
│   ├── so100_bidirectional.ros2_control.xacro
│   └── so100_bidirectional.urdf.xacro
├── scripts/
│   └── calibrate_arm.py           # Calibration script
├── launch/
│   └── hardware.launch.py        # Hardware interface launcher
└── src/
    └── so100_bidirectional_interface.cpp  # Hardware interface implementation
```

## Dependencies

- ROS2 (tested on Humble)
- yaml-cpp
- Standard ROS2 Control packages

## Joint Names

The system expects these 6 joints in order:
1. `Shoulder_Rotation`
2. `Shoulder_Pitch` 
3. `Elbow`
4. `Wrist_Pitch`
5. `Wrist_Roll`
6. `Gripper`

Servo IDs are mapped sequentially (1-6).
