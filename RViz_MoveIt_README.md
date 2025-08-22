# RViz MoveIt Integration for SO100 Bidirectional

This package now includes RViz visualization with MoveIt integration for the SO100 bidirectional robot arm.

## Features

- **Real-time Visualization**: Displays the current arm position in RViz
- **MoveIt Integration**: Full motion planning capabilities
- **Calibration-based Conversion**: Uses calibration.yaml to convert servo ticks to joint angles
- **Interactive Motion Planning**: Plan and visualize trajectories using MoveIt
- **Live Position Capture**: Continuously captures and displays the actual arm position

## Quick Start

### 1. Build the Package

```bash
cd /home/andrew/ros2_ws
colcon build --packages-select so100_bidirectional
source install/setup.bash
```

### 2. Launch RViz with MoveIt (Complete System)

**For hardware use (recommended):**
```bash
ros2 launch so100_bidirectional rviz_moveit.launch.py
```

**For simulation/testing only:**
```bash
ros2 launch so100_bidirectional rviz_moveit.launch.py use_hardware:=false use_fake_hardware:=true
```

**To customize serial port:**
```bash
ros2 launch so100_bidirectional rviz_moveit.launch.py serial_port:=/dev/ttyUSB0
```

**To run without RViz (headless):**
```bash
ros2 launch so100_bidirectional rviz_moveit.launch.py use_rviz:=false
```

### 3. Use the Visualizer

After launching, you'll see:
- RViz window with the robot model
- MoveIt Motion Planning panel
- Real-time position updates from the physical arm

In the terminal where you launched, you can:
- Press `d` + Enter to demonstrate sample movements
- Press `q` + Enter to quit

## Components

### Files Created/Modified

1. **scripts/rviz_moveit_visualizer.py** - Main visualization script
2. **launch/rviz_moveit.launch.py** - **INTEGRATED** launch file (replaces hardware.launch.py)
3. **config/so100_bidirectional.srdf** - MoveIt semantic description
4. **config/kinematics.yaml** - Kinematics solver configuration
5. **config/joint_limits.yaml** - Joint limits for motion planning
6. **config/moveit_controllers.yaml** - MoveIt controller configuration
7. **config/ros2_controllers.yaml** - ROS2 controller configuration
8. **config/moveit.rviz** - RViz configuration file

### Integration Notes

- **No need for separate hardware.launch.py** - Everything is now integrated into rviz_moveit.launch.py
- **Conditional hardware loading** - Can run with or without physical hardware
- **Parameter-driven configuration** - Easy to switch between hardware/simulation modes

### How It Works

1. **Position Capture**: The script calls the `record_position` service to get current servo positions in ticks
2. **Calibration Conversion**: Uses calibration.yaml to convert ticks to radians using the formula:
   ```
   normalized = (ticks - min_ticks) / range_ticks
   radians = (normalized * 2.0 - 1.0) * π
   ```
3. **Joint State Publishing**: Publishes converted positions as ROS joint states
4. **Visualization**: RViz displays the robot model with real-time position updates

### MoveIt Planning Groups

- **arm**: All arm joints (Shoulder_Rotation through Wrist_Roll)
- **gripper**: Just the gripper joint

### Predefined Poses

- **home**: All joints at 0.0
- **extended**: Arm extended forward
- **open/closed**: Gripper states

## Usage Tips

1. **Ensure Hardware Connection**: Make sure the physical arm is connected and the hardware interface is running
2. **Check Calibration**: Verify that calibration.yaml contains valid data for all joints
3. **Motion Planning**: Use the MoveIt panel in RViz to plan and execute motions
4. **Real-time Updates**: The display updates at 10Hz showing the actual arm position

## Troubleshooting

- **No Position Updates**: Check if the `record_position` service is available
- **Incorrect Positions**: Verify calibration.yaml has correct min/max/center values
- **MoveIt Errors**: Ensure all MoveIt dependencies are installed
- **RViz Issues**: Check that the robot description is properly loaded

## Dependencies

The package now includes all necessary MoveIt and visualization dependencies. Run:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

To install any missing dependencies.
