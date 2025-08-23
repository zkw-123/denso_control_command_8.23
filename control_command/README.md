# Control Command Package

A ROS2 package for controlling DENSO robots through simple text commands. This package provides a unified interface for pose control, joint control, and linear insertion operations.

## Overview

The `control_command` package enables intuitive robot control through string-based commands published to a ROS2 topic. It integrates with MoveIt for motion planning and supports both simulation and real robot environments.

## Features

- **Pose Control**: Move robot to specific Cartesian coordinates with fixed orientation
- **Free Movement**: Move robot to specific positions with flexible orientation
- **Joint Control**: Direct joint angle control for precise positioning  
- **Linear Insertion**: Specialized insertion operations along Z-axis with dual strategy support
- **Status Monitoring**: Real-time robot state and position feedback with suggested targets
- **Dual Environment**: Automatic detection and adaptation for simulation vs real robot
- **Advanced Planning**: Separate plan and execute phases with comprehensive error handling
- **Flexible Constraints**: Multiple planning strategies to handle complex workspace limitations

## Package Structure

```
control_command/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── insert_strategy.hpp      # Header for insertion strategy
├── src/
│   ├── robot_controller_node.cpp  # Main control node
│   └── insert_strategy.cpp       # Insertion strategy implementation
└── launch/
    └── robot_control.launch.py   # Launch file for easy startup
```

## Dependencies

- `rclcpp` - ROS2 C++ client library
- `std_msgs` - Standard ROS2 message types
- `geometry_msgs` - Geometry-related message types
- `moveit_ros_planning_interface` - MoveIt planning interface
- `moveit_msgs` - MoveIt message types
- `tf2_geometry_msgs` - Transform geometry messages

## Installation

1. **Clone the package into your ROS2 workspace:**
   ```bash
   cd ~/your_ros2_workspace/src
   git clone <repository_url>
   ```

2. **Install dependencies:**
   ```bash
   cd ~/your_ros2_workspace
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package:**
   ```bash
   colcon build --packages-select control_command
   source install/setup.bash
   ```

## Usage

### Starting the System

#### For Simulation (Gazebo):
```bash
# Terminal 1: Start robot simulation
ros2 launch denso_robot_bringup denso_robot_bringup.launch.py model:=vp6242 sim:=true

# Terminal 2: Start control node
ros2 run control_command robot_control_node
```

#### For Real Robot:
```bash
# Terminal 1: Start real robot
ros2 launch denso_robot_bringup denso_robot_bringup.launch.py model:=vp6242 sim:=false ip_address:=192.168.1.100

# Terminal 2: Start control node  
ros2 run control_command robot_control_node --ros-args -p use_sim_time:=false
```

### Command Interface

Send commands via the `/robot_command` topic:

```bash
ros2 topic pub --once /robot_command std_msgs/msg/String "data: '<command>'"
```

## Supported Commands

### 1. Pose Control (Fixed Orientation)
Move robot end-effector to specific Cartesian coordinates with fixed orientation:
```bash
ros2 topic pub --once /robot_command std_msgs/msg/String "data: 'pose <x> <y> <z>'"
```
**Example:**
```bash
ros2 topic pub --once /robot_command std_msgs/msg/String "data: 'pose 0.3 0.1 0.5'"
```

### 2. Free Movement (Flexible Orientation)
Move robot end-effector to specific position allowing optimal orientation selection:
```bash
ros2 topic pub --once /robot_command std_msgs/msg/String "data: 'move <x> <y> <z>'"
```
**Example:**
```bash
ros2 topic pub --once /robot_command std_msgs/msg/String "data: 'move 0.3 0.1 0.5'"
```

### 3. Joint Control
Control individual joint angles (in radians):
```bash
ros2 topic pub --once /robot_command std_msgs/msg/String "data: 'joints <j1> <j2> <j3> <j4> <j5> <j6>'"
```
**Example:**
```bash
ros2 topic pub --once /robot_command std_msgs/msg/String "data: 'joints 0.0 0.5 -0.3 0.0 0.0 0.0'"
```

### 4. Linear Insertion
Perform linear insertion/extraction along Z-axis with dual strategy support:
```bash
ros2 topic pub --once /robot_command std_msgs/msg/String "data: 'insert <distance>'"
```
**Examples:**
```bash
ros2 topic pub --once /robot_command std_msgs/msg/String "data: 'insert 0.02'"   # Insert 2cm down
ros2 topic pub --once /robot_command std_msgs/msg/String "data: 'insert -0.02'"  # Extract 2cm up
```

### 5. Status Query
Get current robot position and suggested nearby targets:
```bash
ros2 topic pub --once /robot_command std_msgs/msg/String "data: 'current'"
```

## Command Output Example

When you send a command, you'll see detailed feedback:

```
[Command #1] Input received: 'move 0.3 0.1 0.5'
Command parsed successfully, type: 'move'
Starting control execution...
Move command parameters: x=0.300, y=0.100, z=0.500
Setting target position (orientation free)...
Current position: (0.293, -0.006, 1.115)
Target position: (0.300, 0.100, 0.500)
Distance to target: 0.625 m
Planning motion with position-only constraint...
Planning successful! Executing motion...
Position motion completed successfully!
Final position: (0.300, 0.100, 0.500)
Final orientation: (0.123, 0.456, 0.789, 0.321)
Command #1 execution time: 3.45 seconds
========================================
```

## Planning Strategies

The package uses multiple planning strategies to maximize success rate:

1. **Position-only constraints** (`move` command): Allows optimal orientation selection
2. **Full pose constraints** (`pose` command): Fixed orientation with fallback to current orientation
3. **Joint space planning** (`joints` command): Direct joint control bypassing Cartesian planning
4. **Cartesian path planning** (`insert` command): Linear interpolation with dual strategy support

## Advanced Features

### Dual Strategy Insertion
The insertion command uses two strategies:
1. **Primary**: Optimized InsertStrategy with enhanced path planning
2. **Fallback**: Standard Cartesian path planning if primary fails

### Intelligent Error Recovery
- Automatic fallback from strict to relaxed constraints
- Multiple planning attempts with increasing tolerance
- Comprehensive error reporting with actionable suggestions

## Error Handling

The package provides comprehensive error messages and suggestions:

- **Planning failures**: Automatic fallback strategies and alternative constraint methods
- **Unreachable targets**: Provides nearby achievable coordinates with multiple options
- **Joint limits**: Warns about joint constraint violations with suggested alternatives
- **Collision detection**: Reports potential obstacles with recommended solutions
- **Orientation constraints**: Automatic relaxation of pose constraints when needed

## Troubleshooting

### Common Issues:

1. **"Could not find parameter robot_description_semantic"**
   - Ensure MoveIt is running before starting the control node
   - Check that `use_sim_time` parameter matches your environment (true for simulation, false for real robot)
   - Verify all required parameters are loaded using: `ros2 param list | grep robot_description`

2. **"Planning failed" or "Unable to sample any valid states"**
   - Target position may be outside robot workspace - use `current` command for workspace-aware suggestions
   - Try using `move` command instead of `pose` for more flexible planning
   - Consider smaller incremental movements or joint-space commands
   - Check for workspace obstacles or joint limits

3. **"Failed to fetch current robot state"**
   - Check time synchronization between nodes (simulation vs real time)
   - Verify `/joint_states` topic is publishing: `ros2 topic hz /joint_states`
   - Ensure proper parameter copying from move_group node

4. **"Motion execution failed"**
   - Planning succeeded but execution failed - check controller status
   - Verify robot hardware/simulation is responding properly
   - Review joint trajectory controller configuration

### Debug Commands:

```bash
# Check if MoveIt is running
ros2 node list | grep move_group

# Check joint states and frequency
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states

# Verify robot description parameters
ros2 param list | grep robot_description

# Check controller status
ros2 control list_controllers

# Test with safe joint movement first
ros2 topic pub --once /robot_command std_msgs/msg/String "data: 'joints 0.0 0.0 0.0 0.0 0.0 0.0'"
```

## Advanced Configuration

### Custom Planning Parameters

Modify planning settings in `robot_controller_node.cpp`:

```cpp
move_group_->setPlanningTime(20.0);      // Planning timeout (seconds)
move_group_->setNumPlanningAttempts(15); // Number of planning attempts
move_group_->setGoalTolerance(0.01);     // Goal position tolerance (meters)
```

### Adding New Commands

To add new command types:

1. Update the command parsing in `commandCallback()`
2. Implement the new command function
3. Add command description to `printSupportedCommands()`

## API Reference

### Main Classes

- **`RobotController`**: Primary control node class
- **`InsertStrategy`**: Specialized insertion operations

### Key Methods

- `moveToPose(x, y, z)`: Execute Cartesian movement with fixed orientation
- `moveToPosition(x, y, z)`: Execute Cartesian movement with flexible orientation  
- `moveToJoints(joints)`: Execute joint movement
- `executeInsert(distance)`: Perform linear insertion with dual strategy
- `getCurrentStatus()`: Query robot state with suggested targets

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly in both simulation and real robot environments
5. Submit a pull request

## License

This package is licensed under the Apache License 2.0.

## Support

For issues and questions:
- Check the troubleshooting section above
- Review MoveIt documentation for planning issues
- Ensure proper robot calibration and setup

## Version History
v1.6：移除 J6 世界姿态约束；新增 J4 路径约束；逐步放宽；兜底逻辑

v1.5：Plan/Execute 分离与失败兜底

v1.4：插补双策略

v1.3：move 自由姿态

v1.0：初始版本

- **v1.0.0**: Initial release with basic pose, joint, and insertion control
- **v1.1.0**: Added status monitoring and error handling improvements
- **v1.2.0**: Enhanced environment detection and debugging features
- **v1.3.0**: Added flexible orientation planning with `move` command
- **v1.4.0**: Implemented dual-strategy insertion and advanced error recovery
- **v1.5.0**: Enhanced planning with separate plan/execute phases and comprehensive fallback mechanisms
