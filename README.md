# STM32 ROS2 Main Controller

A ROS2 control package for STM32 micro-ROS robot with dual-mode support for both hardware control and Gazebo simulation.

## Overview

This package provides a unified control interface for your STM32 robot with seamless switching between hardware and simulation modes:
- **Launcher GUI**: Unified interface to start/stop all services
- **Dual Mode Controller**: GUI controller supporting both STM32 hardware and Gazebo simulation
- **Gazebo Integration**: Full simulation support with proper controller configuration
- **Process Management**: Intelligent service management with proper cleanup

## Features

### 🚀 Unified Launcher Interface
- **Service Management**: Start/stop Gazebo simulation and control GUI
- **Real-time Status**: Live monitoring of running services
- **Process Cleanup**: Proper termination of all ROS processes
- **Log Management**: Real-time log output with save/clear functionality

### 🎮 Dual Mode Control
- **STM32 Hardware Mode**: Direct communication with STM32 microcontroller (degrees)
- **Gazebo Simulation Mode**: Physics simulation with visual feedback (radians)
- **Automatic Unit Conversion**: Proper unit handling for each mode
- **Mode-Specific Presets**: Tailored preset movements for each mode
- **Connection Monitoring**: Real-time connection status detection

### 🔧 Control Features
- **Joint Position Sliders**: Precise control of all joints with real-time feedback
- **Preset Movements**: Home, Middle, Full extension, Random positions
- **Real-time Feedback**: Live position monitoring and status updates
- **Safety Features**: Joint limits and position validation

## Package Structure

```
stm32_ros2_main_controller/
├── scripts/
│   ├── launcher_gui.py                    # Main launcher interface
│   └── gui_controller.py                  # Dual-mode GUI controller
├── launch/
│   └── launch.py                          # Launch file with Gazebo support
├── config/
│   └── controllers.yaml                   # ROS2 controller configuration
├── urdf/
│   ├── stm32_robot.urdf.xacro            # Robot description
│   └── stm32_robot_gazebo.urdf.xacro     # Gazebo-specific robot description
├── package.xml                            # Package dependencies
├── CMakeLists.txt                         # Build configuration
└── README.md                              # This file
```

## Installation

1. **Clone the package** into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <your-repo-url>/stm32_ros2_main_controller
   ```

2. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:
   ```bash
   colcon build --packages-select stm32_ros2_main_controller
   source install/setup.bash
   ```

## Usage

### Quick Start - Launcher GUI

The main way to use the system is through the unified launcher:

```bash
ros2 run stm32_ros2_main_controller launcher_gui.py
```

**Launcher Features:**
- **Start Gazebo**: Launch Gazebo simulation with robot model and RViz
- **Start Control GUI**: Launch the control interface in STM32 or Gazebo mode
- **System Status**: Real-time monitoring of all services
- **Stop All Services**: One-click cleanup of all processes
- **Log Output**: Live system logs with save/clear functionality

### Individual Components

#### 1. Gazebo Simulation
Launch Gazebo with robot model and controllers:
```bash
ros2 launch stm32_ros2_main_controller launch.py gazebo:=true rviz:=true
```

#### 2. GUI Controller - STM32 Mode
Direct hardware control:
```bash
ros2 run stm32_ros2_main_controller gui_controller.py --ros-args -p control_mode:=stm32
```

#### 3. GUI Controller - Gazebo Mode
Simulation control:
```bash
ros2 run stm32_ros2_main_controller gui_controller.py --ros-args -p control_mode:=gazebo
```

## GUI Controller Features

### Control Interface
- **Joint Position Sliders**: Precise control of all joints with real-time feedback
- **Status Display**: Current positions, target positions, connection status, and control mode
- **Preset Movements**: Quick access to common positions
- **Send Command**: Execute current slider positions
- **Mode Indicator**: Clear indication of STM32 vs Gazebo mode

### STM32 Mode (Hardware)
- **Units**: Degrees (0-180°)
- **Communication**: JointJog messages to `/joint_command`
- **Presets**: Home (0°), Middle (90°), Full (180°), Random
- **Connection**: Direct STM32 microcontroller communication

### Gazebo Mode (Simulation)
- **Units**: Radians (0-π)
- **Communication**: Float64MultiArray to `/position_controller/commands`
- **Trajectory Support**: JointTrajectory messages for smooth movements
- **Presets**: Home (0 rad), Middle (π/2), Full (π), Random
- **Physics**: Full physics simulation with collision detection

## Integration Modes

### STM32 Hardware Integration

**Required Topics:**
- **Subscribe**: `/joint_states` (sensor_msgs/JointState)
- **Publish**: `/joint_command` (control_msgs/JointJog)

**Launch Sequence:**
```bash
# Terminal 1: Micro-ROS Agent
ros2 run micro_ros_agent udp4 --port 8888

# Terminal 2: GUI Controller (STM32 Mode)
ros2 run stm32_ros2_main_controller gui_controller.py --ros-args -p control_mode:=stm32
```

### Gazebo Simulation Integration

**Required Topics:**
- **Subscribe**: `/joint_states` (sensor_msgs/JointState)
- **Publish**: `/position_controller/commands` (std_msgs/Float64MultiArray)
- **Action**: `/joint_trajectory_controller/joint_trajectory` (trajectory_msgs/JointTrajectory)

**Launch Sequence:**
```bash
# Option 1: Use Launcher GUI
ros2 run stm32_ros2_main_controller launcher_gui.py

# Option 2: Manual Launch
# Terminal 1: Gazebo Simulation
ros2 launch stm32_ros2_main_controller launch.py gazebo:=true rviz:=true

# Terminal 2: GUI Controller (Gazebo Mode)
ros2 run stm32_ros2_main_controller gui_controller.py --ros-args -p control_mode:=gazebo
```

## Controller Configuration

The system uses ROS2 controller configuration in `config/controllers.yaml`:

### Available Controllers
- **joint_state_broadcaster**: Publishes joint states
- **position_controller**: Direct position control
- **joint_trajectory_controller**: Trajectory following with tolerances and constraints

## Monitoring and Debugging

**Monitor joint states:**
```bash
ros2 topic echo /joint_states
```

**Check controller status:**
```bash
ros2 control list_controllers
```

**Monitor commands:**
```bash
# STM32 mode
ros2 topic echo /joint_command

# Gazebo mode
ros2 topic echo /position_controller/commands
```

## Troubleshooting

### Common Issues

1. **Launcher GUI not starting services**:
   - Check if ROS2 workspace is sourced
   - Verify all dependencies are installed
   - Check log output for specific errors

2. **GUI Controller shows "Disconnected"**:
   - Ensure appropriate services are running (STM32 or Gazebo)
   - Check `/joint_states` topic is publishing
   - Verify controller configuration

3. **Gazebo simulation not loading**:
   - Check URDF/xacro files are valid
   - Verify Gazebo plugins are installed
   - Check controller configuration in `controllers.yaml`

4. **STM32 mode not responding**:
   - Verify STM32 robot is connected
   - Check micro-ROS agent is running
   - Ensure STM32 control system is active

5. **Processes not stopping properly**:
   - Use "Stop All Services" button in launcher
   - Check for zombie processes: `ps aux | grep ros2`
   - Force kill if needed: `pkill -f ros2`

### Debug Commands

```bash
# Check running nodes
ros2 node list | grep -E "(robot|gazebo|controller)"

# Check topics
ros2 topic list | grep -E "(joint|position|trajectory|command)"

# Check controller manager
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers

# Check Gazebo model
ros2 service call /gazebo/get_model_list gazebo_msgs/srv/GetModelList
```

## Development

### Package Dependencies
- **Core ROS2**: rclcpp, rclpy, std_msgs, sensor_msgs, control_msgs, trajectory_msgs
- **ROS2 Control**: controller_manager, ros2_control, ros2_controllers
- **Gazebo**: gazebo-ros2-control
- **GUI**: python3-tkinter

### Adding New Features
1. **New Control Modes**: Extend `gui_controller.py` with additional modes
2. **Custom Controllers**: Add new controller configurations to `controllers.yaml`
3. **Enhanced GUI**: Modify launcher or controller GUIs for new functionality
4. **Simulation Features**: Extend URDF/xacro files for additional robot features

## License

This project is licensed under the MIT License.

## Support

For issues and questions:
- Create an issue in the project repository
- Check the troubleshooting section above
- Verify integration with STM32 control system or Gazebo simulation

---

**Happy Robot Controlling! 🤖** 