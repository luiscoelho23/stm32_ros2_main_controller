# STM32 ROS2 Main Controller

A comprehensive ROS2 control package for STM32 micro-ROS robot with dual-mode support for both hardware control and Gazebo simulation.

## 🚀 Overview

This package provides a unified control interface for STM32-based robots with seamless switching between hardware and simulation modes. It includes a comprehensive launcher GUI for service management and advanced process control capabilities.

### Key Features

- **🎮 Unified Launcher Interface**: Comprehensive GUI for service management and system monitoring
- **🔄 Dual Mode Controller**: Native support for STM32 hardware and Gazebo simulation
- **🛠️ Advanced Process Management**: Intelligent service lifecycle management with proper cleanup
- **📊 Real-time Monitoring**: Live status tracking, logging, and debugging capabilities
- **🎯 URDF Selection**: Dynamic robot model selection for trajectory generation
- **🔧 Enhanced RViz Integration**: Proper visualization with custom configuration

## 📦 Package Structure

```
stm32_ros2_main_controller/
├── scripts/
│   ├── launcher_gui.py            # 🎮 Main launcher interface
│   └── gui_controller.py          # 🎯 Dual-mode GUI controller
├── launch/
│   └── launch.py                  # 🚀 Comprehensive launch configuration
├── config/
│   └── controllers.yaml           # ⚙️ ROS2 controller configuration
├── resources/                     # 📁 Organized resource directory
│   ├── urdf/                      # 🤖 Robot description files
│   │   ├── stm32_robot.urdf.xacro
│   │   └── stm32_robot_gazebo.urdf.xacro
│   └── rviz/                      # 👁️ RViz visualization config
│       └── config.rviz
├── package.xml                    # 📋 Package dependencies
├── CMakeLists.txt                 # 🔨 Build configuration
└── README.md                      # 📖 This file
```

## ✨ Recent Improvements

### 🎯 URDF Selection Feature
- **Dynamic URDF Selection**: Users can now choose different robot models when generating trajectories
- **Backward Compatibility**: Maintains compatibility with existing trajectory scripts
- **Validation**: Automatic URDF file existence validation
- **Integration**: Seamless integration with other packages

### 🛠️ Enhanced Process Management
- **Robust Termination**: Multiple-layer process killing for stubborn GUI components
- **Selective Targeting**: Precise process identification to avoid killing the launcher itself
- **Graceful Shutdown**: Proper SIGTERM before SIGKILL approach
- **Joint State Publisher GUI**: Fixed termination issues with joint_state_publisher_gui

### 📁 Improved Package Organization
- **Resources Directory**: Centralized resource management under `resources/`
- **URDF Organization**: Moved URDF files to `resources/urdf/` for better structure
- **RViz Configuration**: Added proper RViz config file for visualization
- **Build System**: Updated CMakeLists.txt for new directory structure

### 🔧 Package Name Consistency
- **Unified Naming**: Resolved all package name inconsistencies across files
- **Launch Scripts**: Fixed launcher and GUI references
- **URDF References**: Corrected plugin path configurations

## 🚀 Installation

### Prerequisites

```bash
# ROS2 Humble
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-joint-state-publisher-gui \
  python3-tkinter
```

### Build Instructions

1. **Navigate to your workspace**:
   ```bash
   cd ~/your_workspace
   ```

2. **Install dependencies**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:
   ```bash
   colcon build --packages-select stm32_ros2_main_controller
   source install/setup.bash
   ```

## 🎮 Usage

### Quick Start - Launcher GUI

The recommended way to use the system:

```bash
ros2 run stm32_ros2_main_controller launcher_gui.py
```

**Launcher Capabilities:**
- 🟢 **Start Gazebo**: Full simulation with robot model, controllers, and RViz
- 🎮 **Start Control GUI**: Launch controller in STM32 or Gazebo mode
- 📊 **System Status**: Real-time service monitoring and connection status
- 🛑 **Stop All Services**: Comprehensive cleanup of all ROS processes
- 📝 **Log Management**: Live system logs with save/clear functionality

### Individual Components

#### 1. 🏗️ Gazebo Simulation
```bash
ros2 launch stm32_ros2_main_controller launch.py gazebo:=true rviz:=true
```

**Features:**
- Full physics simulation environment
- Robot state visualization in RViz
- Multiple controller support (position, trajectory)
- Joint state broadcasting
- Customizable world files

#### 2. 🎯 GUI Controller - STM32 Mode
```bash
ros2 run stm32_ros2_main_controller gui_controller.py --ros-args -p control_mode:=stm32
```

**STM32 Mode Features:**
- **Units**: Degrees (0-180°)
- **Communication**: JointJog messages to `/joint_command`
- **Presets**: Home (0°), Middle (90°), Full (180°), Random positions
- **Real-time Feedback**: Live position monitoring from hardware

#### 3. 🤖 GUI Controller - Gazebo Mode
```bash
ros2 run stm32_ros2_main_controller gui_controller.py --ros-args -p control_mode:=gazebo
```

**Gazebo Mode Features:**
- **Units**: Radians (0-π)
- **Communication**: Float64MultiArray to `/position_controller/commands`
- **Trajectory Support**: Smooth JointTrajectory movements
- **Physics Integration**: Full collision detection and dynamics

## ⚙️ Controller Configuration

### Available Controllers

```yaml
# config/controllers.yaml
controller_manager:
  ros__parameters:
    joint_state_broadcaster:      # Joint state publishing
      type: joint_state_broadcaster/JointStateBroadcaster
    
    position_controller:          # Direct position control
      type: position_controllers/JointGroupPositionController
    
    joint_trajectory_controller:  # Smooth trajectory execution
      type: joint_trajectory_controller/JointTrajectoryController
```

### Launch Arguments

```bash
# Controller selection
ros2 launch stm32_ros2_main_controller launch.py controller_type:=position_controller

# Service toggling
ros2 launch stm32_ros2_main_controller launch.py rviz:=true gazebo:=true joint_state_publisher:=true

# World customization
ros2 launch stm32_ros2_main_controller launch.py world:=custom_world.xml
```

## 🔗 Integration with Other Packages

### Example: Trajectory Generation with URDF Selection

```bash
# Using with skill acquisition packages
python3 trajectory_mapping.py input.csv output.csv 3 manipulator.urdf
```

### Example: Micro-ROS Integration

```bash
# Start micro-ROS agent for STM32 communication
ros2 run micro_ros_agent udp4 --port 8888

# Then start STM32 mode controller
ros2 run stm32_ros2_main_controller gui_controller.py --ros-args -p control_mode:=stm32
```

## 📊 Monitoring and Debugging

### System Health Checks

```bash
# Check running nodes
ros2 node list | grep -E "(robot|gazebo|controller)"

# Monitor topics
ros2 topic list | grep -E "(joint|position|trajectory|command)"

# Controller status
ros2 control list_controllers

# Joint states
ros2 topic echo /joint_states --once
```

### Debug Commands

```bash
# STM32 mode monitoring
ros2 topic echo /joint_command

# Gazebo mode monitoring
ros2 topic echo /position_controller/commands

# Service debugging
ros2 service list | grep controller_manager
```

## 🛠️ Troubleshooting

### Common Issues and Solutions

#### 🔴 Launcher GUI Issues
- **Services not starting**: Check workspace sourcing and dependencies
- **Process termination problems**: Use enhanced "Stop All Services" with selective targeting
- **Log errors**: Review real-time log output for specific error messages

#### 🔴 RViz Visualization Issues
- **Model not loading**: Verify URDF files in `resources/urdf/`
- **Config not found**: Ensure `resources/rviz/config.rviz` exists
- **TF errors**: Check robot_state_publisher and joint_state_broadcaster

#### 🔴 Controller Connection Issues
- **STM32 "Disconnected"**: Verify micro-ROS agent and STM32 hardware
- **Gazebo "Disconnected"**: Ensure Gazebo simulation is running
- **Joint states missing**: Check `/joint_states` topic publication

#### 🔴 URDF Selection Issues
- **File not found**: Verify URDF exists in the target package
- **Validation errors**: Check URDF syntax and package references
- **Trajectory generation fails**: Ensure backward compatibility with script arguments

### Advanced Debugging

```bash
# Process management debugging
ps aux | grep -E "(ros2|gazebo|rviz)" | head -20

# Service dependency checking
ros2 pkg executables stm32_ros2_main_controller

# Launch file validation
ros2 launch stm32_ros2_main_controller launch.py --show-args

# Topic bandwidth monitoring
ros2 topic bw /joint_states
```

## 🧪 Development and Extension

### Package Dependencies

```xml
<!-- Core ROS2 -->
<depend>rclcpp</depend>
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>control_msgs</depend>
<depend>trajectory_msgs</depend>

<!-- ROS2 Control -->
<depend>controller_manager</depend>
<depend>ros2_control</depend>
<depend>ros2_controllers</depend>

<!-- Simulation -->
<depend>gazebo-ros2-control</depend>
<depend>urdf</depend>

<!-- GUI -->
<depend>python3-tkinter</depend>
```

### Adding New Features

1. **🎮 New Control Modes**: Extend `gui_controller.py` with additional robot interfaces
2. **⚙️ Custom Controllers**: Add configurations to `controllers.yaml`
3. **🎯 Enhanced GUIs**: Modify launcher or controller interfaces
4. **🤖 Robot Models**: Add new URDF files to `resources/urdf/`
5. **🌍 Simulation Worlds**: Create custom Gazebo environments

### Contributing Guidelines

1. **Code Style**: Follow ROS2 Python style guidelines
2. **Documentation**: Update README.md for new features
3. **Testing**: Verify compatibility with both STM32 and Gazebo modes
4. **Integration**: Ensure compatibility with other packages

## 📄 License

This project is licensed under the **MIT License**.

## 🆘 Support

### Getting Help

- **Issues**: Create GitHub issues for bugs and feature requests
- **Documentation**: Refer to ROS2 documentation for additional guidance
- **Integration**: Check package compatibility when integrating with other systems

### Useful Resources

- [ROS2 Control Documentation](https://control.ros.org/)
- [Gazebo ROS2 Integration](https://gazebosim.org/docs)
- [Micro-ROS Documentation](https://micro.ros.org/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)

---

**🤖 Happy Robot Controlling! 🚀** 