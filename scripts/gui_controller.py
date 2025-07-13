#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import math
import time

class GuiController(Node):
    def __init__(self):
        super().__init__('gui_controller')
        
        # Control mode: 'stm32' or 'gazebo'
        self.declare_parameter('control_mode', 'stm32')
        self.control_mode = self.get_parameter('control_mode').value
        
        # Connection monitoring
        self.last_joint_state_time = time.time()
        self.connection_timeout = 3.0  # seconds
        self.is_connected = False
        
        # Publishers for different control modes
        if self.control_mode == 'stm32':
            # STM32 hardware mode - using JointJog messages
            self.joint_cmd_pub = self.create_publisher(
                JointJog, 
                '/joint_command', 
                10
            )
        else:  # gazebo mode
            # Gazebo simulation mode - using position controller
            self.position_cmd_pub = self.create_publisher(
                Float64MultiArray,
                '/position_controller/commands',
                10
            )
            
            # Also support trajectory controller for smoother movements
            self.trajectory_cmd_pub = self.create_publisher(
                JointTrajectory,
                '/joint_trajectory_controller/joint_trajectory',
                10
            )
        
        # Subscriber for joint states (common for both modes)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Timer to check connection status
        self.connection_timer = self.create_timer(1.0, self.check_connection)
        
        # Current joint positions
        if self.control_mode == 'stm32':
            # STM32 mode uses degrees
            self.current_positions = [0.0, 0.0, 0.0]
            self.target_positions = [0.0, 0.0, 0.0]
            self.joint_limits = [(0.0, 180.0) for _ in range(3)]
            self.angle_unit = "degrees"
        else:
            # Gazebo mode uses radians
            self.current_positions = [0.0, 0.0, 0.0]
            self.target_positions = [0.0, 0.0, 0.0]
            self.joint_limits = [(0.0, 3.14159) for _ in range(3)]
            self.angle_unit = "radians"
        
        self.get_logger().info(f"STM32 GUI Controller started in {self.control_mode} mode")
        
        # Create GUI
        self.create_gui()
        
    def joint_state_callback(self, msg):
        if len(msg.position) >= 3:
            positions = list(msg.position[:3])
            
            # Convert radians to degrees for STM32 mode display
            if self.control_mode == 'stm32':
                self.current_positions = [math.degrees(pos) for pos in positions]
            else:
                self.current_positions = positions
            
            self.last_joint_state_time = time.time()
            
            # Update connection status if not already connected
            if not self.is_connected:
                self.is_connected = True
                self.root.after(0, self.update_connection_status_connected)
                
            # Update GUI in main thread
            self.root.after(0, self.update_position_display)
    
    def check_connection(self):
        """Check if ROS/Gazebo services are still available"""
        current_time = time.time()
        if self.is_connected and (current_time - self.last_joint_state_time) > self.connection_timeout:
            self.is_connected = False
            self.get_logger().warning("Connection to robot lost")
            self.root.after(0, self.update_connection_status_disconnected)
    
    def update_connection_status_connected(self):
        """Update GUI to show connected status"""
        self.connection_label.config(text="Connected", foreground="green")
        mode_color = "blue" if self.control_mode == 'gazebo' else "darkgreen"
        self.mode_label.config(text=self.control_mode.upper(), foreground=mode_color)
    
    def update_connection_status_disconnected(self):
        """Update GUI to show disconnected status"""
        self.connection_label.config(text="Disconnected", foreground="red")
        self.mode_label.config(text="N/A", foreground="gray")
    
    def create_gui(self):
        """Create the main GUI window"""
        self.root = tk.Tk()
        self.root.title(f"STM32 Controller - {self.control_mode.upper()} Mode")
        self.root.geometry("650x550")
        self.root.configure(bg='#f0f0f0')
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title with mode indicator
        title_text = f"STM32 Controller - {self.control_mode.upper()} Mode"
        title_label = ttk.Label(main_frame, text=title_text, 
                               font=('Arial', 16, 'bold'))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 10))
        
        # Mode info
        mode_info = f"Control Mode: {self.control_mode.upper()} | Units: {self.angle_unit}"
        mode_label = ttk.Label(main_frame, text=mode_info, 
                              font=('Arial', 10), foreground='blue')
        mode_label.grid(row=1, column=0, columnspan=3, pady=(0, 15))
        
        # Joint control section
        self.create_joint_controls(main_frame)
        
        # Status section
        self.create_status_section(main_frame)
        
        # Preset buttons
        self.create_preset_buttons(main_frame)
        
        # Control buttons
        self.create_control_buttons(main_frame)
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
    
    def create_joint_controls(self, parent):
        """Create joint control sliders"""
        # Joint controls frame
        joint_frame = ttk.LabelFrame(parent, text="Joint Controls", padding="10")
        joint_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.joint_vars = []
        self.joint_scales = []
        self.joint_labels = []
        
        for i in range(3):
            # Joint label
            joint_label = ttk.Label(joint_frame, text=f"Joint {i+1}:")
            joint_label.grid(row=i, column=0, sticky=tk.W, padx=(0, 10))
            
            # Slider variable
            var = tk.DoubleVar()
            var.trace('w', lambda *args, idx=i: self.on_slider_change(idx))
            self.joint_vars.append(var)
            
            # Slider with appropriate limits
            min_val, max_val = self.joint_limits[i]
            scale = ttk.Scale(joint_frame, from_=min_val, to=max_val, 
                            variable=var, orient=tk.HORIZONTAL, length=300)
            scale.grid(row=i, column=1, sticky=(tk.W, tk.E), padx=(0, 10))
            self.joint_scales.append(scale)
            
            # Value label
            unit_symbol = "°" if self.control_mode == 'stm32' else "rad"
            value_label = ttk.Label(joint_frame, text=f"0.0{unit_symbol}")
            value_label.grid(row=i, column=2, sticky=tk.W)
            self.joint_labels.append(value_label)
        
        joint_frame.columnconfigure(1, weight=1)
    
    def create_status_section(self, parent):
        """Create status display section"""
        status_frame = ttk.LabelFrame(parent, text="Status", padding="10")
        status_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        unit_symbol = "°" if self.control_mode == 'stm32' else "rad"
        
        # Current positions
        ttk.Label(status_frame, text="Current Positions:").grid(row=0, column=0, sticky=tk.W)
        self.current_pos_label = ttk.Label(status_frame, text=f"[0.0{unit_symbol}, 0.0{unit_symbol}, 0.0{unit_symbol}]", 
                                          font=('Courier', 10))
        self.current_pos_label.grid(row=0, column=1, sticky=tk.W, padx=(10, 0))
        
        # Target positions
        ttk.Label(status_frame, text="Target Positions:").grid(row=1, column=0, sticky=tk.W)
        self.target_pos_label = ttk.Label(status_frame, text=f"[0.0{unit_symbol}, 0.0{unit_symbol}, 0.0{unit_symbol}]", 
                                         font=('Courier', 10))
        self.target_pos_label.grid(row=1, column=1, sticky=tk.W, padx=(10, 0))
        
        # Connection status
        ttk.Label(status_frame, text="Connection:").grid(row=2, column=0, sticky=tk.W)
        self.connection_label = ttk.Label(status_frame, text="Connected", 
                                         foreground="green", font=('Arial', 10, 'bold'))
        self.connection_label.grid(row=2, column=1, sticky=tk.W, padx=(10, 0))
        
        # Control mode
        ttk.Label(status_frame, text="Control Mode:").grid(row=3, column=0, sticky=tk.W)
        mode_color = "blue" if self.control_mode == 'gazebo' else "darkgreen"
        self.mode_label = ttk.Label(status_frame, text=self.control_mode.upper(), 
                                   foreground=mode_color, font=('Arial', 10, 'bold'))
        self.mode_label.grid(row=3, column=1, sticky=tk.W, padx=(10, 0))
    
    def create_preset_buttons(self, parent):
        """Create preset movement buttons"""
        preset_frame = ttk.LabelFrame(parent, text="Preset Movements", padding="10")
        preset_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        if self.control_mode == 'stm32':
            # STM32 mode presets in degrees
            ttk.Button(preset_frame, text="Home (0°)", 
                      command=lambda: self.move_to_preset([0, 0, 0])).grid(row=0, column=0, padx=(0, 5))
            ttk.Button(preset_frame, text="Middle (90°)", 
                      command=lambda: self.move_to_preset([90, 90, 90])).grid(row=0, column=1, padx=5)
            ttk.Button(preset_frame, text="Full (180°)", 
                      command=lambda: self.move_to_preset([180, 180, 180])).grid(row=0, column=2, padx=5)
        else:
            # Gazebo mode presets in radians
            ttk.Button(preset_frame, text="Home (0 rad)", 
                      command=lambda: self.move_to_preset([0, 0, 0])).grid(row=0, column=0, padx=(0, 5))
            ttk.Button(preset_frame, text="Middle (π/2)", 
                      command=lambda: self.move_to_preset([1.57, 1.57, 1.57])).grid(row=0, column=1, padx=5)
            ttk.Button(preset_frame, text="Full (π)", 
                      command=lambda: self.move_to_preset([3.14, 3.14, 3.14])).grid(row=0, column=2, padx=5)
        
        ttk.Button(preset_frame, text="Random", 
                  command=self.random_position).grid(row=0, column=3, padx=(5, 0))
    
    def create_control_buttons(self, parent):
        """Create control buttons"""
        control_frame = ttk.Frame(parent)
        control_frame.grid(row=5, column=0, columnspan=3, pady=(10, 0))
        
        ttk.Button(control_frame, text="Send Command", 
                  command=self.send_current_command).grid(row=0, column=0, padx=(0, 10))
        ttk.Button(control_frame, text="Exit", 
                  command=self.on_exit).grid(row=0, column=1, padx=(10, 0))
    
    def on_slider_change(self, joint_idx):
        """Handle slider value changes"""
        value = self.joint_vars[joint_idx].get()
        unit_symbol = "°" if self.control_mode == 'stm32' else "rad"
        self.joint_labels[joint_idx].config(text=f"{value:.2f}{unit_symbol}")
        
        # Update target positions
        self.target_positions[joint_idx] = value
        self.update_target_display()
    
    def send_current_command(self):
        """Send current slider positions to robot"""
        if self.control_mode == 'stm32':
            # STM32 mode - send JointJog message
            msg = JointJog()
            msg.joint_names = ['joint_1', 'joint_2', 'joint_3']
            msg.displacements = [self.target_positions[0], self.target_positions[1], self.target_positions[2]]
            msg.duration = 0.0
            self.joint_cmd_pub.publish(msg)
            self.get_logger().info(f"Sent STM32 command: {self.target_positions} degrees")
        else:
            # Gazebo mode - send position command
            msg = Float64MultiArray()
            msg.data = [self.target_positions[0], self.target_positions[1], self.target_positions[2]]
            self.position_cmd_pub.publish(msg)
            self.get_logger().info(f"Sent Gazebo command: {self.target_positions} radians")
    
    def smooth_trajectory(self):
        """Send smooth trajectory command (Gazebo only)"""
        if self.control_mode != 'gazebo':
            return
            
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3']
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [self.target_positions[0], self.target_positions[1], self.target_positions[2]]
        point.velocities = [0.0, 0.0, 0.0]
        point.time_from_start.sec = 2  # 2 seconds to reach target
        point.time_from_start.nanosec = 0
        
        msg.points = [point]
        
        self.trajectory_cmd_pub.publish(msg)
        self.get_logger().info(f"Sent smooth trajectory: {self.target_positions} radians")
    
    def move_to_preset(self, values_list):
        """Move to preset position"""
        for i, value in enumerate(values_list):
            self.joint_vars[i].set(value)
        self.send_current_command()
    
    def random_position(self):
        """Move to random position"""
        import random
        for i in range(3):
            min_val, max_val = self.joint_limits[i]
            random_val = random.uniform(min_val, max_val)
            self.joint_vars[i].set(random_val)
        self.send_current_command()
    
    def update_position_display(self):
        """Update current position display"""
        unit_symbol = "°" if self.control_mode == 'stm32' else "rad"
        self.current_pos_label.config(text=f"[{self.current_positions[0]:.2f}{unit_symbol}, {self.current_positions[1]:.2f}{unit_symbol}, {self.current_positions[2]:.2f}{unit_symbol}]")
    
    def update_target_display(self):
        """Update target position display"""
        unit_symbol = "°" if self.control_mode == 'stm32' else "rad"
        self.target_pos_label.config(text=f"[{self.target_positions[0]:.2f}{unit_symbol}, {self.target_positions[1]:.2f}{unit_symbol}, {self.target_positions[2]:.2f}{unit_symbol}]")
    
    def on_exit(self):
        """Handle exit button"""
        self.destroy_node()
        self.root.quit()
    
    def run_gui(self):
        """Run the GUI main loop"""
        self.root.mainloop()

def spin_node(node):
    """Spin ROS node in separate thread"""
    while rclpy.ok():
        try:
            rclpy.spin_once(node, timeout_sec=0.1)
        except:
            break

def main():
    rclpy.init()
    
    try:
        controller = GuiController()
        
        # Start ROS spinning in separate thread
        ros_thread = threading.Thread(target=spin_node, args=(controller,), daemon=True)
        ros_thread.start()
        
        # Run GUI in main thread
        controller.run_gui()
        
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 