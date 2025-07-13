#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import subprocess
import threading
import os
import signal
import sys
import time

class RobotLauncher:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("STM32 Robot Controller Launcher")
        self.root.geometry("800x600")
        self.root.configure(bg='#f0f0f0')
        
        # Process tracking
        self.processes = {}
        self.running_services = set()
        
        # Setup GUI
        self.setup_gui()
        
        # Setup cleanup on exit
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def setup_gui(self):
        """Setup the main GUI interface"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = ttk.Label(main_frame, text="STM32 Robot Controller Launcher", 
                               font=('Arial', 18, 'bold'))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # Status section
        self.create_status_section(main_frame)
        
        # Control buttons section
        self.create_control_section(main_frame)
        
        # Log output section
        self.create_log_section(main_frame)
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(3, weight=1)
        
    def create_status_section(self, parent):
        """Create status indicators"""
        status_frame = ttk.LabelFrame(parent, text="System Status", padding="10")
        status_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Gazebo status
        ttk.Label(status_frame, text="Gazebo:").grid(row=0, column=0, sticky=tk.W, padx=(0, 10))
        self.gazebo_status = ttk.Label(status_frame, text="STOPPED", 
                                      foreground="red", font=('Arial', 10, 'bold'))
        self.gazebo_status.grid(row=0, column=1, sticky=tk.W, padx=(0, 20))
        
        # GUI status
        ttk.Label(status_frame, text="Control GUI:").grid(row=0, column=2, sticky=tk.W, padx=(0, 10))
        self.gui_status = ttk.Label(status_frame, text="STOPPED", 
                                   foreground="red", font=('Arial', 10, 'bold'))
        self.gui_status.grid(row=0, column=3, sticky=tk.W)
        
    def create_control_section(self, parent):
        """Create individual control buttons"""
        control_frame = ttk.LabelFrame(parent, text="Individual Controls", padding="10")
        control_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Gazebo controls
        ttk.Label(control_frame, text="Gazebo Simulation:", font=('Arial', 12, 'bold')).grid(
            row=0, column=0, columnspan=3, sticky=tk.W, pady=(0, 5))
        
        self.gazebo_start_btn = ttk.Button(control_frame, text="Start Gazebo", 
                                          command=self.start_gazebo)
        self.gazebo_start_btn.grid(row=1, column=0, padx=(0, 5), pady=(0, 10))
        
        self.gazebo_stop_btn = ttk.Button(control_frame, text="Stop Gazebo", 
                                         command=self.stop_gazebo, state='disabled')
        self.gazebo_stop_btn.grid(row=1, column=1, padx=5, pady=(0, 10))
        
        # GUI controls
        ttk.Label(control_frame, text="Control GUI:", font=('Arial', 12, 'bold')).grid(
            row=2, column=0, columnspan=3, sticky=tk.W, pady=(5, 5))
        
        # Control mode selection
        ttk.Label(control_frame, text="Mode:").grid(row=3, column=0, sticky=tk.W, padx=(0, 5))
        self.mode_var = tk.StringVar(value="gazebo")
        mode_combo = ttk.Combobox(control_frame, textvariable=self.mode_var, 
                                 values=["gazebo", "stm32"], state="readonly", width=10)
        mode_combo.grid(row=3, column=1, sticky=tk.W, padx=(0, 10))
        
        self.gui_start_btn = ttk.Button(control_frame, text="Start Control GUI", 
                                       command=self.start_control_gui)
        self.gui_start_btn.grid(row=4, column=0, padx=(0, 5), pady=(5, 0))
        
        self.gui_stop_btn = ttk.Button(control_frame, text="Stop Control GUI", 
                                      command=self.stop_control_gui, state='disabled')
        self.gui_stop_btn.grid(row=4, column=1, padx=5, pady=(5, 0))
        
        # Stop all button
        ttk.Button(control_frame, text="Stop All Services", 
                  command=self.stop_all_services).grid(row=5, column=0, columnspan=2, pady=(15, 0))
        
    def create_log_section(self, parent):
        """Create log output section"""
        log_frame = ttk.LabelFrame(parent, text="System Log", padding="10")
        log_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        
        # Log text area
        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, width=80)
        self.log_text.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Log control buttons
        log_controls = ttk.Frame(log_frame)
        log_controls.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(5, 0))
        
        ttk.Button(log_controls, text="Clear Log", 
                  command=self.clear_log).grid(row=0, column=0, padx=(0, 5))
        
        ttk.Button(log_controls, text="Save Log", 
                  command=self.save_log).grid(row=0, column=1, padx=5)
        
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
    def log_message(self, message):
        """Add message to log"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.root.update_idletasks()
        
    def run_command_async(self, command, service_name, callback=None):
        """Run command asynchronously"""
        def run():
            try:
                self.log_message(f"Starting {service_name}...")
                self.log_message(f"Command: {command}")
                
                process = subprocess.Popen(
                    command,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    universal_newlines=True,
                    preexec_fn=os.setsid
                )
                
                self.processes[service_name] = process
                
                # Read output
                for line in iter(process.stdout.readline, ''):
                    if line.strip():
                        self.log_message(f"{service_name}: {line.strip()}")
                
                process.wait()
                
                if process.returncode == 0:
                    self.log_message(f"{service_name} completed successfully")
                else:
                    self.log_message(f"{service_name} exited with code {process.returncode}")
                    
            except Exception as e:
                self.log_message(f"Error running {service_name}: {str(e)}")
            finally:
                if service_name in self.processes:
                    del self.processes[service_name]
                if callback:
                    callback()
                    
        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        
    def start_gazebo(self):
        """Start Gazebo simulation"""
        if 'gazebo' in self.running_services:
            self.log_message("Gazebo is already running")
            return
            
        command = "ros2 launch stm32_robot_controller launch.py gazebo:=true rviz:=true"
        
        def on_complete():
            self.running_services.discard('gazebo')
            self.gazebo_status.config(text="STOPPED", foreground="red")
            self.gazebo_start_btn.config(state='normal')
            self.gazebo_stop_btn.config(state='disabled')
            
        self.running_services.add('gazebo')
        self.gazebo_status.config(text="RUNNING", foreground="green")
        self.gazebo_start_btn.config(state='disabled')
        self.gazebo_stop_btn.config(state='normal')
        
        self.run_command_async(command, 'gazebo', on_complete)
        
    def stop_gazebo(self):
        """Stop Gazebo simulation and related services"""
        self.log_message("Stopping Gazebo simulation...")
        
        # Stop our tracked Gazebo process
        if 'gazebo' in self.processes:
            try:
                os.killpg(os.getpgid(self.processes['gazebo'].pid), signal.SIGTERM)
                self.log_message("Stopping tracked Gazebo process...")
            except:
                pass
        
        # Wait a moment for graceful shutdown
        time.sleep(1)
        
        # Kill specific Gazebo-related processes
        gazebo_processes = [
            'gazebo',
            'gzserver', 
            'gzclient',
            'rviz2',
            'joint_state_publisher',
            'joint_state_publisher_gui',
            'robot_state_publisher'
        ]
        
        for process_name in gazebo_processes:
            try:
                result = subprocess.run(['pkill', '-f', process_name], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    self.log_message(f"Killed {process_name} processes")
            except:
                pass
        
        # Force kill stubborn Gazebo processes
        try:
            subprocess.run(['pkill', '-9', '-f', 'gazebo'], capture_output=True)
            subprocess.run(['pkill', '-9', '-f', 'joint_state'], capture_output=True)
            self.log_message("Force killed stubborn Gazebo processes")
        except:
            pass
        
        # Update UI
        self.running_services.discard('gazebo')
        self.gazebo_status.config(text="STOPPED", foreground="red")
        self.gazebo_start_btn.config(state='normal')
        self.gazebo_stop_btn.config(state='disabled')
        
        self.log_message("Gazebo simulation stopped")
                
    def start_control_gui(self):
        """Start control GUI"""
        if 'control_gui' in self.running_services:
            self.log_message("Control GUI is already running")
            return
            
        mode = self.mode_var.get()
        command = f"ros2 run stm32_robot_controller gui_controller.py --ros-args -p control_mode:={mode}"
        
        def on_complete():
            self.running_services.discard('control_gui')
            self.gui_status.config(text="STOPPED", foreground="red")
            self.gui_start_btn.config(state='normal')
            self.gui_stop_btn.config(state='disabled')
            
        self.running_services.add('control_gui')
        self.gui_status.config(text="RUNNING", foreground="green")
        self.gui_start_btn.config(state='disabled')
        self.gui_stop_btn.config(state='normal')
        
        self.run_command_async(command, 'control_gui', on_complete)
        
    def stop_control_gui(self):
        """Stop control GUI"""
        self.log_message("Stopping control GUI...")
        
        if 'control_gui' in self.processes:
            try:
                os.killpg(os.getpgid(self.processes['control_gui'].pid), signal.SIGTERM)
                self.log_message("Stopping tracked control GUI process...")
            except:
                pass
        
        # Also kill any gui_controller.py processes
        try:
            subprocess.run(['pkill', '-f', 'gui_controller.py'], capture_output=True)
            self.log_message("Killed gui_controller.py processes")
        except:
            pass
        
        # Update UI
        self.running_services.discard('control_gui')
        self.gui_status.config(text="STOPPED", foreground="red")
        self.gui_start_btn.config(state='normal')
        self.gui_stop_btn.config(state='disabled')
        
        self.log_message("Control GUI stopped")
                
    def stop_all_services(self):
        """Stop all running services and related ROS processes"""
        self.log_message("Stopping all services...")
        
        # First stop our tracked processes
        services_to_stop = list(self.processes.keys())
        for service in services_to_stop:
            if service in self.processes:
                try:
                    os.killpg(os.getpgid(self.processes[service].pid), signal.SIGTERM)
                    self.log_message(f"Stopping {service}...")
                except:
                    pass
        
        # Wait a moment for graceful shutdown
        time.sleep(2)
        
        # Kill any remaining ROS processes that might be hanging around
        ros_processes = [
            'joint_state_publisher',
            'joint_state_publisher_gui',
            'robot_state_publisher', 
            'controller_manager',
            'gazebo',
            'gzserver',
            'gzclient',
            'rviz2',
            'ros2',
            'gui_controller.py',
            'python3.*joint_state_publisher',
            'python3.*robot_state_publisher'
        ]
        
        for process_name in ros_processes:
            try:
                # Use pkill to kill processes by name
                result = subprocess.run(['pkill', '-f', process_name], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    self.log_message(f"Killed remaining {process_name} processes")
            except:
                pass
        
        # Also kill any Python processes that might be ROS-related
        try:
            subprocess.run(['pkill', '-f', 'python3.*ros2'], capture_output=True)
            subprocess.run(['pkill', '-f', 'python.*joint_state'], capture_output=True)
            self.log_message("Killed Python ROS processes")
        except:
            pass
        
        # Force kill any stubborn processes
        try:
            subprocess.run(['pkill', '-9', '-f', 'ros2'], capture_output=True)
            subprocess.run(['pkill', '-9', '-f', 'gazebo'], capture_output=True)
            subprocess.run(['pkill', '-9', '-f', 'joint_state'], capture_output=True)
            self.log_message("Force killed any remaining ROS processes")
        except:
            pass
                    
        # Reset all status indicators
        self.gazebo_status.config(text="STOPPED", foreground="red")
        self.gui_status.config(text="STOPPED", foreground="red")
        
        # Reset all buttons
        self.gazebo_start_btn.config(state='normal')
        self.gazebo_stop_btn.config(state='disabled')
        self.gui_start_btn.config(state='normal')
        self.gui_stop_btn.config(state='disabled')
        
        self.running_services.clear()
        self.log_message("All services stopped")
        
    def clear_log(self):
        """Clear log output"""
        self.log_text.delete(1.0, tk.END)
        
    def save_log(self):
        """Save log to file"""
        try:
            with open("robot_launcher.log", "w") as f:
                f.write(self.log_text.get(1.0, tk.END))
            self.log_message("Log saved to robot_launcher.log")
        except Exception as e:
            self.log_message(f"Error saving log: {str(e)}")
            
    def on_closing(self):
        """Handle window closing"""
        if self.processes:
            if messagebox.askokcancel("Quit", "Stop all running services and quit?"):
                self.stop_all_services()
                time.sleep(1)  # Give processes time to stop
                self.root.destroy()
        else:
            self.root.destroy()
            
    def run(self):
        """Run the launcher GUI"""
        self.log_message("Robot Launcher started")
        self.root.mainloop()

def main():
    launcher = RobotLauncher()
    launcher.run()

if __name__ == '__main__':
    main() 