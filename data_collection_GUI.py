"""
Raspberry Pi Motor Control System with GUI
This program provides a class-based implementation with a GUI interface
for controlling motors and logging sensor data on a Raspberry Pi with a DSI display.
"""

import os
import sys
import time
import signal
import numpy as np
import threading
import csv
from datetime import datetime
from multiprocessing import Process, Queue, Value
from ctypes import c_bool

# GUI libraries
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for Raspberry Pi
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

# For Raspberry Pi GPIO
try:
    import RPi.GPIO as GPIO
    from hx711py.hx711 import HX711
    import board
    PI_AVAILABLE = True
except ImportError:
    print("Warning: Running in simulation mode (Raspberry Pi libraries not available)")
    PI_AVAILABLE = False


class MotorControlSystem:
    """Main class for the motor control system"""
    
    def __init__(self):
        
        self.logging_active = Value(c_bool, False)
        # Configuration
        self.SENSOR_SAMPLE_RATE_HZ = 10   # How often to sample load cells (Hz)
        self.MOTOR_UPDATE_RATE_HZ = 50    # How often to update motors (Hz)
        self.LEVER_ARM_LENGTH = 0.25      # meters - distance between load cells and center of rotation
        self.filter_window_size = 10      # Filter length
        self.reading_history = {'loadcell1': [], 'loadcell2': [], 'loadcell3': []} #history of sensor readings
        # TA8248K Motor Driver pins configuration (no separate PWM pins)
        self.MOTOR_PINS = {
            'motor1': {'in1': 27, 'in2': 22},  # BCM numbering
            'motor2': {'in1': 19, 'in2': 26}   # BCM numbering
        }
        
        # HX711 Load Cell pins configuration
        self.LOADCELL_PINS = {
            'loadcell1': {'dout': 20, 'sck': 21},     # BCM numbering
            'loadcell2': {'dout': 17, 'sck': 18},     # BCM numbering
            'loadcell3': {'dout': 5, 'sck': 6}        # BCM numbering
        }
        
        # Pre-calibrated factors for load cells
        self.CALIBRATION_FACTORS = {
            'loadcell1': 219.1472,    # Already calibrated
            'loadcell2': -221.8872,   # Already calibrated 
            'loadcell3': -221.4656    # Already calibrated
        }
        
        # Wave pattern parameters
        self.period = 2.0         # seconds per cycle
        self.phase = 0.5          # seconds between motor1 and motor2
        self.latency = 0.2        # seconds of off time before changing direction
        self.reverse = False      # whether to start in reverse direction
        self.motor_speed = 70     # motor speed as percentage (0-100)
        self.use_pwm = True
        
        # File management
        self.data_directory = "test_data"
        self.active_test_name = "test"
        self.log_files = []
        
        # Common timing system
        self.cycle_start_time = None  # When the current cycle started
        self.cycle_number = 0        # Current cycle number
        self.cycle_position = 0.0    # Position in cycle (0-1)
        self.pattern_active = False  # Whether pattern is currently running
        
        # Create data directory if it doesn't exist
        os.makedirs(self.data_directory, exist_ok=True)
        print(f"Data directory path: {os.path.abspath(self.data_directory)}")
        # Check data directory permissions
        if not os.path.exists(self.data_directory):
            print(f"Creating data directory: {self.data_directory}")
            os.makedirs(self.data_directory, exist_ok=True)
        else:
            print(f"Data directory exists: {self.data_directory}")
        
        # Check write permissions
        try:
            test_file = os.path.join(self.data_directory, "test_write.txt")
            with open(test_file, 'w') as f:
                f.write("Test")
            os.remove(test_file)
            print(f"Successfully verified write permissions for {self.data_directory}")
        except Exception as e:
            print(f"Error: No write permission for data directory: {e}")
        # Create shared flag for graceful shutdown
        self.running = Value(c_bool, True)
        
        # Flag for wave pattern running
        self.wave_running = Value(c_bool, False)
        
        # Shared memory for cycle timing information
        self.cycle_info = {
            'cycle_number': Value('i', 0),
            'cycle_position': Value('d', 0.0),
            'pattern_active': Value(c_bool, False)
        }
        
        # Queues for inter-process communication
        self.sensor_queue = Queue()       # Sensor data → Logging
        self.command_queue = Queue()      # UI → Motor control
        self.clock_queue = Queue()        # Cycle info → Logging
        
        # Data storage for GUI
        self.latest_sensor_data = {
            "timestamp": datetime.now().isoformat(),
            "forces": {
                "raw_readings": [0, 0, 0],
                "thrust": 0,
                "lift": 0,
                "moment": 0
            },
            "cycle_info": {
                "cycle_number": 0,
                "cycle_position": 0.0,
                "pattern_active": False
            }
        }
        self.data_lock = threading.Lock()  # For thread-safe data access
        
        # Initialize log file
        self.log_filename = self.generate_log_filename()
        self.log_files.append(self.log_filename)
        
        # Motor state tracking
        self.motor_state = {
            'motor1': {'speed': 0, 'direction': 0},
            'motor2': {'speed': 0, 'direction': 0}
        }
        
        # Hardware setup
        if PI_AVAILABLE:
            self.setup_gpio()
            
    def update_pin_configuration(self, pin_type, device, pin_name, new_value):
        """Update pin configuration and restart hardware if needed"""
        try:
            # Convert to integer
            new_value = int(new_value)
            
            # Validate pin number (Raspberry Pi has GPIO pins 0-27)
            if new_value < 0 or new_value > 27:
                return False, f"Invalid pin number: {new_value}. Must be between 0 and 27."
                
            # Update the appropriate pin configuration
            if pin_type == "motor":
                if device in self.MOTOR_PINS and pin_name in self.MOTOR_PINS[device]:
                    # Store old value for reference
                    old_value = self.MOTOR_PINS[device][pin_name]
                    
                    # Update value
                    self.MOTOR_PINS[device][pin_name] = new_value
                    
                    # Restart hardware if running
                    if PI_AVAILABLE:
                        # Clean up and restart GPIO
                        GPIO.cleanup()
                        self.setup_gpio()
                        
                    return True, f"Updated {device} {pin_name} from {old_value} to {new_value}"
                else:
                    return False, f"Invalid motor device or pin name: {device}, {pin_name}"
            
            elif pin_type == "loadcell":
                if device in self.LOADCELL_PINS and pin_name in self.LOADCELL_PINS[device]:
                    # Store old value
                    old_value = self.LOADCELL_PINS[device][pin_name]
                    
                    # Update value
                    self.LOADCELL_PINS[device][pin_name] = new_value
                    
                    # Restart hardware if running
                    if PI_AVAILABLE:
                        # Clean up and restart GPIO
                        GPIO.cleanup()
                        self.setup_gpio()
                        
                    return True, f"Updated {device} {pin_name} from {old_value} to {new_value}"
                else:
                    return False, f"Invalid load cell device or pin name: {device}, {pin_name}"
            
            else:
                return False, f"Invalid pin type: {pin_type}"
                
        except ValueError:
            return False, f"Invalid pin number format: {new_value}. Must be an integer."
        except Exception as e:
            return False, f"Error updating pin configuration: {e}"

    def end_logging(self):
        """End the current logging session with a marker"""
        if self.logging_active.value:
            try:
                # Add a marker for the end of logging
                with open(self.log_filename, 'a') as log_file:
                    log_file.write(f"# Logging ended: {datetime.now().isoformat()}\n")
                print(f"Logging ended for {os.path.basename(self.log_filename)}")
            except Exception as e:
                print(f"Error marking end of logging: {e}")
            
            # Disable logging flag
            self.logging_active.value = False
    
    def update_calibration_factor(self, device, new_value):
        """Update calibration factor for a load cell"""
        try:
            # Convert to float
            new_value = float(new_value)
            
            # Validate the device
            if device in self.CALIBRATION_FACTORS:
                # Store old value
                old_value = self.CALIBRATION_FACTORS[device]
                
                # Update value
                self.CALIBRATION_FACTORS[device] = new_value
                
                # Update load cell calibration if running
                if PI_AVAILABLE and hasattr(self, 'load_cells') and device in self.load_cells:
                    self.load_cells[device]._calibration = new_value
                    
                return True, f"Updated {device} calibration from {old_value} to {new_value}"
            else:
                return False, f"Invalid load cell device: {device}"
                
        except ValueError:
            return False, f"Invalid calibration format: {new_value}. Must be a number."
        except Exception as e:
            return False, f"Error updating calibration: {e}"
            
    def generate_log_filename(self):
        """Generate a log filename based on test name and timestamp"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        safe_test_name = ''.join(c if c.isalnum() or c in '-_' else '_' for c in self.active_test_name)
        return os.path.join(self.data_directory, f"{safe_test_name}_{timestamp}.csv")
            
    def _init_log_file(self, filename):
        """Initialize a new log file with headers only if needed"""
        try:
            # Check if file already exists
            if os.path.exists(filename):
                file_size = os.path.getsize(filename)
                if file_size > 0:
                    print(f"Log file {filename} already exists with {file_size} bytes")
                    return
                    
            # Create the file with headers
            with open(filename, 'w') as log_file:
                log_file.write("timestamp,thrust,lift,moment,raw1,raw2,raw3,motor1_speed,motor1_dir,motor2_speed,motor2_dir,cycle_number,cycle_position,pattern_active\n")
            print(f"Initialized log file: {filename}")
        except Exception as e:
            print(f"Error creating log file {filename}: {e}")
            
    def set_test_name(self, test_name):
        """Set active test name and create a new log file"""
        if not test_name:
            test_name = "test"  # Default if empty
            
        self.active_test_name = test_name
        self.log_filename = self.generate_log_filename()
        self.log_files.append(self.log_filename)
        print(f"Test name set to {test_name}, log file will be {os.path.basename(self.log_filename)}")
        return self.log_filename
    
    def get_log_files(self):
        """Get list of all log files in the data directory"""
        all_files = []
        for file in os.listdir(self.data_directory):
            if file.endswith(".csv"):
                full_path = os.path.join(self.data_directory, file)
                file_size = os.path.getsize(full_path)
                mod_time = os.path.getmtime(full_path)
                mod_time_str = datetime.fromtimestamp(mod_time).strftime('%Y-%m-%d %H:%M:%S')
                all_files.append({
                    'filename': file,
                    'path': full_path,
                    'size': file_size,
                    'modified': mod_time_str
                })
        
        # Sort by modified time (newest first)
        all_files.sort(key=lambda x: x['modified'], reverse=True)
        return all_files
    
    def delete_log_file(self, filename):
        """Delete a log file"""
        try:
            full_path = os.path.join(self.data_directory, filename)
            os.remove(full_path)
            
            # If the deleted file was the current log file, create a new one
            if full_path == self.log_filename:
                self.log_filename = self.generate_log_filename()
                self._init_log_file(self.log_filename)
                self.log_files.append(self.log_filename)
                
            return True
        except Exception as e:
            print(f"Error deleting file: {e}")
            return False

    def setup_gpio(self):
        """Set up GPIO pins for TA8248K motor drivers and HX711 load cells"""
        # Set up GPIO
        GPIO.setmode(GPIO.BCM)
        
        # Set up TA8248K motor driver pins
        for motor in self.MOTOR_PINS.values():
            GPIO.setup(motor['in1'], GPIO.OUT)
            GPIO.setup(motor['in2'], GPIO.OUT)
        
        # Set up PWM for motors - with TA8248K we use PWM on the control pins directly
        self.pwm_motors = {
            'motor1': {
                'in1_pwm': GPIO.PWM(self.MOTOR_PINS['motor1']['in1'], 1000),  # 1kHz frequency
                'in2_pwm': GPIO.PWM(self.MOTOR_PINS['motor1']['in2'], 1000)
            },
            'motor2': {
                'in1_pwm': GPIO.PWM(self.MOTOR_PINS['motor2']['in1'], 1000),
                'in2_pwm': GPIO.PWM(self.MOTOR_PINS['motor2']['in2'], 1000)
            }
        }
        
        # Start all PWM channels at 0% duty cycle
        for motor in self.pwm_motors.values():
            motor['in1_pwm'].start(0)
            motor['in2_pwm'].start(0)
        
        # Set up HX711 load cell amplifiers
        self.load_cells = {}
        self.reading_history = {'loadcell1': [], 'loadcell2': [], 'loadcell3': []}
        self.filter_window_size = 10  # Window size for median filter
        
        # Initialize HX711 load cells using pins from configuration
        try:
            # Create HX711 objects using specified pins
            self.load_cells['loadcell1'] = HX711(self.LOADCELL_PINS['loadcell1']['dout'], 
                                                 self.LOADCELL_PINS['loadcell1']['sck'])
            self.load_cells['loadcell2'] = HX711(self.LOADCELL_PINS['loadcell2']['dout'], 
                                                 self.LOADCELL_PINS['loadcell2']['sck'])
            self.load_cells['loadcell3'] = HX711(self.LOADCELL_PINS['loadcell3']['dout'], 
                                                 self.LOADCELL_PINS['loadcell3']['sck'])
            
            # Set reading format for all load cells
            for name, load_cell in self.load_cells.items():
                load_cell.set_reading_format("MSB", "MSB")
                
                # Set reference unit from calibration factors
                if name in self.CALIBRATION_FACTORS:
                    load_cell.set_reference_unit(self.CALIBRATION_FACTORS[name])
                else:
                    load_cell.set_reference_unit(1.0)  # Default value if not calibrated
                
                # Reset and tare the load cell
                load_cell.reset()
                load_cell.tare()
                print(f"{name} tare done")
                
        except Exception as e:
            print(f"Error setting up load cells: {e}")
    
    def median_filter(self, values, window_size):
        """Apply median filter to smooth sensor readings"""
        if len(values) < window_size:
            return values[-1] if values else None
        return sorted(values[-window_size:])[window_size // 2]
    
    def read_sensors(self):
        """Read from load cells and calculate derived values using median filtering"""
        timestamp = datetime.now().isoformat()
        raw_readings = []
        
        if PI_AVAILABLE:
            # Get readings from each load cell
            for name in ['loadcell1', 'loadcell2', 'loadcell3']:
                try:
                    if name in self.load_cells:
                        print(f"Trying to read {name}...")
                        # Read weight directly using get_weight method
                        # Use 1 reading for faster response (can be adjusted)
                        raw_value = self.load_cells[name].get_weight(1)
                        print(f"{name} raw value: {raw_value}")
                        # Apply sign adjustment based on cell position if needed
                        # For example, like in the original code where loadcell2 and loadcell3 values are negated
                        if name in ['loadcell2', 'loadcell3']:
                            raw_value = -raw_value
                        
                        # Add to history and apply median filter
                        if name not in self.reading_history:
                            self.reading_history[name] = []
                        self.reading_history[name].append(raw_value)
                        
                        # Apply median filter
                        filtered_value = self.median_filter(self.reading_history[name], 
                                                           self.filter_window_size)
                        raw_readings.append(filtered_value)
                        
                        # Trim history to prevent memory growth
                        if len(self.reading_history[name]) > self.filter_window_size * 2:
                            self.reading_history[name] = self.reading_history[name][-self.filter_window_size:]
                    else:
                        raw_readings.append(None)
                except Exception as e:
                    raw_readings.append(None)
                    print(f"Sensor reading error ({name}): {e}")
        else:
            # Simulation mode - generate random values
            raw_readings = [
                np.random.random() * 10.0,  # Thrust
                np.random.random() * 5.0,   # Lift 1
                np.random.random() * 5.0    # Lift 2
            ]
        
        # Calculate derived forces if all readings are valid
        forces = {
            "raw_readings": raw_readings,
            "thrust": None,
            "lift": None, 
            "moment": None
        }
        
        # Only calculate if we have all three sensor readings
        if None not in raw_readings and len(raw_readings) == 3:
            # Sensor 1 measures thrust directly
            forces["thrust"] = raw_readings[0]
            
            # Lift is average of sensors 2 and 3
            forces["lift"] = (raw_readings[1] + raw_readings[2]) / 2
            
            # In-plane moment is (sensor2 - sensor3) * lever arm length
            forces["moment"] = (raw_readings[1] - raw_readings[2]) * self.LEVER_ARM_LENGTH
        
        data = {
            "timestamp": timestamp,
            "forces": forces
        }
        return data

    def set_motor(self, motor_name, direction, speed):
        """Set motor speed and direction"""
        # Calculate actual motor names to use
        if motor_name == "both":
            motor_names = ["motor1", "motor2"]
        else:
            motor_names = [motor_name]
        
        # If in digital mode, force speed to 100%
        if hasattr(self, 'use_pwm') and not self.use_pwm:
            speed = 100
        
        # Update motor state dictionary
        for name in motor_names:
            self.motor_state[name] = {'speed': speed, 'direction': direction}
        
        # Update physical motors if on Raspberry Pi
        if PI_AVAILABLE:
            for name in motor_names:
                if name in self.pwm_motors:
                    if hasattr(self, 'use_pwm') and not self.use_pwm:
                        # Digital control mode (no PWM)
                        if direction > 0:  # Forward
                            GPIO.output(self.MOTOR_PINS[name]['in1'], GPIO.HIGH)
                            GPIO.output(self.MOTOR_PINS[name]['in2'], GPIO.LOW)
                        elif direction < 0:  # Reverse
                            GPIO.output(self.MOTOR_PINS[name]['in1'], GPIO.LOW)
                            GPIO.output(self.MOTOR_PINS[name]['in2'], GPIO.HIGH)
                        else:  # Brake/Stop
                            GPIO.output(self.MOTOR_PINS[name]['in1'], GPIO.LOW)
                            GPIO.output(self.MOTOR_PINS[name]['in2'], GPIO.LOW)
                    else:
                        # PWM control mode
                        # For TA8248K, we apply PWM to the control pins directly
                        # TA8248K control:
                        # IN1=PWM, IN2=0: Forward with speed control
                        # IN1=0, IN2=PWM: Reverse with speed control
                        # IN1=0, IN2=0: Brake/Stop
                        
                        if direction > 0:  # Forward
                            self.pwm_motors[name]['in1_pwm'].ChangeDutyCycle(speed)
                            self.pwm_motors[name]['in2_pwm'].ChangeDutyCycle(0)
                        elif direction < 0:  # Reverse
                            self.pwm_motors[name]['in1_pwm'].ChangeDutyCycle(0)
                            self.pwm_motors[name]['in2_pwm'].ChangeDutyCycle(speed)
                        else:  # Brake/Stop
                            self.pwm_motors[name]['in1_pwm'].ChangeDutyCycle(0)
                            self.pwm_motors[name]['in2_pwm'].ChangeDutyCycle(0)
        
        # Put command in queue for logging
        self.command_queue.put(self.motor_state.copy())

    def run_test_sequence(self, callback=None):
        """Run a predefined test sequence for motors"""
        # Define motor test sequence (open loop control)
        # Format: [motor_name, direction, speed, duration]
        test_sequence = [
            # Motor 1 tests (forward)
            ["motor1", 1, 25, 5],   # Forward at 25% for 5 seconds
            ["motor1", 1, 50, 5],   # Forward at 50% for 5 seconds
            ["motor1", 1, 75, 5],   # Forward at 75% for 5 seconds
            ["motor1", 0, 0, 2],    # Stop for 2 seconds
            
            # Motor 1 tests (reverse)
            ["motor1", -1, 25, 5],  # Reverse at 25% for 5 seconds
            ["motor1", -1, 50, 5],  # Reverse at 50% for 5 seconds
            ["motor1", -1, 75, 5],  # Reverse at 75% for 5 seconds
            ["motor1", 0, 0, 2],    # Stop for 2 seconds
            
            # Motor 2 tests (forward)
            ["motor2", 1, 25, 5],   # Forward at 25% for 5 seconds
            ["motor2", 1, 50, 5],   # Forward at 50% for 5 seconds
            ["motor2", 1, 75, 5],   # Forward at 75% for 5 seconds
            ["motor2", 0, 0, 2],    # Stop for 2 seconds
            
            # Motor 2 tests (reverse)
            ["motor2", -1, 25, 5],  # Reverse at 25% for 5 seconds
            ["motor2", -1, 50, 5],  # Reverse at 50% for 5 seconds
            ["motor2", -1, 75, 5],  # Reverse at 75% for 5 seconds
            ["motor2", 0, 0, 2],    # Stop for 2 seconds
            
            # Combined motor tests
            ["both", 1, 50, 5],     # Both forward at 50% for 5 seconds
            ["both", -1, 50, 5],    # Both reverse at 50% for 5 seconds
            ["both", 0, 0, 2],      # Both stop for 2 seconds
        ]
            # Enable logging for this test
        self.logging_active.value = True
        print(f"Starting logging to {os.path.basename(self.log_filename)}")
        
        # Create the log file if it doesn't exist yet
        try:
            with open(self.log_filename, 'a') as log_file:
                # Check if the file is empty (needs headers)
                if log_file.tell() == 0:
                    log_file.write("timestamp,thrust,lift,moment,raw1,raw2,raw3,motor1_speed,motor1_dir,motor2_speed,motor2_dir,cycle_number,cycle_position,pattern_active\n")
                # Add a marker for this test
                log_file.write(f"# Test sequence started\n")
        except Exception as e:
            print(f"Error preparing log file: {e}")
        # Run in a separate thread to avoid blocking the GUI
        def sequence_thread():
            try:
                for step in test_sequence:
                    if not self.running.value:
                        break
                    motor, direction, speed, duration = step
                    status_msg = f"Setting {motor} to direction={direction}, speed={speed}% for {duration}s"
                    if callback:
                        callback(status_msg)
                    print(status_msg)
                    self.set_motor(motor, direction, speed)
                    time.sleep(duration)
                
                # Ensure motors are stopped at the end
                self.set_motor("both", 0, 0)
                if callback:
                    callback("Test sequence completed")
                print("Test sequence completed")
                self.logging_active.value = False
                print("Test sequence complete - logging stopped")
            
            except Exception as e:
                if callback:
                    callback(f"Test sequence error: {e}")
                print(f"Test sequence error: {e}")
                self.logging_active.value = False
        
        # Start the thread
        thread = threading.Thread(target=sequence_thread)
        thread.daemon = True
        thread.start()
        return thread
        
    def start_sensor_process(self):
        """Start the sensor data collection process"""
        def sensor_process_func():
            print("Sensor process started")
            
            while self.running.value:
                try:
                    # Read sensor data
                    sensor_data = self.read_sensors()
                    print(f"Sensor data: {sensor_data['forces']['raw_readings']}")
                    # Store the latest data for GUI access
                    with self.data_lock:
                        self.latest_sensor_data = sensor_data
                    
                    # Put data in the queue for logging
                    self.sensor_queue.put(sensor_data)
                    print("Added sensor data to queue")
                    # Sleep to maintain sample rate
                    time.sleep(1.0 / self.SENSOR_SAMPLE_RATE_HZ)
                    
                except Exception as e:
                    print(f"Error in sensor process: {e}")
                    time.sleep(0.1)  # Brief pause on error
                
            print("Sensor process stopped")
        
        # Create and start the process
        process = Process(target=sensor_process_func)
        process.daemon = True
        process.start()
        return process

    def start_logging_process(self):
        """Start the data logging process"""
        def logging_process_func():
            print("Logging process started")
            # Keep track of the last motor commands
            motor_commands = {
                'motor1': {'speed': 0, 'direction': 0},
                'motor2': {'speed': 0, 'direction': 0}
            }
            
            # Keep track of current cycle information
            cycle_info = {
                'cycle_number': 0,
                'cycle_position': 0.0,
                'pattern_active': False
            }
            
            # Current log file
            current_log_file = self.log_filename

            first_write_done = False
            
            # NEW: Data counters
            data_points_queued = 0
            data_points_logged = 0
            
            while self.running.value:
                try:
                    # Check if log file has changed
                    if current_log_file != self.log_filename:
                        current_log_file = self.log_filename
                        first_write_done = False  # NEW: Reset write flag for new file
                    
                    # Check for cycle information updates
                    if not self.clock_queue.empty():
                        cycle_info = self.clock_queue.get()
                    
                    # Check for new motor commands
                    if not self.command_queue.empty():
                        motor_commands = self.command_queue.get()
                    
                    # Check for new sensor data
                    if not self.sensor_queue.empty():
                        sensor_data = self.sensor_queue.get()
                        data_points_queued += 1
                        print(f"Logging process received data: {sensor_data['forces']['raw_readings']}")
                        # Store cycle information with sensor data for GUI access
                        with self.data_lock:
                            self.latest_sensor_data["cycle_info"] = cycle_info.copy()

                        if self.logging_active.value:
                        # NEW: Initialize file on first write if needed
                            if not first_write_done:
                                if not os.path.exists(current_log_file) or os.path.getsize(current_log_file) == 0:
                                    with open(current_log_file, 'w') as log_file:
                                        log_file.write("timestamp,thrust,lift,moment,raw1,raw2,raw3,motor1_speed,motor1_dir,motor2_speed,motor2_dir,cycle_number,cycle_position,pattern_active\n")
                                first_write_done = True
                        
                            # Extract values
                            timestamp = sensor_data["timestamp"]
                            forces = sensor_data["forces"]
                            raw_readings = forces["raw_readings"]
                            
                            # Format values for logging
                            thrust = forces["thrust"] if forces["thrust"] is not None else "NA"
                            lift = forces["lift"] if forces["lift"] is not None else "NA"
                            moment = forces["moment"] if forces["moment"] is not None else "NA"
                            
                            raw1 = raw_readings[0] if len(raw_readings) > 0 and raw_readings[0] is not None else "NA"
                            raw2 = raw_readings[1] if len(raw_readings) > 1 and raw_readings[1] is not None else "NA"
                            raw3 = raw_readings[2] if len(raw_readings) > 2 and raw_readings[2] is not None else "NA"
                            
                            # Extract motor data
                            motor1_speed = motor_commands['motor1']['speed'] if 'motor1' in motor_commands else 0
                            motor1_dir = motor_commands['motor1']['direction'] if 'motor1' in motor_commands else 0
                            motor2_speed = motor_commands['motor2']['speed'] if 'motor2' in motor_commands else 0
                            motor2_dir = motor_commands['motor2']['direction'] if 'motor2' in motor_commands else 0
                            
                            # Extract cycle info
                            cycle_number = cycle_info['cycle_number']
                            cycle_position = cycle_info['cycle_position']
                            pattern_active = 1 if cycle_info['pattern_active'] else 0
                            try:
                            # Write to log file
                                with open(current_log_file, 'a') as log_file:
                                    log_entry = (
                                        f"{timestamp},{thrust},{lift},{moment},{raw1},{raw2},{raw3},"
                                        f"{motor1_speed},{motor1_dir},{motor2_speed},{motor2_dir},"
                                        f"{cycle_number},{cycle_position:.4f},{pattern_active}\n"
                                    )
                                    log_file.write(log_entry)
                                    log_file.flush()  # NEW: Ensure data is written immediately
                                    data_points_logged += 1  # NEW: Count logged data
    
                                    if data_points_logged % 100 == 0:
                                        print(f"Logged {data_points_logged} data points to {os.path.basename(current_log_file)}")
    
                            except Exception as e:
                                print(f"Error writing to log file {current_log_file}: {e}")
                    # Brief sleep to avoid CPU spinning
                    time.sleep(0.01)
                    
                except Exception as e:
                    print(f"Error in logging process: {e}")
                    time.sleep(0.1)  # Brief pause on error
            
            print("Logging process stopped")
        
        # Create and start the process
        process = Process(target=logging_process_func)
        process.daemon = True
        process.start()
        return process
    
    def stop_all_motors(self):
        """Stop all motors"""
        self.set_motor("both", 0, 0)
    
    def update_cycle_position(self, position, cycle_number, active):
        """Update and synchronize cycle position information"""
        # Update shared memory values
        self.cycle_info['cycle_position'].value = position
        self.cycle_info['cycle_number'].value = cycle_number
        self.cycle_info['pattern_active'].value = active
        
        # Send update to logging process
        self.clock_queue.put({
            'cycle_number': cycle_number,
            'cycle_position': position,
            'pattern_active': active
        })
    
    def run_wave_pattern(self, num_cycles=10, callback=None):
        """Run motors in square wave pattern according to parameters"""
        # Stop any previous wave pattern
        if self.wave_running.value:
            self.wave_running.value = False
            time.sleep(0.5)  # Give time for previous pattern to stop

        self.log_filename = self.generate_log_filename()
        self.log_files.append(self.log_filename)
        
        self.wave_running.value = True
        self.logging_active.value = True
        self._init_log_file(self.log_filename)
        print(f"Starting logging to {os.path.basename(self.log_filename)}")

        try:
            with open(self.log_filename, 'a') as log_file:
                log_file.write(f"# Wave pattern test started: {num_cycles} cycles, period={self.period}s, phase={self.phase}s\n")
        except Exception as e:
            print(f"Error marking log file: {e}")
        
            # Create the log file if it doesn't exist yet
        try:
            with open(self.log_filename, 'a') as log_file:
                # Check if the file is empty (needs headers)
                if log_file.tell() == 0:
                    log_file.write("timestamp,thrust,lift,moment,raw1,raw2,raw3,motor1_speed,motor1_dir,motor2_speed,motor2_dir,cycle_number,cycle_position,pattern_active\n")
                # Add a marker for this test
                log_file.write(f"# Wave pattern test started: {num_cycles} cycles, period={self.period}s, phase={self.phase}s\n")
        except Exception as e:
            print(f"Error preparing log file: {e}")
        # Reset cycle information
        self.cycle_info['cycle_number'].value = 0
        self.cycle_info['cycle_position'].value = 0.0
        self.cycle_info['pattern_active'].value = True
        
        # Send initial cycle info to logging process
        self.clock_queue.put({
            'cycle_number': 0,
            'cycle_position': 0.0,
            'pattern_active': True
        })
        
        # Function to run in a separate thread
        def wave_thread():
            try:
                if callback:
                    callback(f"Starting wave pattern: {num_cycles} cycles")
                
                # Initial direction based on reverse flag
                direction = -1 if self.reverse else 1
                speed = self.motor_speed
                
                # Start with half cycle to get motors up from zero position
                if callback:
                    callback("Starting initial half cycle")
                
                # Record cycle start time
                cycle_start_time = time.time()
                
                # Set motor 1 first
                self.set_motor("motor1", direction, speed)
                
                # Update cycle position - motor 1 start
                self.update_cycle_position(0.0, 0, True)
                
                # Wait for phase delay then set motor 2
                time.sleep(self.phase)
                if not self.wave_running.value:
                    self.stop_all_motors()
                    return
                
                # Set motor 2
                self.set_motor("motor2", direction, speed)
                
                # Update cycle position - motor 2 start
                phase_fraction = self.phase / self.period
                self.update_cycle_position(phase_fraction, 0, True)
                
                # Wait until half cycle is complete
                time.sleep(self.period / 2 - self.phase - self.latency)
                if not self.wave_running.value:
                    self.stop_all_motors()
                    return
                
                # Main cycle loop
                for cycle in range(num_cycles):
                    if not self.wave_running.value:
                        break
                    
                    # Update cycle number
                    self.cycle_info['cycle_number'].value = cycle + 1
                    
                    if callback:
                        callback(f"Running cycle {cycle+1}/{num_cycles}")
                    
                    # Reverse direction
                    direction = -direction
                    
                    # Stop motor 1, wait for latency, then reverse direction
                    self.set_motor("motor1", 0, 0)
                    
                    # Update cycle position - motor 1 stop
                    self.update_cycle_position(0.5, cycle + 1, True)
                    
                    time.sleep(self.latency)
                    if not self.wave_running.value:
                        break
                    
                    # Start motor 1 in reverse
                    self.set_motor("motor1", direction, speed)
                    
                    # Update cycle position - motor 1 reverse
                    latency_fraction = self.latency / self.period
                    self.update_cycle_position(0.5 + latency_fraction, cycle + 1, True)
                    
                    # Wait for phase delay
                    time.sleep(self.phase)
                    if not self.wave_running.value:
                        break
                    
                    # Stop motor 2, wait for latency, then reverse direction
                    self.set_motor("motor2", 0, 0)
                    
                    # Update cycle position - motor 2 stop
                    self.update_cycle_position(0.5 + latency_fraction + phase_fraction, cycle + 1, True)
                    
                    time.sleep(self.latency)
                    if not self.wave_running.value:
                        break
                    
                    # Start motor 2 in reverse
                    self.set_motor("motor2", direction, speed)
                    
                    # Update cycle position - motor 2 reverse
                    self.update_cycle_position(0.5 + 2*latency_fraction + phase_fraction, cycle + 1, True)
                    
                    # Wait until cycle is complete
                    wait_time = self.period - self.phase - 2 * self.latency
                    time.sleep(max(0, wait_time))
                    if not self.wave_running.value:
                        break
                    
                    # Update cycle position - cycle complete
                    self.update_cycle_position(1.0, cycle + 1, True)
                
                # Final half cycle to return to zero
                if self.wave_running.value:
                    if callback:
                        callback("Running final half cycle to return to zero")
                    
                    # Update cycle number for final half cycle
                    final_cycle = num_cycles + 1
                    self.cycle_info['cycle_number'].value = final_cycle
                    
                    # Reverse direction one last time
                    direction = -direction
                    
                    # Stop motor 1, wait for latency, then reverse direction
                    self.set_motor("motor1", 0, 0)
                    
                    # Update cycle position - final half cycle motor 1 stop
                    self.update_cycle_position(0.0, final_cycle, True)
                    
                    time.sleep(self.latency)
                    if not self.wave_running.value:
                        self.stop_all_motors()
                        return
                    
                    # Start motor 1 in reverse
                    self.set_motor("motor1", direction, speed)
                    
                    # Update cycle position - final half cycle motor 1 reverse
                    self.update_cycle_position(latency_fraction, final_cycle, True)
                    
                    # Wait for phase delay
                    time.sleep(self.phase)
                    if not self.wave_running.value:
                        self.stop_all_motors()
                        return
                    
                    # Stop motor 2, wait for latency, then reverse direction
                    self.set_motor("motor2", 0, 0)
                    
                    # Update cycle position - final half cycle motor 2 stop
                    self.update_cycle_position(latency_fraction + phase_fraction, final_cycle, True)
                    
                    time.sleep(self.latency)
                    if not self.wave_running.value:
                        self.stop_all_motors()
                        return
                    
                    # Start motor 2 in reverse
                    self.set_motor("motor2", direction, speed)
                    
                    # Update cycle position - final half cycle motor 2 reverse
                    self.update_cycle_position(latency_fraction + phase_fraction + latency_fraction, final_cycle, True)
                    
                    # Wait half cycle for completion
                    time.sleep(self.period / 2 - self.phase - self.latency)
                    
                    # Update cycle position - pattern complete
                    self.update_cycle_position(0.5, final_cycle, True)
                
                # Stop all motors at the end
                self.stop_all_motors()

                        # Mark pattern as inactive
                self.cycle_info['pattern_active'].value = False
                self.clock_queue.put({
                    'cycle_number': self.cycle_info['cycle_number'].value,
                    'cycle_position': self.cycle_info['cycle_position'].value,
                    'pattern_active': False
                })
                
                # IMPORTANT: Disable logging when pattern completes successfully
                self.logging_active.value = False
                
                if callback and self.wave_running.value:
                    callback("Wave pattern completed")
                
            except Exception as e:
                if callback:
                    callback(f"Error in wave pattern: {e}")
                    print(f"Error in wave pattern: {e}")
                    
                    # IMPORTANT: Disable logging on error
                    self.logging_active.value = False
                
            finally:
                self.wave_running.value = False
                self.cycle_info['pattern_active'].value = False
                self.clock_queue.put({
                    'cycle_number': self.cycle_info['cycle_number'].value,
                    'cycle_position': self.cycle_info['cycle_position'].value,
                    'pattern_active': False
                })
                self.stop_all_motors()
                self.logging_active.value = False
                print(f"Wave pattern complete - logging stopped")
        
        # Start the wave pattern in a separate thread
        thread = threading.Thread(target=wave_thread)
        thread.daemon = True
        thread.start()
        return thread
    
    def stop_wave_pattern(self):
        """Stop the running wave pattern"""
        self.wave_running.value = False
        time.sleep(0.2)  # Give a moment for the thread to notice the flag
        self.stop_all_motors()
        self.end_logging()
        print("Wave pattern stopped - logging stopped")

    def cleanup(self):
        """Clean up resources"""
        print("Cleaning up resources...")
        self.running.value = False
        self.logging_active.value = False
        # Stop motors
        if PI_AVAILABLE:
            try:
                for motor_name, pwm in self.pwm_motors.items():
                    # Stop both PWM channels
                    pwm['in1_pwm'].ChangeDutyCycle(0)
                    pwm['in2_pwm'].ChangeDutyCycle(0)
                    pwm['in1_pwm'].stop()
                    pwm['in2_pwm'].stop()
                
                # Clean up GPIO
                GPIO.cleanup()
            except Exception as e:
                print(f"Error cleaning up GPIO: {e}")


class MotorControlGUI:
    """GUI for the Motor Control System"""
        
    def __init__(self, root):
        self.root = root
        self.root.title("Motor Control System")
        
        # Configure for 5-inch DSI display (800x480)
        self.root.attributes('-fullscreen', True)
        self.root.geometry("800x480")  # Set explicit window size
        
        # Set appropriate font size for small touch screens
        default_font = tk.font.nametofont("TkDefaultFont")
        default_font.configure(size=10)  # Reduce from 12 to 10
        self.root.option_add("*Font", default_font)
        
        # Create the motor control system
        self.system = MotorControlSystem()
        
        # Create the UI
        self.create_ui()
        
        # Start data update timer (100ms)
        self.root.after(100, self.update_data_display)
        
        # Start the processes
        self.processes = []
        
        # Set up cleanup on window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
            
    def create_ui(self):
        """Create the user interface"""
        # Create a notebook (tabbed interface) with smaller padding
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)  # Reduced padding
        
        # Create tabs
        self.control_tab = ttk.Frame(self.notebook)
        self.data_tab = ttk.Frame(self.notebook)
        self.files_tab = ttk.Frame(self.notebook)
        self.pinout_tab = ttk.Frame(self.notebook)
        self.settings_tab = ttk.Frame(self.notebook)
        
        self.notebook.add(self.control_tab, text="Control")
        self.notebook.add(self.data_tab, text="Data")
        self.notebook.add(self.files_tab, text="Files")
        self.notebook.add(self.pinout_tab, text="Pinout")
        self.notebook.add(self.settings_tab, text="Settings")
        
        # Setup each tab
        self.setup_control_tab()
        self.setup_data_tab()
        self.setup_files_tab()
        self.setup_pinout_tab()
        self.setup_settings_tab()
    
    def setup_control_tab(self):
        """Set up the motor control tab with more compact layout"""
        # Create a canvas with scrollbar for overflow content
        control_canvas = tk.Canvas(self.control_tab)
        scrollbar = ttk.Scrollbar(self.control_tab, orient="vertical", command=control_canvas.yview)
        scrollable_frame = ttk.Frame(control_canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: control_canvas.configure(scrollregion=control_canvas.bbox("all"))
        )
        
        control_canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        control_canvas.configure(yscrollcommand=scrollbar.set)
        
        control_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Test configuration - more compact
        test_frame = ttk.LabelFrame(scrollable_frame, text="Test Configuration")
        test_frame.pack(fill=tk.X, padx=3, pady=2)
        
        # Test name entry
        ttk.Label(test_frame, text="Test Name:").grid(row=0, column=0, padx=3, pady=2, sticky=tk.W)
        self.test_name_var = tk.StringVar(value=self.system.active_test_name)
        ttk.Entry(test_frame, textvariable=self.test_name_var, width=15).grid(row=0, column=1, padx=3, pady=2, sticky=tk.W)
        
        # Set test name button
        ttk.Button(test_frame, text="Set", 
                  command=self.set_test_name, width=8).grid(row=0, column=2, padx=3, pady=2)
        
        # Current log file display
        self.log_file_var = tk.StringVar(value=f"Log: {os.path.basename(self.system.log_filename)}")
        ttk.Label(test_frame, textvariable=self.log_file_var).grid(row=1, column=0, columnspan=3, padx=3, pady=1, sticky=tk.W)
        
        # Motor control mode selection (PWM vs Digital)
        control_mode_frame = ttk.LabelFrame(scrollable_frame, text="Motor Control Mode")
        control_mode_frame.pack(fill=tk.X, padx=3, pady=2)
        
        # Create a variable to track the motor control mode
        self.motor_control_mode = tk.StringVar(value="PWM")
        
        # Create radio buttons for PWM vs Digital
        ttk.Radiobutton(control_mode_frame, text="PWM Control", 
                       variable=self.motor_control_mode, value="PWM",
                       command=self.update_motor_control_mode).grid(row=0, column=0, padx=10, pady=2, sticky=tk.W)
        
        ttk.Radiobutton(control_mode_frame, text="Digital Control", 
                       variable=self.motor_control_mode, value="Digital",
                       command=self.update_motor_control_mode).grid(row=0, column=1, padx=10, pady=2, sticky=tk.W)
        
        # Wave Pattern Parameters
        wave_frame = ttk.LabelFrame(scrollable_frame, text="Wave Pattern Parameters")
        wave_frame.pack(fill=tk.X, padx=3, pady=2)
        
        # Period setting
        ttk.Label(wave_frame, text="Period (s):").grid(row=0, column=0, padx=3, pady=2, sticky=tk.W)
        self.period_var = tk.DoubleVar(value=self.system.period)
        ttk.Entry(wave_frame, textvariable=self.period_var, width=6).grid(row=0, column=1, padx=3, pady=2, sticky=tk.W)
        ttk.Label(wave_frame, text="Cycle time").grid(row=0, column=2, padx=3, pady=2, sticky=tk.W)
        
        # Phase setting
        ttk.Label(wave_frame, text="Phase (s):").grid(row=1, column=0, padx=3, pady=2, sticky=tk.W)
        self.phase_var = tk.DoubleVar(value=self.system.phase)
        ttk.Entry(wave_frame, textvariable=self.phase_var, width=6).grid(row=1, column=1, padx=3, pady=2, sticky=tk.W)
        ttk.Label(wave_frame, text="Motor delay").grid(row=1, column=2, padx=3, pady=2, sticky=tk.W)
        
        # Latency setting
        ttk.Label(wave_frame, text="Latency (s):").grid(row=2, column=0, padx=3, pady=2, sticky=tk.W)
        self.latency_var = tk.DoubleVar(value=self.system.latency)
        ttk.Entry(wave_frame, textvariable=self.latency_var, width=6).grid(row=2, column=1, padx=3, pady=2, sticky=tk.W)
        ttk.Label(wave_frame, text="Off time").grid(row=2, column=2, padx=3, pady=2, sticky=tk.W)
        
        # Speed setting
        ttk.Label(wave_frame, text="Speed (%):").grid(row=3, column=0, padx=3, pady=2, sticky=tk.W)
        self.speed_var = tk.IntVar(value=self.system.motor_speed)
        self.speed_scale = ttk.Scale(wave_frame, from_=0, to=100, orient=tk.HORIZONTAL, 
                             variable=self.speed_var, length=150)
        self.speed_scale.grid(row=3, column=1, padx=3, pady=2, sticky=tk.W+tk.E)
        self.speed_label = ttk.Label(wave_frame, textvariable=self.speed_var)
        self.speed_label.grid(row=3, column=2, padx=3, pady=2, sticky=tk.W)
        
        # Reverse direction checkbox
        self.reverse_var = tk.BooleanVar(value=self.system.reverse)
        ttk.Checkbutton(wave_frame, text="Start in Reverse", 
                       variable=self.reverse_var).grid(row=4, column=0, columnspan=3, padx=3, pady=2, sticky=tk.W)
        
        # Number of cycles
        ttk.Label(wave_frame, text="Cycles:").grid(row=5, column=0, padx=3, pady=2, sticky=tk.W)
        self.cycles_var = tk.IntVar(value=10)
        ttk.Spinbox(wave_frame, from_=1, to=1000, textvariable=self.cycles_var, width=6).grid(row=5, column=1, padx=3, pady=2, sticky=tk.W)
        
        # Apply parameters button
        ttk.Button(wave_frame, text="Apply Parameters", 
                  command=self.apply_wave_parameters, width=15).grid(row=6, column=0, columnspan=2, padx=3, pady=2)
        
        # Control buttons
        control_frame = ttk.Frame(wave_frame)
        control_frame.grid(row=7, column=0, columnspan=3, padx=3, pady=2)
        
        ttk.Button(control_frame, text="Start Wave", 
                  command=self.start_wave_pattern, width=12).grid(row=0, column=0, padx=3, pady=2)
        ttk.Button(control_frame, text="Stop Wave", 
                  command=self.stop_wave_pattern, width=12).grid(row=0, column=1, padx=3, pady=2)
        
        # Manual Control Frame - with more compact layout
        manual_frame = ttk.LabelFrame(scrollable_frame, text="Manual Motor Control")
        manual_frame.pack(fill=tk.X, padx=3, pady=2)
        
        # Motor control buttons - compact layout for touchscreen
        button_frame = ttk.Frame(manual_frame)
        button_frame.pack(fill=tk.X, padx=3, pady=2)
        
        # Motor 1 controls
        m1_frame = ttk.LabelFrame(button_frame, text="Motor 1")
        m1_frame.grid(row=0, column=0, padx=3, pady=2, sticky=tk.W+tk.E)
        
        ttk.Button(m1_frame, text="Fwd", 
                  command=lambda: self.system.set_motor("motor1", 1, self.speed_var.get()), 
                  width=6).grid(row=0, column=0, padx=2, pady=2)
        ttk.Button(m1_frame, text="Stop", 
                  command=lambda: self.system.set_motor("motor1", 0, 0), 
                  width=6).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(m1_frame, text="Rev", 
                  command=lambda: self.system.set_motor("motor1", -1, self.speed_var.get()), 
                  width=6).grid(row=0, column=2, padx=2, pady=2)
        
        # Motor 2 controls
        m2_frame = ttk.LabelFrame(button_frame, text="Motor 2")
        m2_frame.grid(row=0, column=1, padx=3, pady=2, sticky=tk.W+tk.E)
        
        ttk.Button(m2_frame, text="Fwd", 
                  command=lambda: self.system.set_motor("motor2", 1, self.speed_var.get()), 
                  width=6).grid(row=0, column=0, padx=2, pady=2)
        ttk.Button(m2_frame, text="Stop", 
                  command=lambda: self.system.set_motor("motor2", 0, 0), 
                  width=6).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(m2_frame, text="Rev", 
                  command=lambda: self.system.set_motor("motor2", -1, self.speed_var.get()), 
                  width=6).grid(row=0, column=2, padx=2, pady=2)
        
        # Combined controls
        combined_frame = ttk.LabelFrame(button_frame, text="Both Motors")
        combined_frame.grid(row=1, column=0, columnspan=2, padx=3, pady=2, sticky=tk.W+tk.E)
        
        ttk.Button(combined_frame, text="Both Fwd", 
                  command=lambda: self.system.set_motor("both", 1, self.speed_var.get()), 
                  width=10).grid(row=0, column=0, padx=2, pady=2)
        ttk.Button(combined_frame, text="Stop All", 
                  command=self.system.stop_all_motors, 
                  width=10).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(combined_frame, text="Both Rev", 
                  command=lambda: self.system.set_motor("both", -1, self.speed_var.get()), 
                  width=10).grid(row=0, column=2, padx=2, pady=2)
        
        # Status display
        status_frame = ttk.LabelFrame(scrollable_frame, text="Status")
        status_frame.pack(fill=tk.X, padx=3, pady=2)
        
        self.status_var = tk.StringVar(value="System ready")
        ttk.Label(status_frame, textvariable=self.status_var, 
                font=("TkDefaultFont", 10, "bold")).pack(padx=3, pady=2)

    def ensure_processes_running(self):
        """Make sure sensor and logging processes are running"""
        if not self.processes:
            self.start_system_processes()
            return True  # Processes were started
        return False  # Processes were already running
    
    def update_motor_control_mode(self):
        """Update the motor control mode between PWM and digital"""
        mode = self.motor_control_mode.get()
        
        if mode == "PWM":
            # Enable speed control for PWM mode
            self.speed_scale.configure(state="normal")
            self.speed_label.configure(state="normal")
            self.system.use_pwm = True
            self.status_var.set("PWM control mode activated")
        else:  # Digital mode
            # Fix speed at 100% for digital mode
            self.speed_var.set(100)
            self.speed_scale.configure(state="disabled")
            self.speed_label.configure(state="disabled")
            self.system.use_pwm = False
            self.status_var.set("Digital control mode activated")
    
    def start_system_processes(self):
        """Start the sensor and logging processes"""
        if not self.processes:
            self.status_var.set("Starting sensor and logging systems...")
            self.system.running.value = True
            self.processes.append(self.system.start_sensor_process())
            self.processes.append(self.system.start_logging_process())
            self.status_var.set("System processes started")
            print("Sensor and logging processes started")
        else:
            self.status_var.set("System processes already running")

    def stop_system_processes(self):
        """Stop the sensor and logging processes"""
        if self.processes:
            self.status_var.set("Stopping system processes...")
            
            # Signal processes to stop
            self.system.running.value = False
            self.system.logging_active.value = False
            self.system.wave_running.value = False
            self.system.cycle_info['pattern_active'].value = False
            
            # Wait for processes to terminate
            for process in self.processes:
                process.join(timeout=1.0)
            
            # Clear process list
            self.processes = []
            
            self.status_var.set("System processes stopped")
            print("Sensor and logging processes stopped")
        else:
            self.status_var.set("No processes running")
    
    def start_wave_pattern(self):
        """Start the wave pattern with the configured parameters"""
        try:
            # Reset system prior to test
            self.stop_system_processes()
            self.start_system_processes()
            # Get number of cycles from UI
            num_cycles = self.cycles_var.get()
            
            # Update status
            self.status_var.set(f"Starting wave pattern for {num_cycles} cycles...")
            
            # Start the wave pattern
            self.system.run_wave_pattern(num_cycles=num_cycles, callback=self.update_status)
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start wave pattern: {e}")
            self.status_var.set(f"Error: {e}")
    
    def stop_wave_pattern(self):
        """Stop the currently running wave pattern"""
        try:
            # Call the system's stop method
            self.system.stop_wave_pattern()
            
            # Update status
            self.status_var.set("Wave pattern stopped")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to stop wave pattern: {e}")
            self.status_var.set(f"Error: {e}")
    
    def setup_data_tab(self):
        """Set up the data display tab with compact layout but keeping all elements"""
        # Create a frame to hold two columns
        main_frame = ttk.Frame(self.data_tab)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # Left column for text data
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # Right column for plot
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # Real-time data display - more compact
        data_frame = ttk.LabelFrame(left_frame, text="Sensor Readings")
        data_frame.pack(fill=tk.X, padx=2, pady=2)
        
        # Sensor values with smaller font
        self.thrust_var = tk.StringVar(value="Thrust: 0.0")
        self.lift_var = tk.StringVar(value="Lift: 0.0")
        self.moment_var = tk.StringVar(value="Moment: 0.0")
        
        ttk.Label(data_frame, textvariable=self.thrust_var, 
                font=("TkDefaultFont", 11)).pack(anchor=tk.W, padx=5, pady=2)
        ttk.Label(data_frame, textvariable=self.lift_var, 
                font=("TkDefaultFont", 11)).pack(anchor=tk.W, padx=5, pady=2)
        ttk.Label(data_frame, textvariable=self.moment_var, 
                font=("TkDefaultFont", 11)).pack(anchor=tk.W, padx=5, pady=2)
        
        # Cycle information
        self.cycle_info_var = tk.StringVar(value="Cycle: Not active")
        ttk.Label(data_frame, textvariable=self.cycle_info_var,
                 font=("TkDefaultFont", 11, "bold")).pack(anchor=tk.W, padx=5, pady=2)
        
        # Raw sensor data
        raw_frame = ttk.LabelFrame(left_frame, text="Raw Sensor Data")
        raw_frame.pack(fill=tk.X, padx=2, pady=2)
        
        self.raw1_var = tk.StringVar(value="Sensor 1: 0.0")
        self.raw2_var = tk.StringVar(value="Sensor 2: 0.0")
        self.raw3_var = tk.StringVar(value="Sensor 3: 0.0")
        
        ttk.Label(raw_frame, textvariable=self.raw1_var).pack(anchor=tk.W, padx=5, pady=1)
        ttk.Label(raw_frame, textvariable=self.raw2_var).pack(anchor=tk.W, padx=5, pady=1)
        ttk.Label(raw_frame, textvariable=self.raw3_var).pack(anchor=tk.W, padx=5, pady=1)
        
        # Motor status
        motor_status_frame = ttk.LabelFrame(left_frame, text="Motor Status")
        motor_status_frame.pack(fill=tk.X, padx=2, pady=2)
        
        self.motor1_status_var = tk.StringVar(value="Motor 1: Stopped")
        self.motor2_status_var = tk.StringVar(value="Motor 2: Stopped")
        
        ttk.Label(motor_status_frame, textvariable=self.motor1_status_var).pack(anchor=tk.W, padx=5, pady=1)
        ttk.Label(motor_status_frame, textvariable=self.motor2_status_var).pack(anchor=tk.W, padx=5, pady=1)
        
        # Graph plotting - optimize for smaller screen
        plot_frame = ttk.LabelFrame(right_frame, text="Sensor Plot (Last 30s)")
        plot_frame.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # Create matplotlib figure optimized for small screen
        self.fig = Figure(figsize=(4, 3), dpi=80)
        self.ax = self.fig.add_subplot(111)
        self.fig.subplots_adjust(left=0.15, bottom=0.15, right=0.95, top=0.9)  # Optimize margins
        self.ax.set_xlabel('Time (s)', fontsize=8)
        self.ax.set_ylabel('Sensor Values', fontsize=8)
        self.ax.tick_params(axis='both', which='major', labelsize=7)
        self.ax.grid(True)
        
        # Create line objects (empty at first)
        self.thrust_line, = self.ax.plot([], [], 'r-', label='Thrust')
        self.lift_line, = self.ax.plot([], [], 'g-', label='Lift')
        self.moment_line, = self.ax.plot([], [], 'b-', label='Moment')
        self.ax.legend(prop={'size': 7}, loc='upper right')
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Simpler toolbar with just essential buttons
        toolbar = NavigationToolbar2Tk(self.canvas, plot_frame)
        toolbar.update()
        for child in toolbar.winfo_children():
            if child.winfo_class() == 'Button':
                child.configure(width=10, padx=1, pady=1)
        
        # Data storage for plotting
        self.plot_times = []
        self.plot_thrust = []
        self.plot_lift = []
        self.plot_moment = []
    
    def setup_files_tab(self):
        """Set up the file management tab"""
        # File list frame
        file_list_frame = ttk.LabelFrame(self.files_tab, text="Test Data Files")
        file_list_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Create treeview for file list
        columns = ("filename", "size", "modified")
        self.file_tree = ttk.Treeview(file_list_frame, columns=columns, show="headings")
        
        # Configure column headings
        self.file_tree.heading("filename", text="Filename")
        self.file_tree.heading("size", text="Size (KB)")
        self.file_tree.heading("modified", text="Modified")
        
        # Configure column widths
        self.file_tree.column("filename", width=250)
        self.file_tree.column("size", width=100, anchor="center")
        self.file_tree.column("modified", width=150, anchor="center")
        
        # Add scrollbar
        file_scroll = ttk.Scrollbar(file_list_frame, orient="vertical", command=self.file_tree.yview)
        self.file_tree.configure(yscrollcommand=file_scroll.set)
        
        # Position the treeview and scrollbar
        self.file_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        file_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        # File actions frame
        file_actions_frame = ttk.Frame(self.files_tab)
        file_actions_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Action buttons
        ttk.Button(file_actions_frame, text="Refresh List", 
                  command=self.refresh_file_list).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(file_actions_frame, text="View File", 
                  command=self.view_file).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(file_actions_frame, text="Export to USB", 
                  command=self.export_file).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(file_actions_frame, text="Delete File", 
                  command=self.delete_file).pack(side=tk.LEFT, padx=5, pady=5)
        
        # File preview frame
        preview_frame = ttk.LabelFrame(self.files_tab, text="File Preview")
        preview_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Text area for file preview
        self.preview_text = scrolledtext.ScrolledText(preview_frame, wrap=tk.WORD, height=10)
        self.preview_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Initial file list population
        self.refresh_file_list()
        
    def setup_pinout_tab(self):
        """Set up the pinout visualization tab with more compact layout"""
        # Create a sub-notebook to organize pinout information
        pin_notebook = ttk.Notebook(self.pinout_tab)
        pin_notebook.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # Create sub-tabs
        motor_pin_tab = ttk.Frame(pin_notebook)
        sensor_pin_tab = ttk.Frame(pin_notebook)
        calib_tab = ttk.Frame(pin_notebook)
        ref_tab = ttk.Frame(pin_notebook)
        
        pin_notebook.add(motor_pin_tab, text="Motors")
        pin_notebook.add(sensor_pin_tab, text="Sensors")
        pin_notebook.add(calib_tab, text="Calibration")
        pin_notebook.add(ref_tab, text="Reference")
        
        # Motor Pins tab - more compact
        motor_frame = ttk.Frame(motor_pin_tab)
        motor_frame.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # Headers
        ttk.Label(motor_frame, text="Motor", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=0, padx=2, pady=2)
        ttk.Label(motor_frame, text="Pin", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=1, padx=2, pady=2)
        ttk.Label(motor_frame, text="BCM#", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=2, padx=2, pady=2)
        ttk.Label(motor_frame, text="Action", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=3, padx=2, pady=2)
        
        # Motor 1 pins - more compact
        ttk.Label(motor_frame, text="Motor 1").grid(row=1, column=0, padx=2, pady=2)
        ttk.Label(motor_frame, text="IN1").grid(row=1, column=1, padx=2, pady=2)
        self.motor1_in1_var = tk.IntVar(value=self.system.MOTOR_PINS['motor1']['in1'])
        ttk.Entry(motor_frame, textvariable=self.motor1_in1_var, width=4).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(motor_frame, text="Update", 
                  command=lambda: self.update_pin("motor", "motor1", "in1"), width=6).grid(row=1, column=3, padx=2, pady=2)
        
        ttk.Label(motor_frame, text="Motor 1").grid(row=2, column=0, padx=2, pady=2)
        ttk.Label(motor_frame, text="IN2").grid(row=2, column=1, padx=2, pady=2)
        self.motor1_in2_var = tk.IntVar(value=self.system.MOTOR_PINS['motor1']['in2'])
        ttk.Entry(motor_frame, textvariable=self.motor1_in2_var, width=4).grid(row=2, column=2, padx=2, pady=2)
        ttk.Button(motor_frame, text="Update", 
                  command=lambda: self.update_pin("motor", "motor1", "in2"), width=6).grid(row=2, column=3, padx=2, pady=2)
        
        # Motor 2 pins - more compact
        ttk.Label(motor_frame, text="Motor 2").grid(row=3, column=0, padx=2, pady=2)
        ttk.Label(motor_frame, text="IN1").grid(row=3, column=1, padx=2, pady=2)
        self.motor2_in1_var = tk.IntVar(value=self.system.MOTOR_PINS['motor2']['in1'])
        ttk.Entry(motor_frame, textvariable=self.motor2_in1_var, width=4).grid(row=3, column=2, padx=2, pady=2)
        ttk.Button(motor_frame, text="Update", 
                  command=lambda: self.update_pin("motor", "motor2", "in1"), width=6).grid(row=3, column=3, padx=2, pady=2)
        
        ttk.Label(motor_frame, text="Motor 2").grid(row=4, column=0, padx=2, pady=2)
        ttk.Label(motor_frame, text="IN2").grid(row=4, column=1, padx=2, pady=2)
        self.motor2_in2_var = tk.IntVar(value=self.system.MOTOR_PINS['motor2']['in2'])
        ttk.Entry(motor_frame, textvariable=self.motor2_in2_var, width=4).grid(row=4, column=2, padx=2, pady=2)
        ttk.Button(motor_frame, text="Update", 
                  command=lambda: self.update_pin("motor", "motor2", "in2"), width=6).grid(row=4, column=3, padx=2, pady=2)
        
        # Load Cell Pins tab - more compact
        loadcell_frame = ttk.Frame(sensor_pin_tab)
        loadcell_frame.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # Headers
        ttk.Label(loadcell_frame, text="Sensor", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=0, padx=2, pady=2)
        ttk.Label(loadcell_frame, text="Pin", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=1, padx=2, pady=2)
        ttk.Label(loadcell_frame, text="BCM#", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=2, padx=2, pady=2)
        ttk.Label(loadcell_frame, text="Action", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=3, padx=2, pady=2)
        
        # Load Cell 1 pins
        ttk.Label(loadcell_frame, text="Sensor 1").grid(row=1, column=0, padx=2, pady=2)
        ttk.Label(loadcell_frame, text="DOUT").grid(row=1, column=1, padx=2, pady=2)
        self.loadcell1_dout_var = tk.IntVar(value=self.system.LOADCELL_PINS['loadcell1']['dout'])
        ttk.Entry(loadcell_frame, textvariable=self.loadcell1_dout_var, width=4).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(loadcell_frame, text="Update", 
                  command=lambda: self.update_pin("loadcell", "loadcell1", "dout"), width=6).grid(row=1, column=3, padx=2, pady=2)
        
        ttk.Label(loadcell_frame, text="Sensor 1").grid(row=2, column=0, padx=2, pady=2)
        ttk.Label(loadcell_frame, text="SCK").grid(row=2, column=1, padx=2, pady=2)
        self.loadcell1_sck_var = tk.IntVar(value=self.system.LOADCELL_PINS['loadcell1']['sck'])
        ttk.Entry(loadcell_frame, textvariable=self.loadcell1_sck_var, width=4).grid(row=2, column=2, padx=2, pady=2)
        ttk.Button(loadcell_frame, text="Update", 
                  command=lambda: self.update_pin("loadcell", "loadcell1", "sck"), width=6).grid(row=2, column=3, padx=2, pady=2)
        
        # Load Cell 2 pins
        ttk.Label(loadcell_frame, text="Sensor 2").grid(row=3, column=0, padx=2, pady=2)
        ttk.Label(loadcell_frame, text="DOUT").grid(row=3, column=1, padx=2, pady=2)
        self.loadcell2_dout_var = tk.IntVar(value=self.system.LOADCELL_PINS['loadcell2']['dout'])
        ttk.Entry(loadcell_frame, textvariable=self.loadcell2_dout_var, width=4).grid(row=3, column=2, padx=2, pady=2)
        ttk.Button(loadcell_frame, text="Update", 
                  command=lambda: self.update_pin("loadcell", "loadcell2", "dout"), width=6).grid(row=3, column=3, padx=2, pady=2)
        
        ttk.Label(loadcell_frame, text="Sensor 2").grid(row=4, column=0, padx=2, pady=2)
        ttk.Label(loadcell_frame, text="SCK").grid(row=4, column=1, padx=2, pady=2)
        self.loadcell2_sck_var = tk.IntVar(value=self.system.LOADCELL_PINS['loadcell2']['sck'])
        ttk.Entry(loadcell_frame, textvariable=self.loadcell2_sck_var, width=4).grid(row=4, column=2, padx=2, pady=2)
        ttk.Button(loadcell_frame, text="Update", 
                  command=lambda: self.update_pin("loadcell", "loadcell2", "sck"), width=6).grid(row=4, column=3, padx=2, pady=2)
        
        # Load Cell 3 pins
        ttk.Label(loadcell_frame, text="Sensor 3").grid(row=5, column=0, padx=2, pady=2)
        ttk.Label(loadcell_frame, text="DOUT").grid(row=5, column=1, padx=2, pady=2)
        self.loadcell3_dout_var = tk.IntVar(value=self.system.LOADCELL_PINS['loadcell3']['dout'])
        ttk.Entry(loadcell_frame, textvariable=self.loadcell3_dout_var, width=4).grid(row=5, column=2, padx=2, pady=2)
        ttk.Button(loadcell_frame, text="Update", 
                  command=lambda: self.update_pin("loadcell", "loadcell3", "dout"), width=6).grid(row=5, column=3, padx=2, pady=2)
        
        ttk.Label(loadcell_frame, text="Sensor 3").grid(row=6, column=0, padx=2, pady=2)
        ttk.Label(loadcell_frame, text="SCK").grid(row=6, column=1, padx=2, pady=2)
        self.loadcell3_sck_var = tk.IntVar(value=self.system.LOADCELL_PINS['loadcell3']['sck'])
        ttk.Entry(loadcell_frame, textvariable=self.loadcell3_sck_var, width=4).grid(row=6, column=2, padx=2, pady=2)
        ttk.Button(loadcell_frame, text="Update", 
                  command=lambda: self.update_pin("loadcell", "loadcell3", "sck"), width=6).grid(row=6, column=3, padx=2, pady=2)
        
        # Calibration Factors tab
        calib_frame = ttk.Frame(calib_tab)
        calib_frame.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # Headers
        ttk.Label(calib_frame, text="Sensor", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=0, padx=2, pady=2)
        ttk.Label(calib_frame, text="Calibration Factor", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=1, padx=2, pady=2)
        ttk.Label(calib_frame, text="Action", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=2, padx=2, pady=2)
        
        # Calibration factors
        ttk.Label(calib_frame, text="Sensor 1").grid(row=1, column=0, padx=2, pady=2)
        self.calib1_var = tk.DoubleVar(value=self.system.CALIBRATION_FACTORS['loadcell1'])
        ttk.Entry(calib_frame, textvariable=self.calib1_var, width=9).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(calib_frame, text="Update", 
                  command=lambda: self.update_calibration("loadcell1"), width=6).grid(row=1, column=2, padx=2, pady=2)
        
        ttk.Label(calib_frame, text="Sensor 2").grid(row=2, column=0, padx=2, pady=2)
        self.calib2_var = tk.DoubleVar(value=self.system.CALIBRATION_FACTORS['loadcell2'])
        ttk.Entry(calib_frame, textvariable=self.calib2_var, width=9).grid(row=2, column=1, padx=2, pady=2)
        ttk.Button(calib_frame, text="Update", 
                  command=lambda: self.update_calibration("loadcell2"), width=6).grid(row=2, column=2, padx=2, pady=2)
        
        ttk.Label(calib_frame, text="Sensor 3").grid(row=3, column=0, padx=2, pady=2)
        self.calib3_var = tk.DoubleVar(value=self.system.CALIBRATION_FACTORS['loadcell3'])
        ttk.Entry(calib_frame, textvariable=self.calib3_var, width=9).grid(row=3, column=1, padx=2, pady=2)
        ttk.Button(calib_frame, text="Update", 
                  command=lambda: self.update_calibration("loadcell3"), width=6).grid(row=3, column=2, padx=2, pady=2)
        
        # Add calibration description
        calib_desc = ttk.Label(calib_frame, text="Note: Calibration factors convert raw sensor readings to weight units.",
                            wraplength=300, justify=tk.LEFT, font=("TkDefaultFont", 8))
        calib_desc.grid(row=4, column=0, columnspan=3, padx=2, pady=5, sticky=tk.W)
        
        # Raspberry Pi pinout reference in a scrollable text widget
        ref_frame = ttk.Frame(ref_tab)
        ref_frame.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # Create a simple text representation of Raspberry Pi GPIO pinout
        pinout_text = """
        Raspberry Pi GPIO Pinout Reference (BCM numbering):
        
        3.3V   1 | 2   5V
        GPIO2  3 | 4   5V
        GPIO3  5 | 6   GND
        GPIO4  7 | 8   GPIO14
        GND    9 | 10  GPIO15
        GPIO17 11| 12  GPIO18
        GPIO27 13| 14  GND
        GPIO22 15| 16  GPIO23
        3.3V   17| 18  GPIO24
        GPIO10 19| 20  GND
        GPIO9  21| 22  GPIO25
        GPIO11 23| 24  GPIO8
        GND    25| 26  GPIO7
        GPIO0  27| 28  GPIO1
        GPIO5  29| 30  GND
        GPIO6  31| 32  GPIO12
        GPIO13 33| 34  GND
        GPIO19 35| 36  GPIO16
        GPIO26 37| 38  GPIO20
        GND    39| 40  GPIO21
        """
        
        # Create scrollable text widget for pinout reference
        pinout_ref = scrolledtext.ScrolledText(ref_frame, wrap=tk.WORD, height=15, width=35, font=("TkDefaultFont", 9))
        pinout_ref.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        pinout_ref.insert(tk.END, pinout_text)
        pinout_ref.configure(state='disabled')  # Make read-only
        
        # Status message at the bottom of the pinout tab
        self.pinout_status_var = tk.StringVar(value="Ready to update pin configuration")
        status_label = ttk.Label(self.pinout_tab, textvariable=self.pinout_status_var, 
                           font=("TkDefaultFont", 9), foreground="blue")
        status_label.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)
    
    def setup_settings_tab(self):
        """Set up the settings tab"""
        # Sampling rates frame
        rates_frame = ttk.LabelFrame(self.settings_tab, text="Sampling Rates")
        rates_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        ttk.Label(rates_frame, text="Sensor sampling rate (Hz):").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.sensor_rate_var = tk.IntVar(value=self.system.SENSOR_SAMPLE_RATE_HZ)
        sensor_rate_spinbox = ttk.Spinbox(rates_frame, from_=1, to=50, textvariable=self.sensor_rate_var, width=5)
        sensor_rate_spinbox.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        
        ttk.Label(rates_frame, text="Motor update rate (Hz):").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.motor_rate_var = tk.IntVar(value=self.system.MOTOR_UPDATE_RATE_HZ)
        motor_rate_spinbox = ttk.Spinbox(rates_frame, from_=1, to=100, textvariable=self.motor_rate_var, width=5)
        motor_rate_spinbox.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)
        
        ttk.Button(rates_frame, text="Apply Rate Changes", 
                 command=self.apply_rate_changes).grid(row=2, column=0, columnspan=2, padx=5, pady=5)
        
        # Lever arm configuration
        lever_frame = ttk.LabelFrame(self.settings_tab, text="Physics Parameters")
        lever_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        ttk.Label(lever_frame, text="Lever arm length (meters):").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.lever_arm_var = tk.DoubleVar(value=self.system.LEVER_ARM_LENGTH)
        lever_arm_entry = ttk.Entry(lever_frame, textvariable=self.lever_arm_var, width=10)
        lever_arm_entry.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        
        ttk.Button(lever_frame, text="Apply", 
                 command=self.apply_lever_arm_change).grid(row=1, column=0, columnspan=2, padx=5, pady=5)
        
        # Display settings
        display_frame = ttk.LabelFrame(self.settings_tab, text="Display Settings")
        display_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.fullscreen_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(display_frame, text="Fullscreen Mode", 
                       variable=self.fullscreen_var, 
                       command=self.toggle_fullscreen).pack(padx=10, pady=5, anchor=tk.W)
        # System process controls
        process_frame = ttk.LabelFrame(self.settings_tab, text="System Processes")
        process_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(process_frame, text="Start Processes", 
                  command=self.start_system_processes, width=15).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(process_frame, text="Stop Processes", 
                  command=self.stop_system_processes, width=15).pack(side=tk.LEFT, padx=5, pady=5)
        # System control buttons
        control_frame = ttk.LabelFrame(self.settings_tab, text="System Control")
        control_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        ttk.Button(control_frame, text="Shutdown System", 
                 command=self.on_close).pack(side=tk.LEFT, padx=20, pady=10)
        ttk.Button(control_frame, text="Reset System", 
                 command=self.reset_system).pack(side=tk.LEFT, padx=20, pady=10)
        
    def update_data_display(self):
        """Update the data display with the latest sensor readings"""
        try:
            # Get the latest sensor data
            with self.system.data_lock:
                data = self.system.latest_sensor_data
            
            print(f"Update display with data: {data['forces']}")
            forces = data["forces"]
            raw_readings = forces["raw_readings"]
            cycle_info = data["cycle_info"]
            
            # Update sensor values display
            if forces["thrust"] is not None:
                self.thrust_var.set(f"Thrust: {forces['thrust']:.2f}")
            else:
                self.thrust_var.set("Thrust: N/A")
                
            if forces["lift"] is not None:
                self.lift_var.set(f"Lift: {forces['lift']:.2f}")
            else:
                self.lift_var.set("Lift: N/A")
                
            if forces["moment"] is not None:
                self.moment_var.set(f"Moment: {forces['moment']:.2f}")
            else:
                self.moment_var.set("Moment: N/A")
            
            # Update raw sensor data display
            if len(raw_readings) > 0 and raw_readings[0] is not None:
                self.raw1_var.set(f"Sensor 1: {raw_readings[0]:.2f}")
            else:
                self.raw1_var.set("Sensor 1: N/A")
                
            if len(raw_readings) > 1 and raw_readings[1] is not None:
                self.raw2_var.set(f"Sensor 2: {raw_readings[1]:.2f}")
            else:
                self.raw2_var.set("Sensor 2: N/A")
                
            if len(raw_readings) > 2 and raw_readings[2] is not None:
                self.raw3_var.set(f"Sensor 3: {raw_readings[2]:.2f}")
            else:
                self.raw3_var.set("Sensor 3: N/A")
            
            # Update cycle information display
            if hasattr(self, 'cycle_info_var'):
                if cycle_info['pattern_active']:
                    self.cycle_info_var.set(
                        f"Cycle: {cycle_info['cycle_number']} | "
                        f"Position: {cycle_info['cycle_position']:.2f}"
                    )
                else:
                    self.cycle_info_var.set("Cycle: Not active")
            
            # Update motor status display
            motor1 = self.system.motor_state['motor1']
            motor2 = self.system.motor_state['motor2']
            
            if motor1['direction'] == 0 or motor1['speed'] == 0:
                self.motor1_status_var.set("Motor 1: Stopped")
            elif motor1['direction'] > 0:
                self.motor1_status_var.set(f"Motor 1: Forward ({motor1['speed']}%)")
            else:
                self.motor1_status_var.set(f"Motor 1: Reverse ({motor1['speed']}%)")
                
            if motor2['direction'] == 0 or motor2['speed'] == 0:
                self.motor2_status_var.set("Motor 2: Stopped")
            elif motor2['direction'] > 0:
                self.motor2_status_var.set(f"Motor 2: Forward ({motor2['speed']}%)")
            else:
                self.motor2_status_var.set(f"Motor 2: Reverse ({motor2['speed']}%)")
            
            # Update plot data
            current_time = time.time()
            
            # Add new data point
            if forces["thrust"] is not None and forces["lift"] is not None and forces["moment"] is not None:
                self.plot_times.append(current_time)
                self.plot_thrust.append(forces["thrust"])
                self.plot_lift.append(forces["lift"])
                self.plot_moment.append(forces["moment"])
                
                # Keep only last 30 seconds of data
                cutoff_time = current_time - 30
                while self.plot_times and self.plot_times[0] < cutoff_time:
                    self.plot_times.pop(0)
                    self.plot_thrust.pop(0)
                    self.plot_lift.pop(0)
                    self.plot_moment.pop(0)
                
                # Update plot if we have data
                if self.plot_times:
                    # Normalize times to seconds ago
                    times_norm = [t - self.plot_times[0] for t in self.plot_times]
                    
                    # Update data
                    self.thrust_line.set_data(times_norm, self.plot_thrust)
                    self.lift_line.set_data(times_norm, self.plot_lift)
                    self.moment_line.set_data(times_norm, self.plot_moment)
                    
                    # Adjust axes limits
                    self.ax.set_xlim(0, times_norm[-1] + 1)
                    
                    min_val = min(min(self.plot_thrust), min(self.plot_lift), min(self.plot_moment))
                    max_val = max(max(self.plot_thrust), max(self.plot_lift), max(self.plot_moment))
                    y_margin = (max_val - min_val) * 0.1 or 1.0  # Add 10% margin or 1.0 if flat
                    self.ax.set_ylim(min_val - y_margin, max_val + y_margin)
                    
                    # Redraw
                    self.canvas.draw()
        
        except Exception as e:
            print(f"Error updating data display: {e}")
        
        # Reschedule
        self.root.after(100, self.update_data_display)
    
    def update_pin(self, pin_type, device, pin_name):
        """Update a pin configuration"""
        # Get the appropriate variable based on type, device, and pin name
        var_name = f"{device}_{pin_name}_var"
        if hasattr(self, var_name):
            # Get the new value from the entry widget
            new_value = getattr(self, var_name).get()
            
            # Update the pin in the system
            success, message = self.system.update_pin_configuration(pin_type, device, pin_name, new_value)
            
            # Update status message
            if success:
                self.pinout_status_var.set(message)
            else:
                messagebox.showerror("Pin Update Error", message)
                # Reset the entry to the current value in the system
                current_value = self.system.MOTOR_PINS[device][pin_name] if pin_type == "motor" else self.system.LOADCELL_PINS[device][pin_name]
                getattr(self, var_name).set(current_value)
    
    def update_calibration(self, device):
        """Update a calibration factor"""
        # Get the appropriate variable
        var_name = f"calib{device[-1]}_var"  # e.g., calib1_var for loadcell1
        if hasattr(self, var_name):
            # Get the new value
            new_value = getattr(self, var_name).get()
            
            # Update the calibration factor
            success, message = self.system.update_calibration_factor(device, new_value)
            
            # Update status message
            if success:
                self.pinout_status_var.set(message)
            else:
                messagebox.showerror("Calibration Update Error", message)
                # Reset the entry to the current value
                current_value = self.system.CALIBRATION_FACTORS[device]
                getattr(self, var_name).set(current_value)
    
    def apply_motor1(self):
        """Apply settings to motor 1"""
        direction = self.motor1_dir_var.get()
        speed = self.motor1_speed_var.get()
        self.system.set_motor("motor1", direction, speed)
        self.status_var.set(f"Motor 1 set to dir={direction}, speed={speed}%")
    
    def apply_motor2(self):
        """Apply settings to motor 2"""
        direction = self.motor2_dir_var.get()
        speed = self.motor2_speed_var.get()
        self.system.set_motor("motor2", direction, speed)
        self.status_var.set(f"Motor 2 set to dir={direction}, speed={speed}%")
    
    def set_both_motors(self, direction, speed):
        """Set both motors to the same direction and speed"""
        self.system.set_motor("both", direction, speed)
        self.status_var.set(f"Both motors set to dir={direction}, speed={speed}%")
        
        # Update GUI controls to match
        self.motor1_dir_var.set(direction)
        self.motor1_speed_var.set(speed)
        self.motor2_dir_var.set(direction)
        self.motor2_speed_var.set(speed)
    
    def stop_all_motors(self):
        """Stop all motors"""
        self.system.set_motor("both", 0, 0)
        self.status_var.set("All motors stopped")
        
        # Update GUI controls to match
        self.motor1_dir_var.set(0)
        self.motor1_speed_var.set(0)
        self.motor2_dir_var.set(0)
        self.motor2_speed_var.set(0)
    
    def run_test_sequence(self):
        """Run the test sequence"""
        self.start_system_processes
        self.status_var.set("Running test sequence...")
        self.system.run_test_sequence(callback=self.update_status)
    
    def update_status(self, message):
        """Update the status display"""
        self.status_var.set(message)
        
        
    def apply_rate_changes(self):
        """Apply sampling rate changes"""
        self.system.SENSOR_SAMPLE_RATE_HZ = self.sensor_rate_var.get()
        self.system.MOTOR_UPDATE_RATE_HZ = self.motor_rate_var.get()
        self.status_var.set("Sampling rates updated")
        
    def apply_lever_arm_change(self):
        """Apply lever arm length change"""
        try:
            length = float(self.lever_arm_var.get())
            if length <= 0:
                messagebox.showerror("Invalid Value", "Lever arm length must be positive")
                return
            
            self.system.LEVER_ARM_LENGTH = length
            self.status_var.set(f"Lever arm length set to {length} meters")
        except ValueError:
            messagebox.showerror("Invalid Value", "Please enter a valid number")
    
    def refresh_file_list(self):
        """Refresh the file list in the treeview"""
        # Clear existing items
        for item in self.file_tree.get_children():
            self.file_tree.delete(item)
        
        # Get updated file list
        files = self.system.get_log_files()
        
        # Add files to treeview
        for file_info in files:
            self.file_tree.insert("", "end", values=(
                file_info['filename'],
                f"{file_info['size'] / 1024:.1f}",
                file_info['modified']
            ))
        
        # Clear preview
        self.preview_text.delete(1.0, tk.END)
        self.preview_text.insert(tk.END, "Select a file to preview its contents.")
    
    def view_file(self):
        """View selected file contents"""
        selected = self.file_tree.selection()
        if not selected:
            messagebox.showinfo("Info", "Please select a file to view")
            return
            
        # Get filename from selected item
        item = self.file_tree.item(selected[0])
        filename = item['values'][0]
        
        # Load file contents
        try:
            filepath = os.path.join(self.system.data_directory, filename)
            
            # Read first 100 lines for preview
            with open(filepath, 'r') as file:
                lines = [next(file) for _ in range(100) if file]
                
            # Show in preview
            self.preview_text.delete(1.0, tk.END)
            self.preview_text.insert(tk.END, f"Preview of '{filename}':\n\n")
            self.preview_text.insert(tk.END, ''.join(lines))
            
            if len(lines) == 100:
                self.preview_text.insert(tk.END, "\n\n[Showing first 100 lines only...]")
                
        except Exception as e:
            self.preview_text.delete(1.0, tk.END)
            self.preview_text.insert(tk.END, f"Error reading file: {e}")
    
    def export_file(self):
        """Export selected file to USB drive"""
        selected = self.file_tree.selection()
        if not selected:
            messagebox.showinfo("Info", "Please select a file to export")
            return
            
        # Get filename from selected item
        item = self.file_tree.item(selected[0])
        filename = item['values'][0]
        source_path = os.path.join(self.system.data_directory, filename)
        
        # Find USB drives
        usb_paths = self.find_usb_drives()
        
        if not usb_paths:
            messagebox.showinfo("No USB Found", "No USB drives detected. Please insert a USB drive.")
            return
            
        # If multiple drives found, let user choose
        target_path = usb_paths[0]
        if len(usb_paths) > 1:
            # This is a simplified approach - in a real system you might want a proper selection dialog
            choice = messagebox.askyesno("Multiple USB Drives", 
                                       f"Found {len(usb_paths)} USB drives. Use {target_path}?")
            if not choice:
                return
        
        try:
            # Copy file
            target_file = os.path.join(target_path, filename)
            import shutil
            shutil.copy2(source_path, target_file)
            messagebox.showinfo("Export Successful", f"File exported to {target_file}")
        except Exception as e:
            messagebox.showerror("Export Failed", f"Error exporting file: {e}")
            
    def find_usb_drives(self):
        """Find mounted USB drives"""
        # This is a simplified approach for Linux/Raspberry Pi
        # Checks common mount points for USB drives
        usb_paths = []
        
        # Common USB mount points on Raspberry Pi
        potential_paths = [
            "/media/pi",  # Raspberry Pi default
            "/media",     # General Linux
            "/mnt",       # Alternative mount location
        ]
        
        for base_path in potential_paths:
            if os.path.exists(base_path):
                # Look for mounted drives in these locations
                for item in os.listdir(base_path):
                    full_path = os.path.join(base_path, item)
                    if os.path.ismount(full_path) and os.access(full_path, os.W_OK):
                        usb_paths.append(full_path)
        
        return usb_paths
    
    def delete_file(self):
        """Delete selected file"""
        selected = self.file_tree.selection()
        if not selected:
            messagebox.showinfo("Info", "Please select a file to delete")
            return
            
        # Get filename from selected item
        item = self.file_tree.item(selected[0])
        filename = item['values'][0]
        
        # Confirm deletion
        if messagebox.askyesno("Confirm Delete", f"Are you sure you want to delete {filename}?"):
            success = self.system.delete_log_file(filename)
            
            if success:
                messagebox.showinfo("Success", f"File '{filename}' deleted successfully")
                # Refresh file list
                self.refresh_file_list()
                # Update log filename display if it was the current file
                self.log_file_var.set(f"Log file: {os.path.basename(self.system.log_filename)}")
            else:
                messagebox.showerror("Error", f"Failed to delete '{filename}'")
    
    def set_test_name(self):
        """Set the test name and create a new log file"""
        test_name = self.test_name_var.get().strip()
        if not test_name:
            messagebox.showinfo("Info", "Please enter a test name")
            return
            
        # Update test name and create new log file
        new_log_file = self.system.set_test_name(test_name)
        
        # Update display
        self.log_file_var.set(f"Log file: {os.path.basename(new_log_file)}")
        self.status_var.set(f"Test name set to '{test_name}'")
        
        # Refresh file list if files tab is active
        if self.notebook.index(self.notebook.select()) == 2:  # Files tab index
            self.refresh_file_list()
    
    def toggle_fullscreen(self):
        """Toggle fullscreen mode"""
        is_fullscreen = self.fullscreen_var.get()
        self.root.attributes('-fullscreen', is_fullscreen)
        if is_fullscreen:
            self.status_var.set("Entered fullscreen mode")
        else:
            self.status_var.set("Exited fullscreen mode")
    
    def apply_wave_parameters(self):
        """Apply the wave pattern parameters"""
        try:
            # Validate period
            period = float(self.period_var.get())
            if period <= 0:
                messagebox.showerror("Invalid Value", "Period must be greater than 0")
                return
                
            # Validate phase
            phase = float(self.phase_var.get())
            if phase < 0 or phase >= period/2:
                messagebox.showerror("Invalid Value", f"Phase must be between 0 and {period/2}")
                return
                
            # Validate latency
            latency = float(self.latency_var.get())
            if latency < 0 or latency >= period/4:
                messagebox.showerror("Invalid Value", f"Latency must be between 0 and {period/4}")
                return
                
            # Validate motor speed
            speed = int(self.speed_var.get())
            if speed <= 0 or speed > 100:
                messagebox.showerror("Invalid Value", "Speed must be between 1 and 100")
                return
                
            # Apply parameters
            self.system.period = period
            self.system.phase = phase
            self.system.latency = latency
            self.system.motor_speed = speed
            self.system.reverse = self.reverse_var.get()
            
            self.status_var.set(f"Wave parameters applied: Period={period}s, Phase={phase}s, Latency={latency}s")
            
        except ValueError as e:
            messagebox.showerror("Invalid Input", f"Please enter valid numbers: {e}")
    
            
    def reset_system(self):
        """Reset the system"""
        # Stop motors
        self.stop_all_motors()
        
        # Confirm reset
        if messagebox.askyesno("Reset Confirmation", "Are you sure you want to reset the system?"):
            # Create new log file
            self.system.log_filename = f"sensor_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            with open(self.system.log_filename, 'w') as log_file:
                log_file.write("timestamp,thrust,lift,moment,raw1,raw2,raw3,motor1_speed,motor1_dir,motor2_speed,motor2_dir\n")
            
            # Update log filename display
            self.log_filename_var.set(f"Log file: {self.system.log_filename}")
            
            # Clear plot data
            self.plot_times = []
            self.plot_thrust = []
            self.plot_lift = []
            self.plot_moment = []
            
            self.status_var.set("System reset complete")
            
    def on_close(self):
        """Handle window close event"""
        if messagebox.askyesno("Quit", "Are you sure you want to quit?"):
            self.status_var.set("Shutting down...")
            self.root.update()
            
            # Stop all motors and clean up
            self.system.set_motor("both", 0, 0)
            self.system.cleanup()
            
            # Terminate processes
            self.stop_system_processes()
                
            # Exit application
            self.root.destroy()


def main():
    """Main function"""
    # Catch SIGINT (Ctrl+C) signal for graceful shutdown
    def signal_handler(sig, frame):
        print("\nShutting down gracefully...")
        root.quit()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
        
