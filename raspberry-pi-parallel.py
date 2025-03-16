#!/usr/bin/env python3
"""
Parallel Computing on Raspberry Pi for Sensor Data Collection and Motor Control
This program demonstrates how to use multiprocessing and threading to handle
sensor data collection and motor control in parallel on a single Raspberry Pi.
"""

import time
import signal
import sys
from multiprocessing import Process, Queue, Value
from ctypes import c_bool
import threading
import numpy as np

# For Raspberry Pi GPIO
try:
    import RPi.GPIO as GPIO
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    PI_AVAILABLE = True
except ImportError:
    print("Warning: Running in simulation mode (Raspberry Pi libraries not available)")
    PI_AVAILABLE = False

# Configuration
SENSOR_SAMPLE_RATE_HZ = 100  # How often to sample sensors (Hz)
MOTOR_UPDATE_RATE_HZ = 50    # How often to update motors (Hz)
PROCESS_RATE_HZ = 200        # How often to process data (Hz)

# Create a shared flag for graceful shutdown
running = Value(c_bool, True)

# Queues for inter-process communication
sensor_queue = Queue()      # Sensor data → Processing
command_queue = Queue()     # Processing → Motor control

def setup_gpio():
    """Set up GPIO pins and sensors"""
    if not PI_AVAILABLE:
        return None, None
    
    # Set up GPIO
    GPIO.setmode(GPIO.BCM)
    
    # Motor pins setup (example for two motors)
    motor_pins = {
        'motor1': {'pwm': 18, 'dir1': 23, 'dir2': 24},
        'motor2': {'pwm': 13, 'dir1': 27, 'dir2': 22}
    }
    
    for motor in motor_pins.values():
        GPIO.setup(motor['pwm'], GPIO.OUT)
        GPIO.setup(motor['dir1'], GPIO.OUT)
        GPIO.setup(motor['dir2'], GPIO.OUT)
    
    # Set up PWM for motors
    pwm_motors = {
        'motor1': GPIO.PWM(motor_pins['motor1']['pwm'], 1000),  # 1000 Hz frequency
        'motor2': GPIO.PWM(motor_pins['motor2']['pwm'], 1000)
    }
    
    for pwm in pwm_motors.values():
        pwm.start(0)  # Start with 0% duty cycle
    
    # Set up I2C ADC for analog sensors
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    
    # Set up analog channels
    analog_channels = {
        'sensor1': AnalogIn(ads, ADS.P0),
        'sensor2': AnalogIn(ads, ADS.P1),
        'sensor3': AnalogIn(ads, ADS.P2)
    }
    
    return motor_pins, pwm_motors, analog_channels

def sensor_process(running, sensor_queue):
    """Process for sensor data acquisition"""
    print("Sensor process started")
    
    # For real Raspberry Pi
    if PI_AVAILABLE:
        _, _, analog_channels = setup_gpio()
        
        while running.value:
            # Read sensor data
            sensor_data = {
                name: {'voltage': channel.voltage, 'value': channel.value}
                for name, channel in analog_channels.items()
            }
            
            # Add timestamp
            sensor_data['timestamp'] = time.time()
            
            # Put data in the queue for processing
            sensor_queue.put(sensor_data)
            
            # Sleep to maintain sample rate
            time.sleep(1.0 / SENSOR_SAMPLE_RATE_HZ)
    else:
        # Simulation mode
        while running.value:
            # Simulate sensor data
            sensor_data = {
                'sensor1': {'voltage': np.random.random() * 3.3, 'value': int(np.random.random() * 65535)},
                'sensor2': {'voltage': np.random.random() * 3.3, 'value': int(np.random.random() * 65535)},
                'sensor3': {'voltage': np.random.random() * 3.3, 'value': int(np.random.random() * 65535)},
                'timestamp': time.time()
            }
            
            sensor_queue.put(sensor_data)
            time.sleep(1.0 / SENSOR_SAMPLE_RATE_HZ)
    
    print("Sensor process stopped")

def process_data(sensor_queue, command_queue, running):
    """Process for data processing and decision making"""
    print("Processing process started")
    
    # Buffer for sensor data
    data_buffer = []
    max_buffer_size = 100  # Adjust based on memory constraints
    
    # Initialize moving averages
    moving_avgs = {
        'sensor1': 0,
        'sensor2': 0,
        'sensor3': 0
    }
    
    alpha = 0.2  # Parameter for exponential moving average
    
    while running.value:
        # Check if there's new sensor data
        if not sensor_queue.empty():
            # Get sensor data from queue
            sensor_data = sensor_queue.get()
            
            # Add to buffer (optional - for more complex processing)
            data_buffer.append(sensor_data)
            if len(data_buffer) > max_buffer_size:
                data_buffer.pop(0)  # Remove oldest data
            
            # Update moving averages
            for sensor in ['sensor1', 'sensor2', 'sensor3']:
                if sensor in sensor_data:
                    moving_avgs[sensor] = alpha * sensor_data[sensor]['voltage'] + (1 - alpha) * moving_avgs[sensor]
            
            # Example: Simple threshold-based control
            # In real applications, you might have more complex control algorithms
            motor_commands = {}
            
            # Example control logic (modify based on your needs)
            if moving_avgs['sensor1'] > 2.0:
                motor_commands['motor1'] = {'speed': 70, 'direction': 1}  # Forward at 70% speed
            elif moving_avgs['sensor1'] < 1.0:
                motor_commands['motor1'] = {'speed': 50, 'direction': -1}  # Reverse at 50% speed
            else:
                motor_commands['motor1'] = {'speed': 0, 'direction': 0}  # Stop
            
            if moving_avgs['sensor2'] > moving_avgs['sensor3']:
                motor_commands['motor2'] = {'speed': 60, 'direction': 1}
            else:
                motor_commands['motor2'] = {'speed': 40, 'direction': -1}
            
            # Send commands to motor process
            command_queue.put(motor_commands)
        
        # Sleep to maintain processing rate
        time.sleep(1.0 / PROCESS_RATE_HZ)
    
    print("Processing process stopped")

def motor_control_process(command_queue, running):
    """Process for motor control"""
    print("Motor control process started")
    
    # For real Raspberry Pi
    if PI_AVAILABLE:
        motor_pins, pwm_motors, _ = setup_gpio()
        
        # Set initial motor states
        for motor_name, pwm in pwm_motors.items():
            pins = motor_pins[motor_name]
            GPIO.output(pins['dir1'], GPIO.LOW)
            GPIO.output(pins['dir2'], GPIO.LOW)
            pwm.ChangeDutyCycle(0)
        
        # Motor control thread function
        def motor_update_thread():
            last_commands = {
                'motor1': {'speed': 0, 'direction': 0},
                'motor2': {'speed': 0, 'direction': 0}
            }
            
            while running.value:
                # Check if there are new commands
                if not command_queue.empty():
                    motor_commands = command_queue.get()
                    
                    # Update last commands with new ones
                    for motor_name, command in motor_commands.items():
                        if motor_name in last_commands:
                            last_commands[motor_name] = command
                
                # Apply the commands to motors
                for motor_name, command in last_commands.items():
                    if motor_name in pwm_motors:
                        pins = motor_pins[motor_name]
                        speed = command['speed']  # 0-100
                        direction = command['direction']  # -1, 0, 1
                        
                        # Set direction pins
                        if direction > 0:
                            GPIO.output(pins['dir1'], GPIO.HIGH)
                            GPIO.output(pins['dir2'], GPIO.LOW)
                        elif direction < 0:
                            GPIO.output(pins['dir1'], GPIO.LOW)
                            GPIO.output(pins['dir2'], GPIO.HIGH)
                        else:
                            GPIO.output(pins['dir1'], GPIO.LOW)
                            GPIO.output(pins['dir2'], GPIO.LOW)
                        
                        # Set PWM duty cycle
                        pwm_motors[motor_name].ChangeDutyCycle(speed)
                
                # Sleep to maintain update rate
                time.sleep(1.0 / MOTOR_UPDATE_RATE_HZ)
        
        # Create and start the motor update thread
        motor_thread = threading.Thread(target=motor_update_thread)
        motor_thread.daemon = True
        motor_thread.start()
        
        # Main loop just keeps the process alive until running is False
        while running.value:
            time.sleep(0.1)
            
        # Clean up
        for pwm in pwm_motors.values():
            pwm.stop()
    else:
        # Simulation mode
        while running.value:
            if not command_queue.empty():
                motor_commands = command_queue.get()
                print(f"Motor commands: {motor_commands}")
            time.sleep(1.0 / MOTOR_UPDATE_RATE_HZ)
    
    print("Motor control process stopped")

def logging_process(sensor_queue, command_queue, running):
    """Optional process for logging data to file"""
    print("Logging process started")
    
    # Open log file
    with open(f"sensor_motor_log_{time.strftime('%Y%m%d_%H%M%S')}.csv", "w") as f:
        # Write header
        f.write("timestamp,sensor1_voltage,sensor2_voltage,sensor3_voltage,motor1_speed,motor1_dir,motor2_speed,motor2_dir\n")
        
        # Initialize data containers
        sensor_data = {
            'timestamp': 0,
            'sensor1': {'voltage': 0},
            'sensor2': {'voltage': 0},
            'sensor3': {'voltage': 0}
        }
        
        motor_commands = {
            'motor1': {'speed': 0, 'direction': 0},
            'motor2': {'speed': 0, 'direction': 0}
        }
        
        # Local queues to avoid disturbing the main data flow
        local_sensor_queue = Queue()
        local_command_queue = Queue()
        
        # Thread to copy sensor queue data to local queue
        def sensor_copy_thread():
            while running.value:
                if not sensor_queue.empty():
                    data = sensor_queue.get()
                    sensor_queue.put(data)  # Put it back
                    local_sensor_queue.put(data)
                time.sleep(0.01)
        
        # Thread to copy command queue data to local queue
        def command_copy_thread():
            while running.value:
                if not command_queue.empty():
                    data = command_queue.get()
                    command_queue.put(data)  # Put it back
                    local_command_queue.put(data)
                time.sleep(0.01)
        
        # Start copy threads
        threads = [
            threading.Thread(target=sensor_copy_thread),
            threading.Thread(target=command_copy_thread)
        ]
        
        for thread in threads:
            thread.daemon = True
            thread.start()
        
        # Main logging loop
        while running.value:
            # Update sensor data if available
            if not local_sensor_queue.empty():
                sensor_data = local_sensor_queue.get()
            
            # Update motor commands if available
            if not local_command_queue.empty():
                motor_commands = local_command_queue.get()
            
            # Write log entry
            log_entry = (
                f"{sensor_data['timestamp']},"
                f"{sensor_data['sensor1']['voltage']},"
                f"{sensor_data['sensor2']['voltage']},"
                f"{sensor_data['sensor3']['voltage']},"
                f"{motor_commands['motor1']['speed']},"
                f"{motor_commands['motor1']['direction']},"
                f"{motor_commands['motor2']['speed']},"
                f"{motor_commands['motor2']['direction']}\n"
            )
            
            f.write(log_entry)
            f.flush()  # Ensure data is written to disk
            
            time.sleep(0.1)  # Log at 10 Hz
    
    print("Logging process stopped")

def signal_handler(sig, frame):
    """Handle Ctrl+C to gracefully shut down all processes"""
    print("\nShutting down gracefully...")
    running.value = False
    time.sleep(1)  # Give processes time to shut down
    sys.exit(0)

def main():
    """Main function to start and manage all processes"""
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create processes
    processes = [
        Process(target=sensor_process, args=(running, sensor_queue)),
        Process(target=process_data, args=(sensor_queue, command_queue, running)),
        Process(target=motor_control_process, args=(command_queue, running))
    ]
    
    # Optional logging process
    # Uncomment the following line to enable logging
    # processes.append(Process(target=logging_process, args=(sensor_queue, command_queue, running)))
    
    # Start all processes
    print("Starting processes...")
    for p in processes:
        p.daemon = True  # Ensures processes terminate when main program exits
        p.start()
    
    print("System running. Press Ctrl+C to stop.")
    
    # Keep main process alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        # This should be caught by signal_handler, but just in case
        running.value = False
        
    # Wait for all processes to terminate
    for p in processes:
        p.join()
    
    # Clean up GPIO if on Raspberry Pi
    if PI_AVAILABLE:
        GPIO.cleanup()
    
    print("All processes terminated. System shutdown complete.")

if __name__ == "__main__":
    main()
