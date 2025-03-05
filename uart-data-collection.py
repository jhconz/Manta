from datetime import datetime
import numpy as np
import json
import time
import threading
import serial
from collections import deque
import RPi.GPIO as GPIO
from hx711 import HX711  # Assuming you're using a HX711 library

class SensorNode:
    def __init__(self, serial_port='/dev/ttyAMA0', baud_rate=115200):
        # Initialize UART communication
        self.ser = serial.Serial(
            port=serial_port,
            baudrate=baud_rate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        
        # Initialize sensors
        self.sensors = []
        self.sensor_pins = [(20, 21), (17, 16), (5, 6)]  # (dt, sck) pins for each HX711
        self.calib = []
        for dt_pin, sck_pin in self.sensor_pins:
            sensor = HX711(dt_pin, sck_pin)
            sensor.set_reading_format("MSB", "MSB")
            sensor.set_reference_unit(1)  # Calibration value
            sensor.reset()
            sensor.tare()
            self.sensors.append(sensor)
    
    def read_sensors(self):
        timestamp = datetime.now().isoformat()
        raw_readings = []
        
        for sensor in self.sensors:
            try:
                value = sensor.get_weight(1)
                raw_readings.append(value)
            except Exception as e:
                raw_readings.append(None)
                print(f"Sensor reading error: {e}")
        
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
            
            # In-plane moment is (sensor2 + sensor3) * lever arm length
            forces["moment"] = (raw_readings[1] + raw_readings[2]) * self.lever_arm_length
        
        data = {
            "timestamp": timestamp,
            "forces": forces
        }
        return data
    
    def listen_for_commands(self):
        """Listen for commands from master"""
        while True:
            if self.ser.in_waiting > 0:
                command = self.ser.readline().decode('utf-8').strip()
                if command == "READ":
                    # Read sensors and send data back
                    data = self.read_sensors()
                    json_data = json.dumps(data) + '\n'
                    self.ser.write(json_data.encode('utf-8'))
    
    def run(self):
        # Start listening for commands in a separate thread
        command_thread = threading.Thread(target=self.listen_for_commands)
        command_thread.daemon = True
        command_thread.start()
        
        # Main thread can now do other work or just wait
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.ser.close()
            print("UART connection closed")

class MotorNode:
    def __init__(self, serial_port='/dev/ttyAMA0', baud_rate=115200):
        # Initialize UART communication
        self.ser = serial.Serial(
            port=serial_port,
            baudrate=baud_rate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        
        # Motor control pins setup
        self.motor_pins = [(20, 21), (5, 6)]  # (in1, in2) for each H-bridge
        GPIO.setmode(GPIO.BCM)
        for in1, in2 in self.motor_pins:
            GPIO.setup(in1, GPIO.OUT)
            GPIO.setup(in2, GPIO.OUT)
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
            
        # For tracking executed commands
        self.last_execution_time = None
        self.last_params = None
    
    def execute_square_wave(self, params):
        """Execute square wave with given amplitude and phase"""
        motor_id = params['motor_id']
        amplitude = params['amplitude']
        phase = params['phase']
        
        in1, in2 = self.motor_pins[motor_id]
        
        # Record exact execution time
        execution_time = datetime.now().isoformat()
        self.last_execution_time = execution_time
        self.last_params = params
        
        if amplitude > 0:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
            
        return execution_time
    
    def listen_for_commands(self):
        """Listen for commands from master"""
        while True:
            if self.ser.in_waiting > 0:
                command_raw = self.ser.readline().decode('utf-8').strip()
                try:
                    if command_raw.startswith("{"):  # Check if it's a JSON command
                        command = json.loads(command_raw)
                        if "params" in command:
                            self.execute_square_wave(command["params"])
                            # Send back confirmation with execution timestamp
                            response = {
                                "status": "executed",
                                "timestamp": self.last_execution_time,
                                "params": self.last_params
                            }
                            self.ser.write((json.dumps(response) + '\n').encode('utf-8'))
                    elif command_raw == "STATUS":
                        # Send current status
                        response = {
                            "status": "ready",
                            "last_execution": self.last_execution_time,
                            "last_params": self.last_params
                        }
                        self.ser.write((json.dumps(response) + '\n').encode('utf-8'))
                except json.JSONDecodeError:
                    print(f"Invalid command format: {command_raw}")
                except Exception as e:
                    print(f"Error processing command: {e}")
    
    def run(self):
        # Start listening for commands in a separate thread
        command_thread = threading.Thread(target=self.listen_for_commands)
        command_thread.daemon = True
        command_thread.start()
        
        # Main thread can now do other work or just wait
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.ser.close()
            print("UART connection closed")

class MasterNode:
    def __init__(self, sensor_port='/dev/ttyUSB0', motor_port='/dev/ttyUSB1', baud_rate=115200):
        # Initialize UART communication with both nodes
        self.sensor_uart = serial.Serial(
            port=sensor_port,
            baudrate=baud_rate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        
        self.motor_uart = serial.Serial(
            port=motor_port, 
            baudrate=baud_rate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        
        # Data storage
        self.data_buffer = deque(maxlen=10000)  # Store last 10000 readings
    
    def read_sensor_data(self):
        """Request and read sensor data via UART"""
        try:
            # Clear any pending data
            self.sensor_uart.reset_input_buffer()
            
            # Send command to read sensors
            self.sensor_uart.write(b"READ\n")
            
            # Wait for and read response
            response = self.sensor_uart.readline().decode('utf-8').strip()
            sensor_data = json.loads(response)
            return sensor_data
        except Exception as e:
            print(f"Error reading from sensor node: {e}")
            return None
    
    def send_motor_command(self, motor_state):
        """Send motor commands via UART"""
        try:
            # Clear any pending data
            self.motor_uart.reset_input_buffer()
            
            # Send motor command
            command = json.dumps(motor_state) + '\n'
            self.motor_uart.write(command.encode('utf-8'))
            
            # Wait for confirmation
            response = self.motor_uart.readline().decode('utf-8').strip()
            execution_data = json.loads(response)
            return execution_data
        except Exception as e:
            print(f"Error sending to motor node: {e}")
            return None
    
    def save_data(self, sensor_data, motor_execution):
        """Save synchronized sensor and motor data"""
        combined_data = {
            "sensor_timestamp": sensor_data["timestamp"],
            "sensor_values": sensor_data["sensor_values"],
            "motor_timestamp": motor_execution["timestamp"],
            "motor_params": motor_execution["params"]
        }
        
        # Save to buffer and periodically write to file
        self.data_buffer.append(combined_data)
        
        # Write to file every 1000 readings
        if len(self.data_buffer) % 1000 == 0:
            self.write_buffer_to_file()
    
    def write_buffer_to_file(self):
        """Write buffered data to file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"data_{timestamp}.json"
        
        with open(filename, 'w') as f:
            json.dump(list(self.data_buffer), f)
    
    def run(self):
        current_phase = 0
        try:
            while True:
                # Generate motor commands (square wave)
                motor_state = {
                    "timestamp": datetime.now().isoformat(),
                    "params": {
                        "motor_id": 0,
                        "amplitude": np.sin(current_phase),
                        "phase": current_phase
                    }
                }
                
                # Send motor commands and get execution confirmation
                motor_execution = self.send_motor_command(motor_state)
                
                # Read sensor data
                sensor_data = self.read_sensor_data()
                
                # Save synchronized data
                if sensor_data and motor_execution:
                    self.save_data(sensor_data, motor_execution)
                
                current_phase += 0.1
                time.sleep(0.1)  # Maintain 10Hz cycle
        except KeyboardInterrupt:
            self.sensor_uart.close()
            self.motor_uart.close()
            print("UART connections closed")
            self.write_buffer_to_file()  # Save remaining data

# Entry points for each node
def run_sensor_node(serial_port='/dev/ttyAMA0'):
    node = SensorNode(serial_port=serial_port)
    node.run()

def run_motor_node(serial_port='/dev/ttyAMA0'):
    node = MotorNode(serial_port=serial_port)
    node.run()

def run_master_node(sensor_port='/dev/ttyUSB0', motor_port='/dev/ttyUSB1'):
    node = MasterNode(sensor_port=sensor_port, motor_port=motor_port)
    node.run()

# Example usage:
if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python script.py [master|sensor|motor] [serial_port]")
        sys.exit(1)
    
    mode = sys.argv[1].lower()
    port = sys.argv[2] if len(sys.argv) > 2 else None
    
    if mode == "master":
        sensor_port = sys.argv[2] if len(sys.argv) > 2 else '/dev/ttyUSB0'
        motor_port = sys.argv[3] if len(sys.argv) > 3 else '/dev/ttyUSB1'
        run_master_node(sensor_port, motor_port)
    elif mode == "sensor":
        port = port or '/dev/ttyAMA0'
        run_sensor_node(port)
    elif mode == "motor":
        port = port or '/dev/ttyAMA0'
        run_motor_node(port)
    else:
        print(f"Unknown mode: {mode}")
