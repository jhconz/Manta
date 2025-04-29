# Slave Raspberry Pi script
# For the Pi that will receive commands and control the LED

import time
import board
import busio
import digitalio

# Set up UART on the Raspberry Pi hardware pins
# Using hardware UART on the Raspberry Pi (GPIO 14 and 15)
uart = busio.UART(tx=board.TX, rx=board.RX, baudrate=9600)

# Set up the LED to be controlled
led = digitalio.DigitalInOut(board.D18)  # Using GPIO 18 for the LED
led.direction = digitalio.Direction.OUTPUT

print("Slave initialized. Waiting for commands...")

# Buffer for incoming data
buffer = bytearray()

try:
    while True:
        # Check for incoming data
        if uart.in_waiting > 0:
            data = uart.read(uart.in_waiting)
            if data:
                # Add to buffer
                buffer.extend(data)
                
                # Process complete commands (ending with newline)
                while b'\n' in buffer:
                    idx = buffer.find(b'\n')
                    command = buffer[:idx].strip()
                    buffer = buffer[idx + 1:]
                    
                    # Process command
                    if command == b'ON':
                        print("Turning LED ON")
                        led.value = True
                        uart.write(b'LED is ON\n')
                    elif command == b'OFF':
                        print("Turning LED OFF")
                        led.value = False
                        uart.write(b'LED is OFF\n')
                    elif command == b'STATUS':
                        status = "ON" if led.value else "OFF"
                        print(f"Status requested, LED is {status}")
                        uart.write(f"LED is {status}\n".encode())
                    else:
                        print(f"Unknown command: {command}")
                        uart.write(b'Unknown command\n')
        
        # Small delay to prevent CPU hogging
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Program terminated by user")
    led.value = False  # Turn off LED on exit
