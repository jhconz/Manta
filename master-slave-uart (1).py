# Master Raspberry Pi script (using MCP2221 GPIO expander)
# For the Pi that will send commands via MCP2221 UART
# Using BCM pin numbering mode

import time
import board
import busio
from adafruit_mcp2221.mcp2221 import MCP2221

# Initialize the MCP2221
i2c = busio.I2C(board.SCL, board.SDA)
mcp = MCP2221(i2c)

# Configure TX and RX pins on MCP2221
# GP0 = TX, GP1 = RX
uart_tx = mcp.gp0
uart_rx = mcp.gp1

# Set up UART on the MCP2221
uart = busio.UART(tx=uart_tx, rx=uart_rx, baudrate=9600)

# Commands
LED_ON = b'ON\n'
LED_OFF = b'OFF\n'
STATUS_REQUEST = b'STATUS\n'

print("Master initialized. Testing MCP2221 UART communication...")

try:
    while True:
        # Send command to turn on LED on slave
        print("Sending command to turn LED ON...")
        uart.write(LED_ON)
        
        # Request status
        time.sleep(0.5)  # Short delay to allow slave to process
        uart.write(STATUS_REQUEST)
        
        # Wait for response with timeout
        response = b''
        timeout = time.monotonic() + 2  # 2 second timeout
        while time.monotonic() < timeout:
            if uart.in_waiting > 0:
                incoming = uart.read(uart.in_waiting)
                if incoming:
                    response += incoming
                    if b'\n' in response:
                        break
            time.sleep(0.1)
        
        if response:
            print(f"Received: {response.decode().strip()}")
        else:
            print("No response received - check connections")
        
        time.sleep(2)
        
        # Send command to turn off LED on slave
        print("Sending command to turn LED OFF...")
        uart.write(LED_OFF)
        
        # Request status again
        time.sleep(0.5)
        uart.write(STATUS_REQUEST)
        response = b''
        timeout = time.monotonic() + 2
        while time.monotonic() < timeout:
            if uart.in_waiting > 0:
                incoming = uart.read(uart.in_waiting)
                if incoming:
                    response += incoming
                    if b'\n' in response:
                        break
            time.sleep(0.1)
        
        if response:
            print(f"Received: {response.decode().strip()}")
        else:
            print("No response received - check connections")
        
        time.sleep(2)

except KeyboardInterrupt:
    print("Program terminated by user")
