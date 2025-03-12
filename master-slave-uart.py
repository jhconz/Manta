# Master Raspberry Pi script (using MCP2221 GPIO expander)
# For the Pi that will send commands

import time
import board
import busio
import digitalio
from adafruit_mcp2221.mcp2221 import MCP2221

# Initialize the MCP2221
i2c = busio.I2C(board.SCL, board.SDA)
mcp = MCP2221(i2c)

# Configure TX and RX pins on MCP2221
# GP0 = TX, GP1 = RX
uart_tx = mcp.gp0
uart_rx = mcp.gp1

# Set up UART on the MCP2221
# Note: Using software UART implementation since MCP2221 doesn't have hardware UART
uart = busio.UART(tx=uart_tx, rx=uart_rx, baudrate=9600)

# Simple LED indicator on the master to show activity
led = digitalio.DigitalInOut(board.D18)
led.direction = digitalio.Direction.OUTPUT

# Commands
LED_ON = b'ON\n'
LED_OFF = b'OFF\n'
STATUS_REQUEST = b'STATUS\n'

print("Master initialized. Starting communication...")

try:
    while True:
        # Turn on LED
        print("Sending ON command...")
        uart.write(LED_ON)
        led.value = True  # Local LED on to show command sent
        time.sleep(1)
        
        # Request status
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
            print("No response received")
        
        # Turn off LED
        print("Sending OFF command...")
        uart.write(LED_OFF)
        led.value = False  # Local LED off
        time.sleep(1)
        
        # Request status again
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
            print("No response received")
        
        time.sleep(3)  # Wait before next cycle

except KeyboardInterrupt:
    print("Program terminated by user")
