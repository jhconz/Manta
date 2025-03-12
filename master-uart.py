# Master Raspberry Pi script (using MCP2221 GPIO expander via USB)
# For the Pi that will send commands via MCP2221 UART

import time
import hid
from mcp2221 import MCP2221

# Initialize the MCP2221 via USB
# First attempt to find and connect to the device
mcp = None
try:
    # List available HID devices to help with debugging
    for device in hid.enumerate():
        if device['vendor_id'] == 0x04D8 and device['product_id'] == 0x00DD:  # MCP2221 VID/PID
            print(f"Found MCP2221 at path: {device['path']}")
    
    # Connect to the MCP2221
    mcp = MCP2221()
    print("Connected to MCP2221 via USB")
except Exception as e:
    print(f"Error connecting to MCP2221: {e}")
    exit(1)

# Configure UART pins on MCP2221
# GP0 = TX, GP1 = RX
# Set GPIO mode to UART
try:
    # Configure GP0 as UART TX
    mcp.gp_set_mode(0, MCP2221.GP_MODE_UART_TX)
    # Configure GP1 as UART RX
    mcp.gp_set_mode(1, MCP2221.GP_MODE_UART_RX)
    
    # Set UART baudrate to 9600
    mcp.uart_set_config(9600)
    print("MCP2221 UART configured at 9600 baud")
except Exception as e:
    print(f"Error configuring MCP2221 UART: {e}")
    exit(1)

# Commands to send
LED_ON = b'ON\n'
LED_OFF = b'OFF\n'
STATUS_REQUEST = b'STATUS\n'

print("Master initialized. Testing MCP2221 UART communication...")

try:
    while True:
        # Send command to turn on LED on slave
        print("Sending command to turn LED ON...")
        mcp.uart_write(LED_ON)
        
        # Request status
        time.sleep(0.5)  # Short delay to allow slave to process
        mcp.uart_write(STATUS_REQUEST)
        
        # Wait for response with timeout
        response = b''
        timeout = time.time() + 2  # 2 second timeout
        while time.time() < timeout:
            data = mcp.uart_read()
            if data:
                response += data
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
        mcp.uart_write(LED_OFF)
        
        # Request status again
        time.sleep(0.5)
        mcp.uart_write(STATUS_REQUEST)
        response = b''
        timeout = time.time() + 2
        while time.time() < timeout:
            data = mcp.uart_read()
            if data:
                response += data
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
except Exception as e:
    print(f"Error during operation: {e}")
finally:
    # Release the MCP2221 device
    if mcp:
        print("Closing connection to MCP2221")
        mcp.close()
