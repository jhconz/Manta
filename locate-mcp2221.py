# Script to locate and connect to MCP2221 device
# This will help identify the correct path to use in the main script

import hid

# MCP2221 USB identifiers
MCP2221_VID = 0x04D8  # Vendor ID for Microchip
MCP2221_PID = 0x00DD  # Product ID for MCP2221

def find_mcp2221_devices():
    """Find all MCP2221 devices connected to the system"""
    devices = []
    for device in hid.enumerate():
        if device['vendor_id'] == MCP2221_VID and device['product_id'] == MCP2221_PID:
            devices.append(device)
    return devices

if __name__ == "__main__":
    devices = find_mcp2221_devices()
    
    if not devices:
        print("No MCP2221 devices found. Check connections and permissions.")
    else:
        print(f"Found {len(devices)} MCP2221 device(s):")
        for i, device in enumerate(devices):
            print(f"\nDevice {i+1}:")
            print(f"  Path: {device['path']}")
            print(f"  Serial Number: {device.get('serial_number', 'N/A')}")
            print(f"  Manufacturer: {device.get('manufacturer_string', 'N/A')}")
            print(f"  Product: {device.get('product_string', 'N/A')}")
            print(f"  Interface Number: {device.get('interface_number', 'N/A')}")
            
            # Try to open the device to verify we have permissions
            try:
                # Convert byte path to string if needed
                path = device['path']
                if isinstance(path, bytes):
                    path = path.decode('utf-8')
                
                dev = hid.device()
                dev.open_path(device['path'])
                print("  Successfully opened device connection")
                dev.close()
            except Exception as e:
                print(f"  Error opening device: {e}")
                print("  You may need to run with sudo or fix permissions")
