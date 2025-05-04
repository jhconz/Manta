import RPi.GPIO as GPIO
from hx711py.hx711 import HX711
import time

def test_load_cells():
    # Configure pins for each load cell
    loadcell_pins = {
        'loadcell1': {'dout': 20, 'sck': 21},
        'loadcell2': {'dout': 17, 'sck': 18},
        'loadcell3': {'dout': 5, 'sck': 6}
    }
    
    # Calibration factors
    calibration_factors = {
        'loadcell1': 219.1472,
        'loadcell2': -221.8872,
        'loadcell3': -221.4656
    }
    
    # Set up GPIO
    GPIO.setmode(GPIO.BCM)
    
    load_cells = {}
    
    try:
        # Initialize each load cell
        for name, pins in loadcell_pins.items():
            print(f"Setting up {name} on pins DOUT={pins['dout']}, SCK={pins['sck']}")
            load_cells[name] = HX711(pins['dout'], pins['sck'])
            load_cells[name].set_reading_format("MSB", "MSB")
            load_cells[name].set_reference_unit(calibration_factors[name])
            load_cells[name].reset()
            load_cells[name].tare()
            print(f"{name} tare done")
        
        # Read values in a loop
        for _ in range(10):
            for name, load_cell in load_cells.items():
                try:
                    value = load_cell.get_weight(1)
                    print(f"{name}: {value}")
                except Exception as e:
                    print(f"Error reading {name}: {e}")
            
            print("-----")
            time.sleep(1)
    
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    test_load_cells()
