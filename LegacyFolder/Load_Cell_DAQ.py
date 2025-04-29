# -*- coding: utf-8 -*-
"""
Created on Thu Feb 13 11:19:51 2025

@author: Student
"""

# SPDX-FileCopyrightText: Copyright (c) 2024 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import time
import csv
import board
import digitalio
from adafruit_hx711.hx711 import HX711
from adafruit_hx711.analog_in import AnalogIn

###############################################################################
class DataClock:
    """A continuous clock class for data acquisition timestamping."""
    def __init__(self):
        self.start_time = time.monotonic()
        
    def get_timestamp(self, precision=3):
        """Get current timestamp relative to start time."""
        return round(time.monotonic() - self.start_time, precision)

###############################################################################
# Initialize all sensors
#   Anterior Lift
#
#   Postiive
data_la_pos = digitalio.DigitalInOut(board.D5)
data_la_pos.direction = digitalio.Direction.INPUT
clock_la_pos = digitalio.DigitalInOut(board.D6)
clock_la_pos.direction = digitalio.Direction.OUTPUT
hx711_la_pos = HX711(data_la_pos, clock_la_pos)
channel_a_la_pos = AnalogIn(hx711_la_pos, HX711.CHAN_A_GAIN_128)
#
#   Negative
data_la_neg = digitalio.DigitalInOut(board.D5)
data_la_neg.direction = digitalio.Direction.INPUT
clock_la_neg = digitalio.DigitalInOut(board.D6)
clock_la_neg.direction = digitalio.Direction.OUTPUT
hx711_la_neg = HX711(data_la_neg, clock_la_neg)
channel_a_la_neg = AnalogIn(hx711_la_neg, HX711.CHAN_A_GAIN_128)
#
#   Posterior Lift
#
#   Postiive
data_lp_pos = digitalio.DigitalInOut(board.D5)
data_lp_pos.direction = digitalio.Direction.INPUT
clock_lp_pos = digitalio.DigitalInOut(board.D6)
clock_lp_pos.direction = digitalio.Direction.OUTPUT
hx711_lp_pos = HX711(data_lp_pos, clock_lp_pos)
channel_a_lp_pos = AnalogIn(hx711_lp_pos, HX711.CHAN_A_GAIN_128)
#
#   Negative
data_lp_neg = digitalio.DigitalInOut(board.D5)
data_lp_neg.direction = digitalio.Direction.INPUT
clock_lp_neg = digitalio.DigitalInOut(board.D6)
clock_lp_neg.direction = digitalio.Direction.OUTPUT
hx711_lp_neg = HX711(data_lp_neg, clock_lp_neg)
channel_a_lp_neg = AnalogIn(hx711_lp_neg, HX711.CHAN_A_GAIN_128)
#
#   Thrust
#
#   Postiive
data_t_pos = digitalio.DigitalInOut(board.D5)
data_t_pos.direction = digitalio.Direction.INPUT
clock_t_pos = digitalio.DigitalInOut(board.D6)
clock_t_pos.direction = digitalio.Direction.OUTPUT
hx711_t_pos = HX711(data_t_pos, clock_t_pos)
channel_a_t_pos = AnalogIn(hx711_t_pos, HX711.CHAN_A_GAIN_128)
#
#   Negative
data_t_neg = digitalio.DigitalInOut(board.D5)
data_t_neg.direction = digitalio.Direction.INPUT
clock_t_neg = digitalio.DigitalInOut(board.D6)
clock_t_neg.direction = digitalio.Direction.OUTPUT
hx711_t_neg = HX711(data_t_neg, clock_t_neg)
channel_a_t_neg = AnalogIn(hx711_t_neg, HX711.CHAN_A_GAIN_128)
###############################################################################
# Initialize Clock
clock = DataClock()

# Create headers for CSV file
headers = ['Timestamp', 
          'Anterior_Lift_Pos', 'Anterior_Lift_Neg',
          'Posterior_Lift_Pos', 'Posterior_Lift_Neg',
          'Thrust_Pos', 'Thrust_Neg']

# Open CSV file and write headers
with open('load_cell_data.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(headers)
    
    try:
        while True:
            # Get timestamp
            timestamp = clock.get_timestamp()
            
            # Read all sensors
            data_row = [
                timestamp, 
                channel_a_la_pos.value, channel_a_la_neg.value,
                channel_a_lp_pos.value, channel_a_lp_neg.value,
                channel_a_t_pos.value, channel_a_t_neg.value
            ]
            
            # Write data to CSV
            writer.writerow(data_row)
            
            # Optional: Print data for monitoring
            #print(f"Time: {timestamp:.3f}s, Anterior Lift +: {channel_a_la_pos.value}")
            
            # Small delay to control sampling rate
            # time.sleep(0.1)  # 10 Hz sampling rate, adjust as needed
            
    except KeyboardInterrupt:
        print("\nData acquisition stopped")
