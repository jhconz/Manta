# -*-                                                                          
"""
Created on Sat Jul 13 12:36:50 2024

@author: Student
"""
# This is the base code for controlling brushless DC motor with ras-py
import RPi.GPIO as GPIO
import time
import numpy as np
from math import pi

class motorDC:
    
    def __init__(self,Pins):
        self.OUT1 = Pins[0]
        self.OUT2 = Pins[-1]
        self.setGPIO()
        
    def setGPIO(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.OUT1, GPIO.OUT)
        GPIO.setup(self.OUT2, GPIO.OUT)
        
        
## Initialize Motors
Pins_A = [20,21]
Motor_A = motorDC(Pins_A)
Pins_B = [23,24]
Motor_B = motorDC(Pins_B)
Motor_A.OUT1.value = False
Motor_A.OUT2.value = False
Motor_B.OUT1.value = False
Motor_B.OUT2.value = False


## Actuation Scheme
t_on = 2.0
phase = 1.0
lag = 2.0


try:
    ## Half Cycle to start
    Motor_A.OUT2.value = True # Begin MA CCW
    time.sleep(phase)
    Motor_B.OUT2.value = True #Begin MB CCW
    Motor_A.OUT2.value = False #End MA CCW
    time.sleep(phase)
    Motor_B.OUT2.value = False #End MB CCW
    time.sleep(lag-phase)   
    while True:
        Motor_A.OUT1.value = True #Begin MA CW
        time.sleep(phase)
        Motor_B.OUT1.value = True #Begin MB CW
        time.sleep(t_on-phase)
        Motor_A.OUT1.value = False #End MA CW
        time.sleep(phase)
        Motor_B.OUT1.value = False #End MB CW
        time.sleep(lag-phase)
        Motor_A.OUT2.value = True # Begin MA CCW
        time.sleep(phase)
        Motor_B.OUT2.value = True #Begin MB CCW
        time.sleep(t_on-phase)
        Motor_A.OUT2.value = False #End MA CCW
        time.sleep(phase)
        Motor_B.OUT2.value = False #End MB CCW
        time.sleep(lag-phase)        

except KeyboardInterrupt:
    print("End Actuation")