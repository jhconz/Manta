                                                                    
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
        self.Out1 = Pins[0]
        self.Out2 = Pins[-1]
        self.setGPIO()
        
    def setGPIO(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Out1, GPIO.OUT)
        GPIO.setup(self.Out2, GPIO.OUT)
        

## Initialize Motors
Pins_A = [5,6]
Motor_A = motorDC(Pins_A)
Pins_B = [20,21]
Motor_B = motorDC(Pins_B)
GPIO.output(Motor_A.Out1,GPIO.LOW)
GPIO.output(Motor_A.Out2,GPIO.LOW)
GPIO.output(Motor_B.Out1,GPIO.LOW)
GPIO.output(Motor_B.Out2,GPIO.LOW)


## Actuation Scheme
t_on = 2.0
phase = 1.0
lag = 2.0


try:
    ## Half Cycle to start
    GPIO.output(Motor_A.Out2,GPIO.HIGH) # Begin MA CCW
    time.sleep(phase)
    GPIO.output(Motor_B.Out2,GPIO.HIGH) #Begin MB CCW
    GPIO.output(Motor_A.Out2,GPIO.LOW) # Begin MA CCW #End MA CCW
    time.sleep(phase)
    GPIO.output(Motor_B.Out2,GPIO.LOW) #End MB CCW
    time.sleep(lag-phase)   
    while True:
        GPIO.output(Motor_A.Out1,GPIO.HIGH) # Begin MA CW
        time.sleep(phase)
        GPIO.output(Motor_B.Out1,GPIO.HIGH) #Begin MB CW
        time.sleep(t_on-phase)
        GPIO.output(Motor_A.Out1,GPIO.LOW) #End MA CW
        time.sleep(phase)
        GPIO.output(Motor_B.Out1,GPIO.LOW) #End MB CW
        time.sleep(lag-phase)
        GPIO.output(Motor_A.Out2,GPIO.HIGH) # Begin MA CCW
        time.sleep(phase)
        GPIO.output(Motor_B.Out2,GPIO.HIGH) #Begin MB CCW
        time.sleep(t_on-phase)
        GPIO.output(Motor_A.Out2,GPIO.LOW) #End MA CCW
        time.sleep(phase)
        GPIO.output(Motor_B.Out2,GPIO.LOW) #End MB CCW
        time.sleep(lag-phase)        

except KeyboardInterrupt:
    print("End Actuation")
finally:
    GPIO.cleanup()
