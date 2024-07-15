# -*-                                                                          
"""
Created on Sat Jul 13 12:36:50 2024

@author: Student
"""
# This is the base code for controlling brushless DC motor with ras-py
import RPi.GPIO as GPIO
from time import sleep
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
        self.PWM1 = GPIO.PWM(self.OUT1,1000)
        self.PWM2 = GPIO.PWM(self.OUT2,1000)
        
        
Pins = [32,12]
Motor = motorDC(Pins)
maxPWM = 50
t = np.linspace(0,10,1000)
u1 = maxPWM*(1+np.cos(pi*t/2))
u2 = maxPWM*(1+np.sin(pi*t/2))

Motor.PWM1.start(u1[0])
Motor.PWM2.start(u2[0])

for ii in range(1000):
        Motor.PWM1.ChangeDutyCycle(u1[ii])
        Motor.PWM2.ChangeDutyCycle(u2[ii])
        sleep(.1)
        
        
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
