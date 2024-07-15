# -*- coding: utf-8 -*-
"""
Created on Sat Jul 13 12:36:50 2024

@author: Student
"""
# This is the base code for controlling brushless DC motor with ras-py
import RPi.GPIO as GPIO
from time import sleep

class motorDC():
    
    def __init__(self,Pins):
        self.OUT1 = Pins[0]
        self.OUT2 = Pins[-1]
        self.setGPIO()
        
    def setGPIO(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.OUT1, GPIO.OUT)
        GPIO.setup(self.OUT2, GPIO.OUT)
        PWM1 = GPIO.PWM(self.OUT1,1000)
        PWM2 = GPIO.PWM(self.OUT2,1000)
        
        
Pins = [32,12]
Motor = motorDC()

