#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 13 18:07:27 2020

@author: notfromajedi
"""

import time 
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
servo_pin=31

GPIO.setup(servo_pin, GPIO.OUT)
p=GPIO.PWM(servo_pin, 50)
p.start(7.5)

try:
    while True:
        p.ChangeDutyCycle(7.5)
        time.sleep(1)
        p.ChangeDutyCycle(2.5)
        time.sleep(1)
        p.ChangeDutyCycle(12.5)
        time.sleep(1)
except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()
