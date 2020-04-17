#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb  4 17:43:19 2020

@author: notfromajedi
"""

import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
test_point = 32
GPIO.setup(test_point, GPIO.OUT)

pwm = GPIO.PWM(test_point, 1000)
pwm.start(0)

print("Start")

for i in range(0,100,1):
    pwm.ChangeDutyCycle(i)
    print "Brightness is ",i,"%"
    time.sleep(0.2)
else:
    print("Finished")
    
pwm.stop()
GPIO.cleanup()    
