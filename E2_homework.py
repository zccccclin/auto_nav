#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Feb  5 20:35:52 2020

@author: tuandung
"""

import PyLidar2
import RPi.GPIO as GPIO
import time 

servo_pin = 21
solenoid_pin = 22
lidar_port = 
lidar = PyLidar2.YdLidarX4(lidar_port)

def run_lidar(): #get readings from lidar and store in a dictionary every 0.5s
    global lidar
    if (lidar.Connect()):
        print (lidar.GetDeviceInfo())
        surroundings = lidar.StartScanning()
        t = time.time()
        if (time.time() - t) == 0.5:
            return surroundings 
    else: 
        print('Device not connected')

def action(): #carry out requirements
    l2i = run_lidar()
    if l2i[0] == 1:
        rotation(45)
        solenoid_punch(1)
        time.sleep(1)
    elif l2i[0] == 0:
        print('Distance out of range')
        time.sleep(1)
    else:
        print('Distance not 1m')
        time.sleep(1)


def servo_setup(servo_pin): #set up servo motor
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(servo_pin, GPIO.OUT)
    global p
    p = GPIO.PWM(servo_pin, 50)
    p.start(2.5)

def rotation(angle): #rotate servo 
    dc = angle/18 +2.5 #convert angle to duty cycle
    global p
    p.ChangeDutyCycle(dc) #rotate motor to needed angle 
    time.sleep(0.1)

def solenoid_setup(solenoid_pin): #setup solenoid
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(solenoid_pin, GPIO.OUT) 

def solenoid_punch(n): #make solenoid punch n times
    for i in range(0,n):
        GPIO.output(solenoid_pin, 1)
        time.sleep(0.2)
        GPIO.output(solenoid_pin, 0)
        time.sleep(0.2)

try:
    servo_setup(servo_pin)
    solenoid_setup(solenoid_pin)
    while True: 
        action()
except KeyboardInterrupt:
    lidar.StopScanning()
    lidar.Disconnect()
    p.stop()
    GPIO.cleanup()



        
        
    
    
