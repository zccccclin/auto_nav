#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import time
import RPi.GPIO as GPIO


servo = 32 #servo_pin
plunger = 36 #plunger_pin
angle = 2.5 #45 degrees angle for servo
laser_range = np.array([])



def servo_setup(): #setup servo
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(servo, GPIO.OUT)
    global p
    p=GPIO.PWM(servo,50)
    p.start(7.5)
    
def plunger_setup(): #setup plunger
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(plunger, GPIO.OUT)
    
def rotation(): #function to rotate servo 45 degrees
    global p
    global angle
    angle *= -1
    p.ChangeDutyCycle(7.5+angle)
    time.sleep(1)

def plunger_punch(): #function to punch plunger 1 time
    GPIO.output(plunger,GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(plunger,GPIO.LOW)
    time.sleep(0.1)
    
def getlaser(msg): #read lidar data as array
    	global laser_range
        laser_range = np.array([msg.ranges])
	
def scanner():
    global laser_range
	# initialize node
    rospy.init_node('scanner', anonymous=True)

	# set the update rate to 1 Hz
    rate = rospy.Rate(1) # 1 Hz

	# subscribe to LaserScan data
    rospy.Subscriber('scan', LaserScan, getlaser)
    
    servo_setup()
    plunger_setup()
    while True:
        lr2 = laser_range[0]
        lr2i = lr2[0]
        if round(lr2i,5) == 1:
            rotation()
            plunger_punch()
        rate.sleep()
	# spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
	    scanner()
    except  rospy.ROSInterruptException:
        pass
