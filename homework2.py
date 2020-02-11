#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import time
import RPi.GPIO as GPIO

laser_range = np.array([])
GPIO.setmode(GPIO.BOARD)
servo = 32
plunger = 36
a = 2.5

GPIO.setup(servo, GPIO.OUT)
GPIO.setup(plunger, GPIO.OUT)
p=GPIO.PWM(servo,50)
p.start(7.5)

def callback(msg):
    global laser_range
    laser_range = np.array([msg.ranges])


def scanner():
    global laser_range
    global a
	# initialize node
    rospy.init_node('scanner', anonymous=True)
	# set the update rate to 1 Hz
    rate = rospy.Rate(5) # 1 Hz
    
        
	# subscribe to LaserScan data
    rospy.Subscriber('scan', LaserScan, callback)
    while True:
        lr2i = laser_range[0][0]
        rospy.loginfo('The distance in front is %i ', lr2i)
        
        if lr2i == 1:
            a = a*-1
            p.ChangeDutyCycle(7.5+a)
            time.sleep(1)
            GPIO.output(plunger,GPIO.HIGH)
            time.sleep(1)
        else:
            GPIO.output(plunger,GPIO.LOW)
            time.sleep(1)
    
	# wait until it is time to run again
	rate.sleep()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
    try:
	    scanner()
    except  rospy.ROSInterruptException:
        pass
