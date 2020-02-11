#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import time
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BOARD)
servo = 32
plunger = 36

GPIO.setup(servo, GPIO.OUT)
GPIO.setup(plunger, GPIO.OUT)
p=GPIO.PWM(servo,50)
p.start(7.5)
laser_range = np.array([])


def getlaser(msg):
    	global laser_range
        laser_range = np.array([msg.ranges])
	


def scanner():
    global laser_range
	# initialize node
    rospy.init_node('scanner', anonymous=True)

	# set the update rate to 1 Hz
    rate = rospy.Rate(5) # 1 Hz

	# subscribe to LaserScan data
    rospy.Subscriber('scan', LaserScan, getlaser)
    lr2 = laser_range
    lr2i = lr2[0][0]
    rospy.loginfo(lr2i)
        
	# wait until it is time to run again
    rate.sleep()

	# spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
	    scanner()
    except  rospy.ROSInterruptException:
        pass
