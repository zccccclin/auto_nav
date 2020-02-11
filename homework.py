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

def callback(msg):
    a = 2.5
	# create numpy array
    laser_range = np.array([msg.ranges])
	# replace 0's with nan
    lr2 = laser_range
	# find index with minimum value
    lr2i = lr2[0]
    rospy.loginfo('Shortest distance is %i degrees', lr2i)
    while True:
            if lr2i == 1:
                a = a*-1
                p.ChangeDutyCycle(7.5+a)
                time.sleep(1)
                GPIO.output(plunger,GPIO.HIGH)
                time.sleep(1)
            else:
                GPIO.output(plunger,GPIO.LOW)
                time.sleep(1)


def scanner():
	# initialize node
	rospy.init_node('scanner', anonymous=True)

	# set the update rate to 1 Hz
	rate = rospy.Rate(5) # 1 Hz

	# subscribe to LaserScan data
	rospy.Subscriber('scan', LaserScan, callback)
    
	# wait until it is time to run again
	rate.sleep()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
    try:
	    scanner()
    except  rospy.ROSInterruptException:
        pass
