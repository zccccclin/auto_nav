#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import RPi.GPIO as GPIO

laser_range = np.array([])
servo = 32 #servo_pin
plunger = 36 #plunger_pin
angle = 2.5 #45 degrees angle for servo


def servo_setup(): #setup servo
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(servo, GPIO.OUT)
    global p
    p=GPIO.PWM(servo,50)
    p.start(7.5)


def get_laserscan(msg):
    global laser_range

    # create numpy array
    laser_range = np.array([msg.ranges])


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

def trial():
    global laser_range

    rospy.init_node('trial', anonymous=True)
    rospy.Subscriber('scan', LaserScan, get_laserscan)
    rate = rospy.Rate(5) # 5 Hz
    time.sleep(1)

    servo_setup()
    plunger_setup()

    while not rospy.is_shutdown():
        lr2 = laser_range[0]
        lr2i = lr2[0]
	rospy.loginfo(round(lr2i,2))
        if round(lr2i,2) == 1:
            rotation()
            plunger_punch()
	    a = input('Do you wish to continue: 1 = yes, 2 = no ')
	    if a == 1:
		continue
	    else:
		break
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    try:
        trial()
    except rospy.ROSInterruptException:
	p.stop()
        GPIO.cleanup()
        pass
