#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


def loglidar(msg):
	# create numpy array
	laser_range = np.array([msg.ranges])
	lr2 = laser_range
	lr2i = lr2[0][0]        
       

def scanner():
	# initialize node
	rospy.init_node('scanner', anonymous=True)

	# set the update rate to 1 Hz
	rate = rospy.Rate(5) # 1 Hz

	# subscribe to LaserScan data
	sub = rospy.Subscriber('scan', LaserScan, loglidar)
        print(sub)
        print(sub)
	# wait until it is time to run again
	rate.sleep()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
    try:
	    scanner()
    except  rospy.ROSInterruptException:
        pass
