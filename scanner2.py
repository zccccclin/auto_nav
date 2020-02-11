#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


def callback(msg):
	# create numpy array
	laser_range = np.array([msg.ranges])
	# replace 0's with nan
	lr2 = laser_range
	# find index with minimum value
	lr2i = lr2[0]
	
	# log the info
    	rospy.loginfo('Shortest distance is %i degrees', lr2i)


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
