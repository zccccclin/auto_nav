#!/usr/bin/env python
#yes
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import math
import cmath
import numpy as np
import time
import cv2
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

laser_range = np.array([])
occdata = np.array([])
yaw = 0.0
rotate_speed = 0.1
linear_speed = 0.1
stop_distance = 0.25
occ_bins = [-1, 0, 100, 101]
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)


def get_odom_dir(msg):
    global yaw

    orientation_quat =  msg.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


def get_laserscan(msg):
    global laser_range

    # create numpy array
    laser_range = np.array(msg.ranges)
    # replace 0's with nan's
    # could have replaced all values below msg.range_min, but the small values
    # that are not zero appear to be useful
    laser_range[laser_range==0] = np.nan


def get_occupancy(msg):
    global occdata

    # create numpy array
    msgdata = np.array(msg.data)
    # compute histogram to identify percent of bins with -1
    occ_counts = np.histogram(msgdata,occ_bins)
    # calculate total number of bins
    total_bins = msg.info.width * msg.info.height
    # log the info
    # rospy.loginfo('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i', occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)

    # make msgdata go from 0 instead of -1, reshape into 2D
    oc2 = msgdata + 1
    # reshape to 2D array using column order
    occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))


def rotatebot(rot_angle):
    global yaw

    # create Twist object
    twist = Twist()
    # set up Publisher to cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # set the update rate to 1 Hz
    rate = rospy.Rate(1)

    # get current yaw angle
    current_yaw = np.copy(yaw)
    # log the info
    rospy.loginfo(['Current: ' + str(math.degrees(current_yaw))])
    # we are going to use complex numbers to avoid problems when the angles go from
    # 360 to 0, or from -180 to 180
    c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
    # calculate desired yaw
    target_yaw = current_yaw + math.radians(rot_angle)
    # convert to complex notation
    c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
    rospy.loginfo(['Desired: ' + str(math.degrees(cmath.phase(c_target_yaw)))])
    # divide the two complex numbers to get the change in direction
    c_change = c_target_yaw / c_yaw
    # get the sign of the imaginary component to figure out which way we have to turn
    c_change_dir = np.sign(c_change.imag)
    # set linear speed to zero so the TurtleBot rotates on the spot
    twist.linear.x = 0.0
    # set the direction to rotate
    twist.angular.z = c_change_dir * rotate_speed
    # start rotation
    pub.publish(twist)

    # we will use the c_dir_diff variable to see if we can stop rotating
    c_dir_diff = c_change_dir
    # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
    # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
    # becomes -1.0, and vice versa
    while(c_change_dir * c_dir_diff > 0):
        # get current yaw angle
        current_yaw = np.copy(yaw)
        # get the current yaw in complex form
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        rospy.loginfo('While Yaw: %f Target Yaw: %f', math.degrees(current_yaw), math.degrees(target_yaw))
        # get difference in angle between current and target
        c_change = c_target_yaw / c_yaw
        # get the sign to see if we can stop
        c_dir_diff = np.sign(c_change.imag)
        # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
        rate.sleep()

    rospy.loginfo(['End Yaw: ' + str(math.degrees(current_yaw))])
    # set the rotation speed to 0
    twist.angular.z = 0.0
    # stop the rotation
    time.sleep(1)
    pub.publish(twist)


def pick_direction():
    global laser_range

    # publish to cmd_vel to move TurtleBot
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # stop moving
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)
<<<<<<< HEAD
	
#Part to avoid obstacles
    def identify_openings(laser_range): #return a dict of potential openings with its distances
        opening_list = [] # will make [[30, 60], [120, 150]] (means there's an opening between angle 30 and 60, and angle 120 and 150
        starting_angle = 0
        for angles in range(len(laser_range)):
            initial_distance = laser_range[angles - 1]
            if laser_range[angles] - initial_distance > threshold:
                starting_angle = angles
                break
                
        rospy.loginfo('STARTING ANGLEE' + str(starting_angle))  
        for angles in range(starting_angle, 360):
            initial_distance = laser_range[angles-1]
            if abs(initial_distance - laser_range[angles]) > threshold:
                opening_list.append(angles) 
        for angles in range(0, starting_angle):
            initial_distance = laser_range[angles-1]
            if abs(initial_distance - laser_range[angles]) > threshold:
                opening_list.append(angles) 
        return opening_list
	

    def choose_best_opening(opening_list):
        if opening_list == []:
            return 0
        elif len(opening_list) == 2:
    	    return (abs(opening_list[0] - opening_list[1])//2 + opening_list[0])
        
    	else:
            list_of_potential_angles = []
            
            for x in range(0, len(opening_list) - 2, 2):
    		    angle = (abs(opening_list[x] - opening_list[x+1])//2 + opening_list[x])
    		    if angle > 180:
		            angle = angle - 360
	            list_of_potential_angles.append(abs(opening_list[x] - opening_list[x+1])//2 + opening_list[x])
    	    
    	    lis_temp = list_of_potential_angles
    	    map(abs, lis_temp)
    	    minimum = min(lis_temp)
    	    target_index = lis_temp.index(minimum)
    
    	    lr2i =  list_of_potential_angles[target_index]
            return lr2i
    opening = identify_openings(laser_range)
    lr2i = choose_best_opening(opening)
    rospy.loginfo('LIST OF OPENINGs' + str(opening))  
    if lr2i > 180:
        lr2i = lr2i - 360
    try:
        rospy.loginfo(['Picked direction: ' + str(lr2i) + ' ' + str(laser_range[lr2i]) + ' m'])
=======

    if laser_range.size != 0:
        # use nanargmax as there are nan's in laser_range added to replace 0's
        lr2i = np.nanargmax(laser_range)
    else:
        lr2i = 0

    rospy.loginfo(['Picked direction: ' + str(lr2i) + ' ' + str(laser_range[lr2i]) + ' m'])

>>>>>>> ba4cdd381beeb71b8b35d108653f250d6839bc4a
    # rotate to that direction
    rotatebot(float(lr2i))

    # start moving
    rospy.loginfo(['Start moving'])
    twist.linear.x = linear_speed
    twist.angular.z = 0.0
    # not sure if this is really necessary, but things seem to work more
    # reliably with this
    time.sleep(1)
    pub.publish(twist)


def stopbot():
    # publish to cmd_vel to move TurtleBot
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)


def closure(mapdata):
    # This function checks if mapdata contains a closed contour. The function
    # assumes that the raw map data from SLAM has been modified so that
    # -1 (unmapped) is now 0, and 0 (unoccupied) is now 1, and the occupied
    # values go from 1 to 101.

    # According to: https://stackoverflow.com/questions/17479606/detect-closed-contours?rq=1
    # closed contours have larger areas than arc length, while open contours have larger
    # arc length than area. But in my experience, open contours can have areas larger than
    # the arc length, but closed contours tend to have areas much larger than the arc length
    # So, we will check for contour closure by checking if any of the contours
    # have areas that are more than 10 times larger than the arc length
    # This value may need to be adjusted with more testing.
    ALTHRESH = 10
    # We will slightly fill in the contours to make them easier to detect
    DILATE_PIXELS = 3

    # assumes mapdata is uint8 and consists of 0 (unmapped), 1 (unoccupied),
    # and other positive values up to 101 (occupied)
    # so we will apply a threshold of 2 to create a binary image with the
    # occupied pixels set to 255 and everything else is set to 0
    # we will use OpenCV's threshold function for this
    ret,img2 = cv2.threshold(mapdata,2,255,0)
    # we will perform some erosion and dilation to fill out the contours a
    # little bit
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(DILATE_PIXELS,DILATE_PIXELS))
    # img3 = cv2.erode(img2,element)
    img4 = cv2.dilate(img2,element)
    # use OpenCV's findContours function to identify contours
    # OpenCV version 3 changed the number of return arguments, so we
    # need to check the version of OpenCV installed so we know which argument
    # to grab
    fc = cv2.findContours(img4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    (major, minor, _) = cv2.__version__.split(".")
    if(major == '3'):
        contours = fc[1]
    else:
        contours = fc[0]
    # find number of contours returned
    lc = len(contours)
    # rospy.loginfo('# Contours: %s', str(lc))
    # create array to compute ratio of area to arc length
    cAL = np.zeros((lc,2))
    for i in range(lc):
        cAL[i,0] = cv2.contourArea(contours[i])
        cAL[i,1] = cv2.arcLength(contours[i], True)

    # closed contours tend to have a much higher area to arc length ratio,
    # so if there are no contours with high ratios, we can safely say
    # there are no closed contours
    cALratio = cAL[:,0]/cAL[:,1]
    # rospy.loginfo('Closure: %s', str(cALratio))
    if np.any(cALratio > ALTHRESH):
        return True
    else:
        return False


def mover():
    global laser_range

    rospy.init_node('mover', anonymous=True)

    # subscribe to odometry data
    rospy.Subscriber('odom', Odometry, get_odom_dir)
    # subscribe to LaserScan data
    rospy.Subscriber('scan', LaserScan, get_laserscan)
    # subscribe to map occupancy data
    rospy.Subscriber('map', OccupancyGrid, get_occupancy)

    rospy.on_shutdown(stopbot)

    rate = rospy.Rate(5) # 5 Hz

    # save start time
    start_time = time.time()
    # initialize variable to write elapsed time to file
    contourCheck = 1

    # find direction with the largest distance from the Lidar,
    # rotate to that direction, and start moving
<<<<<<< HEAD
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 1.0
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)
=======
    pick_direction()
>>>>>>> ba4cdd381beeb71b8b35d108653f250d6839bc4a

    while not rospy.is_shutdown():
        if laser_range.size != 0:
            # check distances in front of TurtleBot and find values less
            # than stop_distance
            lri = (laser_range[front_angles]<float(stop_distance)).nonzero()
            rospy.loginfo('Distances: %s', str(lri))
        else:
            lri[0] = []

        # if the list is not empty
        if(len(lri[0])>0):
            rospy.loginfo(['Stop!'])
            # find direction with the largest distance from the Lidar
            # rotate to that direction
            # start moving
            pick_direction()

        # check if SLAM map is complete
        if contourCheck :
            if closure(occdata) :
                # map is complete, so save current time into file
                with open("maptime.txt", "w") as f:
                    f.write("Elapsed Time: " + str(time.time() - start_time))
                contourCheck = 0
                # play a sound
                soundhandle = SoundClient()
                rospy.sleep(1)
                soundhandle.stopAll()
                soundhandle.play(SoundRequest.NEEDS_UNPLUGGING)
                rospy.sleep(2)
                # save the map
                cv2.imwrite('mazemap.png',occdata)

        rate.sleep()


if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
