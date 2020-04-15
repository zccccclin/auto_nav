#!/usr/bin/env python

import time
import threading
import math
import numpy
import time as tm
import matplotlib.pyplot as mt
import rospy
import copy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import OccupancyGrid
import cv2
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

global pub, move, cnt, time, free_time, global_distance, global_mod, x, y, a, b, w, z, flag, computation, odometry, positionx, positiony, robotx, roboty
rospy.init_node('obstacle_avoidance',anonymous=True)
pub=rospy.Publisher('/cmd_vel' ,Twist,queue_size=10)
move=Twist()
flag = 0 
time = 0 
x = 0 
y = 0 
a = 0 
b = 0 
cnt = 0
global_distance = 0
global_mod = 0 
positionx = 0 
positiony = 0 
robotx = 0 
roboty = 0
computation = []
odometryx = numpy.array([])
odometryy = numpy.array([])
odometryw = numpy.array([])
odometryz = numpy.array([])
map_matrix = numpy.full((5000, 5000), 0)
turtlebot = numpy.zeros((30, 30))
init = [2500, 2500]

occdata = numpy.array([])
occ_bins = [-1, 0, 100, 101]
def get_occupancy(msg):
    global occdata
    global cnt

    # create numpy array
    msgdata = numpy.array(msg.data)
    # compute histogram to identify percent of bins with -1
    occ_counts = numpy.histogram(msgdata,occ_bins)
    # calculate total number of bins
    total_bins = msg.info.width * msg.info.height
    # log the info
    # rospy.loginfo('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i', occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)

    # make msgdata go from 0 instead of -1, reshape into 2D
    oc2 = msgdata + 1
    # reshape to 2D array using column order
    occdata = numpy.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
    
    contourCheck = 1
    if contourCheck :
        if closure(occdata) :
                # map is complete, so save current time into file
            with open("maptime.txt", "w") as f:
                f.write("done")
            contourCheck = 0
                # play a sound
            soundhandle = SoundClient()
            rospy.sleep(1)
            soundhandle.stopAll()
            soundhandle.play(SoundRequest.NEEDS_UNPLUGGING)
            rospy.sleep(2)
                # save the map
            cv2.imwrite('mazemap.png',occdata)
            cnt = 2
            print("Map finished, Check your map at 'mapzemap.png'.")
            

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
    ALTHRESH = 8
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
    cAL = numpy.zeros((lc,2))
    for i in range(lc):
        cAL[i,0] = cv2.contourArea(contours[i])
        cAL[i,1] = cv2.arcLength(contours[i], True)

    # closed contours tend to have a much higher area to arc length ratio,
    # so if there are no contours with high ratios, we can safely say
    # there are no closed contours
    cALratio = cAL[:,0]/cAL[:,1]
    rospy.loginfo('Closure: %s', str(cALratio))
    if numpy.any(cALratio > ALTHRESH):
        return True
    else:
        return False

def alpha (distance_differ, k):
    chord = 0.3
    d = distance_differ[k]
    angle = 2 * 180 / math.pi * math.atan(chord/2/d)
    return int(angle) + 1
def motion(mod,k,distance_differ,alpha):
    global flag
    distance = float(distance_differ[k])
    linear = round( 0.26-0.26* math.exp(-1* (distance -0.3)) ,2)
    if linear > 0.04:
        flag = 0
        angular =numpy.sign(mod)*round( 1.8 -1.8*math.exp(-0.35*abs(mod*math.pow(linear,1.5) )) ,2)
    elif flag == 0:
        flag = 1
        if mod != 0:
            angular = numpy.sign(mod) * 0.4 
        else :
            angular = 0.4
    return linear,angular

def start2(msg , x, y):
    angle = range(x,360) + range(0,y)
    maps = {}
    for i in angle:
        if round( float(msg.ranges [i]),2) == 0:
            value = float(5)
        else:
            value = round(float(msg.ranges [i ]),2)
        maps[i] = value
    map_distances = sorted(maps.values() , reverse = True)
    return maps

def start(msg, x , y):
    angle = range(x,360) + range( 0,y)
    maps = {}
    maps_copy = {}
    for i in angle:
        if round(float(msg.ranges[i]),2) == 0:
            value = float(5)
        else:
            value = round(float(msg.ranges [i ]),2)
        maps[i] = value
    map_distances = sorted(maps.values() , reverse = True)
    map_angles = []
    distance_differ = []
    maps_copy= maps.copy()
    for j in map_distances:
        for i in range(len(maps_copy)):
            if maps_copy.values()[i] == j:
                map_angles.append(maps_copy.keys()[i])
                maps_copy[maps.keys()[i]] = -1
                break
    max_map = [map_angles, map_distances]
    for i in range(len(map_distances)):
        j = 0
        if i != 180:
            if map_distances[i] != map_distances[i +1]:
                distance_differ.append(map_distances [i])
            j += 1
        elif i == 180 and map_distances [i] != map_distances [i-1]:
            distance_differ.append(map_distances [i])
    return maps, max_map, distance_differ

def direction(maps, distance_differ, maps2):
    global global_distance , global_mod
    angle = range(270,360) + range(0,91)
    mod = 0 
    k = 0 
    point = 0 
    beta = 0
    clock = True 
    while (clock):
        selection = []
        p_start = len(angle)/2 + mod
        point = alpha(distance_differ , k)
        for i in range(point):
            p = p_start - point/2 + i
            selection.append(angle[p])
        value = distance_differ[k]
        flag = 0
        for j in selection:
            if maps[j] >= value:
                flag = 1
            else:
                flag = 0
                break
        if flag == 0:
            if mod < 0:
                mod = abs( mod)
            else:
                mod = -mod -1
        else :
            controller = control(maps2, distance_differ, mod, k)
            if controller == 1:
                if mod == 0:
                    counter, controller = medium (mod, maps, maps2, selection, value, distance_differ, k)
                    beta = mod + counter
                    clock = False
                    break
                else:
                    counter, controller = medium (mod, maps ,maps2, selection, value, distance_differ , k)
                    if mod > 0:
                        beta = mod + round( counter / 2)
                    else:
                        beta = mod - round( counter / 2)
                    if abs(global_mod-beta) <= ((abs (global_distance - value)+ 0.04) *500):
                        clock = False
                        break
                    else:
                        if mod < 0:
                            mod = abs(mod)
                        else:
                            mod = -mod -1
            else:
                if mod < 0:
                    mod = abs(mod)
                else:
                    mod = -mod -1
        if mod >= 90 - point/2:
            k += 1
            mod = 0
        if k >= len(distance_differ):
            k = "ERROR"
            clock = False
    return beta, k, alpha, value

def control(maps2, distance_differ, mod, k):
    angle = range(181,360) + range(0,181)
    point = 0
    z = len(distance_differ)-1
    bandiera = True
    free_space = 0
    while(bandiera):
        space_angle = []
        p_start = len(angle)/2 + mod
        point = alpha(distance_differ, z)
        for i in range(point) :
            p = p_start - point/2 + i
            space_angle.append(angle[p])
        value = distance_differ[z]
        for j in space_angle:
            if maps2[j] >= value:
                free_space = 1
            else:
                free_space = 0
                bandiera = False
                break
        if free_space == 1:
            z += -1
        if z == k:
            bandiera = False
    return free_space

def medium(mod, maps,maps2, selection , value , distance_differ, k):
    counter = 0 
    g = 0 
    index = 0 
    controller = 0
    flag = True 
    flag1 = True 
    flag2 = True
    if mod != 0:
        while(flag):
            if mod > 0:
                g += 2
                index = selection[-1]+g
            elif mod < 0:
                g -= 2
                index = selection [0]+g
            prova = tm.time ()
            if maps.has_key(index):
                if maps[index] < value:
                    flag = False
                else:
                    controller = control(maps2, distance_differ, mod+g, k)
                    if controller == 1:
                        counter += 2 
                    else:
                        flag = False
            else:
                flag = False
    elif mod == 0:
        while (flag1):
            g += 2
            index = selection[-1]+g
            if maps.has_key(index) :
                if maps[index ] < value:
                    flag1 = False
                else:
                    controller = control(maps2, distance_differ, mod+g, k)
                    if controller == 1:
                        counter +=2
                    else:
                        flag1 = False
            else:
                flag1 = False
        g = 0
        while (flag2):
            g -= 2
            index = selection[0]+g
            if maps.has_key(index):
                if maps[index] < value:
                    flag2 = False
                else:
                    controller = control(maps2, distance_differ, mod+g, k)
                    if controller == 1:
                        counter -=2
                    else:
                        flag2 = False
            else:
                flag2 = False
    return counter,controller



def LiDAR(msg):
    global time , global_distance, global_mod, computation, odometryx, odometryy, odometryw, odometryz, x , y, w, z , a ,b, map_matrix, init
    global cnt
    if cnt == 1:
        time += 1
        print '*'*5 + 'Time' + '*'* 5
        print time
        if time == 1:
            a = x
            b = y
        maps, max_map, distance_differ = start(msg, 270, 91)
        maps2 = start2( msg , 181, 181)
        beta, k, alpha , value = direction (maps, distance_differ, maps2)
        if k == 'ERROR':
            move.linear.x = -0.5
            move.angular.z = 0
            pub.publish(move)
            k = 0
            beta, k, alpha , value = direction (maps, distance_differ, maps2)
        linear , angular = motion(beta , k, distance_differ, alpha)
        global_distance = value
        global_mod = numpy.sign(beta )*int(abs(beta )-angular*0.2*180/math.pi )
        move.linear.x = linear
        move.angular.z = angular
        print '*'*5 + " The robot moves at %.2f meters, rotating for %.2f degrees" % (value , beta) + '*' *5
        print '*'*5 + " Linear velocity = %.2f m/s , Angular velocity = %.2f rad/s" %(linear,angular) + '*'* 5
        w = numpy.sign(z)* round(float( 2*math.acos(w)* 180/ math.pi) , 2)
        print '*'*5 + "Odometry x = %.2f m , y = %.2f m, theta = %.2f degree" %(x-a, y-b, w)+'*'* 5+"\n "
        
    elif cnt== 2:
        move.linear.x =0.0
        move.angular.z =0.0
        pub.publish(move)
        
    pub.publish( move )
    
def Position(msg):
    global x,y,z ,w
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    w = msg.pose.pose.orientation.w
    z = msg.pose.pose.orientation.z

class Publisher(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
    def run(self):
        osub=rospy.Subscriber('scan', LaserScan,LiDAR)
        sub1 =rospy.Subscriber("odom", Odometry, Position)
        rospy.Subscriber('map', OccupancyGrid, get_occupancy)        
        rospy.spin( )
        
if __name__=='__main__': 
    p=Publisher()
    p.start()
    while True: 
        gostop = raw_input("Press 'g'-> to Start the navigation / 's'-> to Stop the robot motion:")
        if gostop=='g':
            cnt = 1
        elif gostop=='s':
            cnt = 2 
