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

def alpha (distance_differ, k):
    chord = 0.5
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
