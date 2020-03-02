#!/usr/bin/env python

import time
import threading
import math
import numpy
import time as tm
import matplotlib.pyplot as mt
import rospy
import copy
import tf2_ros
from PIL import Image
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseStamped
global pub, move, cnt, time, free_time, global_distance, global_mod, x, y, a, b, w, z, flag, computation, odometry, positionx, positiony, robotx, roboty
occ_bins = [-1, 0, 100, 101]
rotated = Image.fromarray(numpy.array(numpy.zeros((1,1))))
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
    angle = 2 * 180 / math.pi * math.atan( chord/ 2/ d)
    return int(angle) + 1
def motion(mod, k, distance_differ, alpha):
    global flag
    distance = float(distance_differ[k])
    linear = round( 0.26-0.26* math.exp(-1* (distance -0.3)) ,2)
    angular = 0
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
        if round(float(msg.ranges [i ]),2) == 0:
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
        point = alpha (distance_differ , k)
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
            controller = control(maps2, distance_differ , mod, k)
            if controller == 1:
                if mod == 0:
                    counter, controller = medium (mod, maps , maps2, selection, value, distance_differ, k)
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
            k = 'ERROR'
            clock = False
        
        
    return beta, k, alpha, value

def control(maps2, distance_differ, mod, k):
    angle = range(181,360) + range(0,181)
    point = 0
    z = len(distance_differ )-1
    bandiera = True
    free_space = 0
    while(bandiera):
        space_angle = []
        p_start = len(angle)/2 + mod
        point = alpha(distance_differ , z)
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

def slam(map_matrix, maps, init, odometryx, odometryy, odometryw):
    global time, positionx , positiony, robotx, roboty, obstacle
    index = init
    theta = 0 
    space = 0
    robot = 0 
    n = 2
    theta = odometryw[-1] 
    x = odometryx [-1] 
    y = odometryy[-1]
    for i in range(len(maps)/n):
        i = i*n
        maps_value = maps.values()[i]
        if maps_value > 0 and maps_value < 5 and time % 10 == 0:
            index = copy.copy(init)
            index[0] += int(x *100 + maps_value *100* math.cos((theta+i)*math.pi/180))
            index[1] += int(y*100 + maps_value *100* math.sin((theta+i) *math.pi/180))
            if map_matrix [index[0]][index[1]] != 2 and map_matrix[index[0]][index[1]] != 3: 
                map_matrix [index[0]][index[1]] = 1
                print "value %d" % (int(maps_value* 100))
            for j in range(1,int(maps_value* 100-30)):
                index = copy.copy(init)
                index[0] += int (x*100 + (maps_value* 100-j)*math.cos((theta+i) *math.pi/180))
                index[1] += int (y*100 + (maps_value* 100-j)*math.sin((theta+i) *math.pi/180))
                if map_matrix [index[0]][index[1]] != 2 and map_matrix[index [0]][index[1]] != 3:
                    map_matrix [index[0]][index [1]] = 0
    space = math.sqrt( (x-positionx)*(x-positionx) + (y-positiony) *( y-positiony) )
    robot = math.sqrt ( (x-robotx)*(x -robotx) + ( y-roboty)*( y-roboty) )
    if time == 1 or robot > 0.3:
        robotx = odometryx [-1]
        roboty = odometryy[-1]
        if space > 2:
            positionx = odometryx [-1]
            positiony = odometryy[-1]
        for i in range(13):
            for j in range(13):
                index = copy.copy(init)
                index[0]+=int( x*100+i *math.cos((theta )*math.pi/180)-j*math.sin((theta)*math.pi/ 180))
                index[1]+=int( y*100 + i *math.sin( (theta)*math.pi/ 180)+ j*math.cos((theta) *math.pi /180))
                if j == 0 and i >= 0:
                    if space > 2 or time == 1:
                        map_matrix[index[0]][index [1]] = 3
                    elif map_matrix[index [0]][index[1]] != 2 and map_matrix[index[0]][index[1]] != 3:
                        map_matrix[index[0]][index [1]] = 0
                else:
                    if space > 2 or time == 1:
                        map_matrix[index[0]][index [1]] = 2
                    elif map_matrix[index[0]][index[1]] != 2 and map_matrix[index [0]][index[1]] != 3:
                        map_matrix[index[0]][index [1]] = 0
                if j > 0:
                    index = copy.copy(init)
                    index[0]+=int(x* 100+i *math.cos ((theta)*math.pi/ 180)+ j* math.sin((theta)* math.pi/ 180) )
                    index[1]+=int( y* 100+i *math.sin( (theta)*math.pi/ 180)-j*math.cos((theta)* math.pi/ 180))
                    if space > 2 or time == 1:
                        map_matrix[index[0]][index[1]] = 2
                    elif map_matrix[index [0]][index[1]] != 2 and map_matrix[index [0]][index[1]] != 3 :
                        map_matrix[index[0]][index [1]] = 0
                if i > 0:
                    index = copy.copy(init) 
                    index[0]+=int(x* 100-i* math.cos ((theta)*math.pi/ 180)-j*math.sin ((theta)* math.pi/ 180))
                    index[1]+=int(y* 100-i*math.sin(( theta) *math.pi/ 180)+ j*math.cos(( theta)* math.pi /180))
                    if space > 2 or time == 1:
                        map_matrix[index[0]][index [1]] = 2
                    elif map_matrix[index [0]][index[1]] != 2 and map_matrix[index [0]][index[1]] != 3 :
                        map_matrix[index[0]][index [1]] = 0
                    index = copy.copy(init)
                    index[0]+=int(x* 100-i*math.cos( (theta )*math.pi/ 180)+ j*math.sin((theta)* math.pi /180))
                    index[1]+=int( y* 100-i* math.sin(( theta)*math.pi/ 180)-j*math.cos((theta)* math.pi/ 180))
                    if space > 2 or time == 1:
                        map_matrix[index[0]][index[1]] = 2
                    elif map_matrix[index [0]][index[1]] != 2 and map_matrix[index [0]][index[1]] != 3 :
                        map_matrix[index[0]][index [1]] = 0
    return map_matrix

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
        print '*'*5 + " Linear velocity = %.2f m/s , Angular velocity = %.2f rad/s" %(linear , angular) + '*'* 5
        w = numpy.sign(z)* round(float( 2*math.acos(w)* 180/ math.pi) , 2)
        print '*'*5 + "Odometry x = %.2f m , y = %.2f m, theta = %.2f degree" %(x-a, y-b, w)+'*'* 5+"\n "
        odometryx = numpy.concatenate(( odometryx , [x-a]))
        odometryy = numpy.concatenate(( odometryy, [y-b]))
        odometryw = numpy.concatenate(( odometryw, [w] ))
        odometryz = numpy.concatenate(( odometryz , [z])) 
        map_matrix = slam( map_matrix, maps2, init, odometryx, odometryy,odometryw)
    elif cnt== 2:
        move.linear.x =0.0
        move.angular.z =0.0
        pub.publish(move)
        X = []
        Y = []
        Green_X = []
        Green_Y = []
        Red_X = []
        Red_Y = []
        file_m = open("map.txt", "w")
        for i in range(len(map_matrix)):
            file_m.write(map_matrix[i])
            for j in range(len (map_matrix)):
                if map_matrix [i][j] == 1:
                    X.append(i)
                    Y.append(j)
                elif map_matrix[i][j] == 3:
                    Green_X.append(i)
                    Green_Y.append(j)
                elif map_matrix[i][j] == 2:
                    Red_X.append(i)
                    Red_Y.append(j)
        mt.plot(X , Y, 'k.')
        mt.plot( Red_X , Red_Y, 'r .')
        mt.plot(Green_X , Green_Y, 'g .')
        mt.plot( odometryx* 100+ 2500, odometryy*100+ 2500 ,' b-')
        file_m.close()
        mt.show()
    pub.publish( move )
    
def Position(msg):
    global x,y,z ,w
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    w = msg.pose.pose.orientation.w
    z = msg.pose.pose.orientation.z

def callback(msg, tfBuffer):
    global rotated

    # create numpy array
    occdata = numpy.array([msg.data])
    # compute histogram to identify percent of bins with -1
    occ_counts = numpy.histogram(occdata,occ_bins)
    # calculate total number of bins
    total_bins = msg.info.width * msg.info.height
    # log the info
    print('Width: %i Height: %i',msg.info.width,msg.info.height)
    print('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i', occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)

    # find transform to convert map coordinates to base_link coordinates
    # lookup_transform(target_frame, source_frame, time)
    trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
    cur_pos = trans.transform.translation
    cur_rot = trans.transform.rotation
    print('Trans: ' + str(cur_pos))
    print('Rot: ' + str(cur_rot))

    # get map resolution
    map_res = msg.info.resolution
    # get map origin struct has fields of x, y, and z
    map_origin = msg.info.origin.position
    # get map grid positions for x, y position
    grid_x = round((cur_pos.x - map_origin.x) / map_res)
    grid_y = round(((cur_pos.y - map_origin.y) / map_res))
    print(['Grid Y: ' + str(grid_y) + ' Grid X: ' + str(grid_x)])

    # make occdata go from 0 instead of -1, reshape into 2D
    oc2 = occdata + 1
    # set all values above 1 (i.e. above 0 in the original map data, representing occupied locations)
    oc3 = (oc2>1).choose(oc2,2)
    # reshape to 2D array using column order
    odata = numpy.uint8(oc3.reshape(msg.info.height,msg.info.width,order='F'))
    
    # set current robot location to 0
    odata[int(grid_x)][int(grid_y)] = 0
    # create image from 2D array using PIL
    img = Image.fromarray(odata.astype(numpy.uint8))
    # find center of image
    i_centerx = msg.info.width/2
    i_centery = msg.info.height/2
    # translate by curr_pos - centerxy to make sure the rotation is performed
    # with the robot at the center
    # using tips from:
    # https://stackabuse.com/affine-image-transformations-in-python-with-numpy-pillow-and-opencv/
    translation_m = numpy.array([[1, 0, (i_centerx-grid_y)],
                               [0, 1, (i_centery-grid_x)],
                               [0, 0, 1]])
    # Image.transform function requires the matrix to be inverted
    tm_inv = numpy.linalg.inv(translation_m)
    # translate the image so that the robot is at the center of the image
    img_transformed = img.transform((msg.info.height, msg.info.width),
                                    Image.AFFINE,
                                    data=tm_inv.flatten()[:6],
                                    resample=Image.NEAREST)

    # convert quaternion to Euler angles
    orientation_list = [cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    rospy.loginfo(['Yaw: R: ' + str(yaw) + ' D: ' + str(numpy.degrees(yaw))])

    # rotate by 180 degrees to invert map so that the forward direction is at the top of the image
    rotated = img_transformed.rotate(numpy.degrees(-yaw)+180)
    # we should now be able to access the map around the robot by converting
    # back to a numpy array: im2arr = np.array(rotated)

    # show image using grayscale map
    mt.imshow(rotated,cmap='gray')
    mt.draw_all()
    # pause to make sure the plot gets created
    mt.pause(0.00000000001)
    
def get_occupancy(msg):
    global occdata

    # create numpy array
    occdata = numpy.array([msg.data])
    # compute histogram to identify percent of bins with -1
    occ_counts = numpy.histogram(occdata,occ_bins)
    # calculate total number of bins
    total_bins = msg.info.width * msg.info.height
    # log the info
    rospy.loginfo('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i', occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)


class Publisher(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
    def run(self):
        osub=rospy.Subscriber('scan', LaserScan,LiDAR)
        sub1=rospy.Subscriber("odom", Odometry, Position)
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
        