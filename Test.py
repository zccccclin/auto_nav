def pick_direction():
    global occdata
    global im2arr
    global i_centerx
    global i_centery
    global laser_range
    from math import sqrt, tan
    global laser_range
    line_increment = 10
    threshold = 1 

    # publish to cmd_vel to move TurtleBot
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # stop moving
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)
	
#Part to avoid obstacles
    def identify_openings(laser_range): #return a dict of potential openings with its distances
        opening_list = [] # will make [[30, 60], [120, 150]] (means there's an opening between angle 30 and 60, and angle 120 and 150
        for angles in range(len(laser_range)):
	    initial_distance = laser_range[angles-1]
	    if initial_distance - laser_range[angles] > 0:
		    starting_angle = angles
	for angles in range(starting_angle, 361 + starting_angle):
	    if abs(initial_distance - laser_range[angles]) > 0:
	        opening_list.append(angles)
	return opening_list
	

    def choose_best_opening(opening_list):
	if len(opening_list) == 2:
	    return abs(opening_list[0] - opening_list[1])//2 + opening_list[0]
	else: 
	    list_of_potential_angles = laser_distances.keys()
	    dict_of_angles_and_differences = {}
	    
	    for angles in laser_distances.keys():
		angle_differences = abs(laser_distances[angles] - goal)
		dict_of_angles_and_differences.update({angle_differences : angles})

	    min_angle_differences = min(dict_of_angles_and_differences.keys())
	    lr2i =  dict_of_angles_and_differences[min_angle_differences]


	return lr2i
            
    lr2i = choose_best_opening(identify_openings(laser_range, line_increment), goal)
