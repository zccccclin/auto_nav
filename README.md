                                                              # auto_nav
                                                          EG2310 Module content
                                              Autonomous Navigation and Firing with turtlebot 3

                                                          Project requirement: 
Laptop:
Ubuntu 16.04  http://releases.ubuntu.com/16.04/
ROS 1 Kinetic http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#install-ros-on-remote-pc
Dependent ROS packages http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#install-dependent-ros-packages
OPENCV sudo apt-get install python-opencv
Soundplay sudo apt-get install ros-kinetic-sound-play

Turtlebot3 burger with rasberry pi 3B+:
Install Linux based http://emanual.robotis.com/docs/en/platform/turtlebot3/raspberry_pi_3_setup/#install-linux-based-on-raspbian



                                                       Folder navigation guide
There are 3 specific folders for three tasks that we went through during the module.

/Lab 
This folder contains all the lab assignments and homeworks that we completed throughout the semester

/Finalnav 
This folder contains all the codes related to autonomous navigation and mapping on turtlebot 3
    /wallfollow is the initial ideas based off online algorithm to navigate the full unknown map.
auto_navonline.py is the initial content that combines many idea from wall following and obstacle avoidance
auto_navonline2.py is the final code produced that combined all the ideas to fullfill the final objective of the module.

/aimfire
This folder contains all the codes related to auto aim and fire at colored target on turtlebot 3
    /trial is the folder that contains all the testing with individual components, servo, camera
finalaimfire.py is the final code that integrated everything from movement to detection of target to aimming and firing.
