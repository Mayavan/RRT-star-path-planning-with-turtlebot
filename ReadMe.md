##############################
Overview
##############################
A* on Project 2 is done in Matlab

Project 3 is done in ROS and Gazebo with C++

 * Turtlebot 3 was used.
 * The world is first converted to 2D map with SLAM.
 * The radius of the robot is traced around the obstacle to plan for a point robot. (Matlab was used).
 * The main code plans the path and publishes the velocity in order at an interval of 0.5 seconds. 

*******************************
******* A* ON PROJECT 2 *******
*******************************
 * Run main.m file

*******************************
********* PROJECT 3 ***********
*******************************
##########################################
Running code Requires OpenCV installation:
##########################################
 https://docs.opencv.org/3.4.0/d7/d9f/tutorial_linux_install.html

 sudo apt-get install build-essential
 sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
 sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

####################################
Launch the RRL world with robot pose 
####################################

To open the RRL in gazebo, run the command:

 $ roslaunch turtlebot3_gazebo turtlebot3_rrl.launch

####################
To run the algorithm
####################

 $ rosrun project3 Main

 The twist.txt contains the twist values to be applied at an interval of 0.5 seconds

 $ git init
 $ git add README.md
 $ git commit -m "first commit"
 $ git remote add origin https://github.com/Mayavan/TurtleBot_Astar.git
 $ git push -u origin master

######################################
References:
######################################
Follow this link on guide to SLAM the workspace:

http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/

Run the map_server to create a publisher node:

$ rosrun map_server map_server ~/planning_ws/src/project3/map.yaml


SEND TO AMCL MOVE_BASE

1) start roscore
2) start map server in project root

rosrun map_server map_server maps/map.yaml

3) roslaunch turtlebot3_gazebo turtlebot3_world.launch    or    bringup

4) Start amcl

roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/mayavan/planning_ws/src/project5/maps/map.yaml

5) Start Rviz

rosrun rviz rviz -d `rospack find turtlebot3_navigation`/rviz/turtlebot3_nav.rviz

6) Localize

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch




