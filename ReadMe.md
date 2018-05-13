
# RRT* implementation on turtlebot3

RRT* on Project 2 is done in Matlab

Project 5 is done in ROS and Gazebo with C++

 * Turtlebot 3 was used.
 * The world is first converted to 2D map with SLAM.
 * The radius of the robot is traced around the obstacle to plan for a point robot. (Matlab was used).
 * The main code plans the path and publishes positions to the amcl local planner

# Installation

1. Running code Requires OpenCV installation:

 https://docs.opencv.org/3.4.0/d7/d9f/tutorial_linux_install.html

```
 sudo apt-get install build-essential
 sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
 sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
```
 2. Install matplotlib for python for the plot to work.

# To run the code

## RRT* ON PROJECT 2 

 * Run main.m file in RRT* on project 2 folder.

## RRT* ON Turtlebot3 
1. Follow this link on guide to SLAM the workspace first:

http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/

2. Start master:
```
$ roscore
```

3. Launch simulation or the robot. To run with simulation,
```
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch 
```
To run with turtlebot3 bringup the robot.

4. Start navigation package
```
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/mayavan/planning_ws/src/project5/maps/map.yaml
```
5.  Start Rviz for visulization
```
$ rosrun rviz rviz -d `rospack find turtlebot3_navigation`/rviz/turtlebot3_nav.rviz
```
6.  Localize with 2D pose estimate

7. Run the planner after setting the start and end point in the code and catkin_make.
```
$ rosrun package_name Main
```



