#pragma once

#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <opencv2/opencv.hpp>
#include <math.h>

class map_manager {
 public:
  map_manager(ros::NodeHandle nh);
  int get_state(int x, int y);
  void show_image();
  bool checkObstacle(std::vector<int> grid);
  std::vector<int> computeGridPosition(std::vector<float> position);

 private:
  nav_msgs::OccupancyGrid map;
  cv::Mat image;
};
