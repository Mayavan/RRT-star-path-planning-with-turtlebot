#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

#include "project5/map_manager.hpp"

using namespace std;

int main(int argc, char **argv) {

  ros::init(argc, argv, "project5_node");

  ros::NodeHandle node;

  // Initialization
  map_manager manager(node);
  manager.show_image();

  return 0;
}