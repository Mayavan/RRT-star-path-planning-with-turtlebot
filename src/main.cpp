#include "project3/Map_manager.hpp"
#include "project3/Planner.hpp"
#include "project3/matplotlibcpp.h"

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <geometry_msgs/Twist.h>

#include <cmath>

# define DEBUG true

namespace plt = matplotlibcpp;

using namespace std;

// Structure used to store the Closed Nodes
struct Nodes
{
  vector<int> point;
  vector<float> state;
  int parent_node;
  int costToCome;
  float costToGo;
  float TotalCost;
};

// Structure used to store the OpenNodes
struct OpenNodes
{
  vector<int> point;
  vector<float> state;
  int parent_node;
  int costToCome;
  float TotalCost;
};

// function to check if the given node is an openNode and also returns the position
int is_open_node(vector<int> node, vector<OpenNodes> openNodes);
// function to check if the given node is an closed Node
bool is_closed_node(vector<int> node, vector<Nodes> nodes);
// check if the node is 5 grids close to destination
bool hasReached(vector<int> currentNode, vector<int> target_point);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "project3_node");

  ros::NodeHandle node;

  // Initialization
  Planner planner;
  Map_manager manager;

  return 0;
}