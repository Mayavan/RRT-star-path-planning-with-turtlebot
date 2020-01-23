#include "project5/Map_manager.hpp"
#include "project5/Planner.hpp"
#include "project5/matplotlibcpp.h"
#include "project5/controller.hpp"

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <geometry_msgs/Twist.h>

#include <cmath>

#define DEBUG false
#define STEP_SIZE 6

namespace plt = matplotlibcpp;
using namespace std;


int main(int argc, char  *argv[])
{
  ros::init(argc, argv, "rrt_demo");

  ros::NodeHandle node("~");

  std::string map_location;
  node.getParam("map_file", map_location);

  std::cout << map_location << ".png" << std::endl;

  // Initialization
  Map_manager manager(std::string(map_location+".png"));
  Planner planner(manager, STEP_SIZE);
  controller control(node, STEP_SIZE);

  // Initialized position corresponds to the spawn position in gazebo environment
  vector<float> start_state, end_state;
  vector<float> start_point, target_point;

  start_state.push_back(-0.5);
  start_state.push_back(-2);

  end_state.push_back(0.5);
  end_state.push_back(2.0);
  end_state.push_back(0);

  float final_yaw = end_state[2];

  target_point = manager.computeGridCoordinate(end_state);
  start_point = manager.computeGridCoordinate(start_state);

  if (DEBUG)
  {
    cout << "Start Point: " << start_point[0] << ", " << start_point[1] << endl;
    cout << "End Point: " << target_point[0] << ", " << target_point[1] << endl;
  }

  // Make the plan with RRT
  std::vector<geometry_msgs::PoseStamped> plan;
  plan = planner.makePlan(start_point, target_point);
  

  int counter = 0;

  ROS_INFO_STREAM("Plotting MAP with Global Plan");
  // Plot the map
  for(int i=0;i<384;i++)
    for(int j=0;j<384;j++)
      if(manager.get_state(i,j)>150)
      {
        vector<double> x,y;
        x.push_back((i*0.05) - 10);
        x.push_back((i*0.05) - 10+0.01);
        y.push_back((j*0.05) - 10);
        y.push_back((j*0.05) - 10+0.01);
        plt::plot(x, y);
        if (DEBUG) cout<<"Printing"<<counter++<<endl;
      }      
  

  // Plot the plan
  vector<double> x,y;
  for(int i=plan.size()-1; i >= 0; i--)
  {
    x.push_back(plan[i].pose.position.x);
    y.push_back(plan[i].pose.position.y);
  }
  plt::plot(x, y);
  plt::xlim(-10, 10);
  plt::ylim(-10, 10);
  plt::show();
  
  control.executePlan(plan, final_yaw);

  return 0;
}