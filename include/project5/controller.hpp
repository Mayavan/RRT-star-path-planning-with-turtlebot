#pragma once

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

class controller {
 public:
  controller();
  bool moveToGoal(geometry_msgs::PoseStamped point);
};
