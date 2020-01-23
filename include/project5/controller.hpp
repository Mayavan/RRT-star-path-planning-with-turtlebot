#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>

class controller
{
public:
  controller(ros::NodeHandle node, int step_size);
  void moveToGoal(geometry_msgs::PoseStamped point);
  bool executePlan(std::vector<geometry_msgs::PoseStamped> plan, float final_yaw);

protected:
  void updater(const move_base_msgs::MoveBaseActionFeedback &msg);

private:
  ros::Publisher pub;
  ros::Subscriber feedback;

  float current_x, current_y;
  float distance_threshold;
};
