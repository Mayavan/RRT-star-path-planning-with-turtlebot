#include "project5/controller.hpp"

controller::controller(ros::NodeHandle node, int step_size)
{
	pub = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
	feedback = node.subscribe("/move_base/feedback", 1000, &controller::updater, this);
	distance_threshold = (step_size * 0.05)/1.2;
}

void controller::updater(const move_base_msgs::MoveBaseActionFeedback &msg)
{
	current_x = msg.feedback.base_position.pose.position.x;
	current_y = msg.feedback.base_position.pose.position.y;
}

void controller::moveToGoal(geometry_msgs::PoseStamped point)
{
	ROS_INFO_STREAM("Point to move to [" << point.pose.position.x << ", " << point.pose.position.y << "]\n");
	point.header.frame_id = "map";
	point.header.stamp = ros::Time::now();

	pub.publish(point);
	ros::spinOnce();
}

bool controller::executePlan(std::vector<geometry_msgs::PoseStamped> plan, float final_yaw)
{
	int len = plan.size() - 1;
	ROS_INFO_STREAM("----------------Executing The Plan---------------");

	for (int i = len; i >= 0; i--)
	{
		float x1, y1, x2, y2, slope, rad;
		if (i == 0)
			rad = final_yaw;
		else
		{
			x1 = plan[i].pose.position.x;
			y1 = plan[i].pose.position.y;
			x2 = plan[i - 1].pose.position.x;
			y2 = plan[i - 1].pose.position.y;

			if (x2 == x1)
				rad = (y2 > y1) ? 1.5708 : -1.5708;
			else
			{
				slope = (y2 - y1) / (x2 - x1);
				rad = atan(slope);
			}
		}

		plan[i].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, rad);
		moveToGoal(plan[i]);

		// Wait for completion
		float distance;
		do
		{
			distance = (float)sqrt((double)pow(x1 - current_x, 2) + (double)pow(y1 - current_y, 2));
			ros::spinOnce();
		} while (distance_threshold < distance);
	}
	return true;
}