#include "ros/ros.h"
#include "std_msgs/String.h"
#include "multi_agent_planning/UpdateGoal.h"


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "test_agent_2");

	ros::NodeHandle n;
	ros::ServiceClient agent = n.serviceClient<multi_agent_planning::UpdateGoal>("update_goal");

	multi_agent_planning::UpdateGoal agent_path;

    agent_path.request.id = "agent_2";
    agent_path.request.goal.x = 6.0;
    agent_path.request.goal.y = 3.0;
	//agent_path.request.goal.theta = 0.0;

    agent.call(agent_path);

	return 0;
}