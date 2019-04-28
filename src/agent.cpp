#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Pose2D.h>
#include "multi_agent_planning/UpdateGoal.h"
#include "multi_agent_planning/GetPlan.h"
#include "multi_agent_planning/common.h"

class Agent
{
	public:
		Agent();
	private:
		bool goal_callback(multi_agent_planning::UpdateGoal::Request &req, 
						   multi_agent_planning::UpdateGoal::Response &res);

		void time_callback(const ros::TimerEvent& event);

		ros::NodeHandle nh_;
		ros::Publisher pub_;
		ros::ServiceServer goal_;
		ros::ServiceClient path_;
		geometry_msgs::Pose2D state;
		multi_agent_planning::GetPlan agent_path;
		ros::Timer timer;
};


Agent::Agent()
{
	pub_ = nh_.advertise<geometry_msgs::Pose2D>("agent_feedback" , 1);
	goal_ = nh_.advertiseService("update_goal", &Agent::goal_callback,this);
	path_ = nh_.serviceClient<multi_agent_planning::GetPlan>("get_plan");
	timer = nh_.createTimer(ros::Duration(0.1), &Agent::time_callback, this);
}


void Agent::time_callback(const ros::TimerEvent& event)
{
	// publish the agent state
	pub_.publish(state);
}


bool Agent::goal_callback(multi_agent_planning::UpdateGoal::Request &req, 
						  multi_agent_planning::UpdateGoal::Response &res)
{
	state.x = req.start.x;
	state.y = req.start.y;
	state.theta = req.start.theta;
    
	//Update state position first
	pub_.publish(state);

    agent_path.request.id = req.id;
	agent_path.request.goal.x = req.goal.x;
	agent_path.request.goal.y = req.goal.y;

	path_.call(agent_path);

	return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "agent");
    Agent agent;
    ros::spin();
    return 0;
}
