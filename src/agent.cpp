#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Pose2D.h>
#include "multi_agent_planning/UpdateGoal.h"
#include "multi_agent_planning/GetPlan.h"
#include "multi_agent_planning/common.h"

#define NODE_NAME "agent"

class Agent
{
	public:
		Agent();
	private:
		bool goal_callback(multi_agent_planning::UpdateGoal::Request &req, 
						   multi_agent_planning::UpdateGoal::Response &res);

		void time_callback(const ros::TimerEvent& event);

		void select_start_position(const std::string id);

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


void Agent::select_start_position(const std::string id)
{
	std::string start_x_param=(std::string)NODE_NAME+"/"+id+"/start_x";
	std::string start_y_param=(std::string)NODE_NAME+"/"+id+"/start_y";

	if(ros::param::has(start_x_param)&&ros::param::has(start_y_param))
	{
		ros::param::get(start_x_param, state.x);
		ros::param::get(start_y_param, state.y);
		state.theta = 0.0;
	}
    else
    {
        state.x = 0.0;
        state.y = 0.0;
        state.theta = 0.0;
    }

    //Update state position first
    pub_.publish(state);
}


void Agent::time_callback(const ros::TimerEvent& event)
{
    // publish the agent state
    pub_.publish(state);
}


bool Agent::goal_callback(multi_agent_planning::UpdateGoal::Request &req, 
                          multi_agent_planning::UpdateGoal::Response &res)
{
    select_start_position(req.id);

    agent_path.request.id = req.id;
    agent_path.request.goal.x = req.goal.x;
    agent_path.request.goal.y = req.goal.y;

    path_.call(agent_path);

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    Agent agent;
    ros::spin();
    return 0;
}
