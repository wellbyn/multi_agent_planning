#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <string>
#include "multi_agent_planning/GetPlan.h"
#include "multi_agent_planning/roadmap.h"

//fisrt agent path wiil be second agent obstical;
vector<vector<int>> obstical_map(11, vector<int>(11, 0));

#define STEP_COST 10    //one step costs 10	

class Planner
{
	public:
		Planner();
	private:
		void time_callback(const ros::TimerEvent& event);
		void state_Callback(const geometry_msgs::Pose2D::ConstPtr& msg);
		bool path_callback(multi_agent_planning::GetPlan::Request &req, multi_agent_planning::GetPlan::Response &res);
		vector<Node::Ptr > reconstruct_path(Node::Ptr start, Node::Ptr end, vector<vector<Node::Ptr>> came_from);
		vector<vector<Node::Ptr>> find_path(Node::Ptr start, Node::Ptr end, vector<Node::Ptr> &map);
		double heuristic(Node::Ptr state, Node::Ptr end);
		ros::NodeHandle nh_;
		ros::Publisher pub_;
		ros::Subscriber sub_;
		ros::ServiceServer plan_service;
		// map info : start and goal
		RoadMap::map_Ptr rm = make_shared<RoadMap>();
		vector<int> map_size;
		vector<int> map_x_size;
		vector<int> map_y_size;
		
		//path
		geometry_msgs::Pose pose;
		nav_msgs::Path final_path;
		Node::Ptr start = make_shared<Node>(0);
		Node::Ptr end = make_shared<Node>(0);

		ros::Timer timer;
		std::string agent_id;
		
};

Planner::Planner()
{
	//acutal path for each agent
	pub_ = nh_.advertise<nav_msgs::Path>("get_map" , 1);
	// sub to teh agent current position
	sub_ = nh_.subscribe<geometry_msgs::Pose2D>("agent_feedback", 10, &Planner::state_Callback, this);
	// wait for the goal
	plan_service = nh_.advertiseService("get_plan", &Planner::path_callback, this);
	timer = nh_.createTimer(ros::Duration(0.1), &Planner::time_callback, this);
	// map info
	rm->buildMap();
	map_size = rm->getSize();
}


void Planner::time_callback(const ros::TimerEvent& event)
{
	pub_.publish(final_path);
}


void Planner::state_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	start->x = msg->x;
	start->y = msg->y;
	start->yaw = msg->theta;
}


double Planner::heuristic(Node::Ptr state, Node::Ptr end)
{
	int xs = state->x, ys = state->y;
	int xg = end->x, yg = end->y;
	double dist = (xg - xs)*(xg - xs) + (yg - ys)*(yg - ys);

	return dist;
}


bool Planner::path_callback(multi_agent_planning::GetPlan::Request &req, 
									 multi_agent_planning::GetPlan::Response &res)
{  
	agent_id = req.id;
	geometry_msgs::Pose2D goal = req.goal;
	
	end->x = goal.x;
	end->y = goal.y;
	end->yaw = goal.theta;

	vector<vector<Node::Ptr>> came_from = find_path(start, end, rm->map_);
	vector<Node::Ptr > node_path = reconstruct_path(start, end, came_from);

	// convert path to path message
	final_path.poses = vector<geometry_msgs::PoseStamped>(node_path.size());
	std_msgs::Header header_msg;
	header_msg.stamp = ros::Time::now();
	header_msg.frame_id = "map";
	final_path.header = header_msg;

	for(int i = 0; i < node_path.size(); i++)
	{
		pose.position.x = node_path[i]->x;
		pose.position.y = node_path[i]->y;
		final_path.poses[i].pose = pose;
	}
	// the start and goal of second agent is still accessible for
	// second agent, need reset here.
	obstical_map[start->x][start->y] = 0;
	obstical_map[start->x][start->y] = 0;

    return true;
}


vector<Node::Ptr> Planner::reconstruct_path(Node::Ptr start, Node::Ptr end, vector<vector<Node::Ptr>> came_from)
{
	vector<Node::Ptr> path;
	path.push_back(end);

	Node::Ptr current = came_from[end->x][end->y]; 
	int x = current->x;
	int y = current->y;
	while(x != start->x || y != start->y){
		path.push_back(current);
		current = came_from[x][y];
		x = current->x;
		y = current->y;
	}
	path.push_back(current);

	return path;
}


vector<vector<Node::Ptr>> Planner::find_path(Node::Ptr start, Node::Ptr end, vector<Node::Ptr> &map)
{
	// Here use basic A* search , also can be used for higher dimention.
	// define the path
	auto comp = [](const Node::Ptr n1, const Node::Ptr n2)
	{
		return (n1->cost) < (n2->cost);
	};

	vector<vector<int>> closed(map_size[0], vector<int>(map_size[1], 0));
	vector<vector<Node::Ptr>> came_from(map_size[0], vector<Node::Ptr>(map_size[1]));

	start->val = 0;
	start->cost = start->val + heuristic(start, end);

	closed[start->x][start->y] = 1;
	came_from[start->x][start->y] = start;

	vector<Node::Ptr> q = {start}; // open list 
	bool finished = false;

	while(!q.empty())
	{
        std::sort(q.begin(), q.end(), comp);
        //grab first elment
        Node::Ptr curr = q[0];
        //pop first element
	    q.erase(q.begin()); 

	    // the fist agent path will be second agent's obstical
	    obstical_map[curr->x][curr->y] = 100;
		//goal found
		if(curr->x == end->x && curr->y == end->y)
		{
			return came_from;
		}

		for(int i = 0; i < map.size(); i++)
		{
			if(curr->x == map[i]->x && curr->y == map[i]->y)
			{
				// find the neightboor of inside map;
				Node::Ptr nb = map[i];
				double g = nb->val;
				while(nb->next != NULL)
				{
					nb = nb->next;
					int x2 = nb->x;
					int y2 = nb->y;
					double g2 = g + STEP_COST; // step cost

					if(obstical_map[nb->x][nb->y] != 100 && closed[nb->x][nb->y] == 0)
					{
						nb->val = g2; // current g
						nb->cost = g2 + heuristic(nb, end);
						closed[x2][y2] = 1;
						q.push_back(nb);
						came_from[x2][y2] = map[i];
					}
				}
			}
		}
	}

	ROS_INFO("no valid path.");
	return came_from;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    Planner planner;
    ros::spin();
    return 0;
}
