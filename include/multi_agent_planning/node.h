#ifndef NODE_H_
#define NODE_H_

#include "multi_agent_planning/common.h"

class Node{
	public:
			using Ptr = shared_ptr<Node>; // repalce typedefine;
			int x;
			int y;
			double yaw;
			Ptr next;
			double val;
			double cost;
			Node(){}
			Node(double yaw){
				this->yaw = yaw;
				this->next = NULL;
			}
};

#endif
