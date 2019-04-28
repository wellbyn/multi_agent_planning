#include "multi_agent_planning/roadmap.h"

RoadMap::RoadMap(const int& m, const int& n): m_(m),n_(n){}

vector<int> RoadMap::getSize()
{
	vector<int> map_size = {m_, n_};
	return map_size;
}

vector<int> RoadMap::getXSize()
{
	vector<int> x_size = {m_};
	return x_size;
}

vector<int> RoadMap::getYSize()
{
	vector<int> y_size = {n_};
	return y_size;
}

void RoadMap::buildMap()
{
	// initial all the nodes in the map
	for(int i = 0; i < m_; i++){
		for(int j = 0; j < n_; j++){
			Node::Ptr nd = make_shared<Node>(0);
			nd->x = i;
			nd->y = j;
			nd->yaw = 0;
			map_.push_back(nd);
		}
	}

	// create sparse matrix to store the neighboors of the nodes
	for(int i = 0; i < m_* n_; i++){
		Node::Ptr dummy = map_[i];
		Node::Ptr node = map_[i];

		for(int j = 0; j < m_* n_; j++){
			if(i != j){
				// right
				if(dummy->x + 1 == map_[j]->x && dummy->y == map_[j]->y){
					Node::Ptr nr = make_shared<Node>(0);
					nr->x = map_[j]->x;
					nr->y = map_[j]->y;
					node->next = nr;
					node = node->next;
				}

				//left
				if(dummy->x - 1 == map_[j]->x && dummy->y == map_[j]->y){
					Node::Ptr nl = make_shared<Node>(0);
					nl->x = map_[j]->x;
					nl->y = map_[j]->y;
					node->next = nl;
					node = node->next;
				}

				// up
				if(dummy->x == map_[j]->x && dummy->y + 1 == map_[j]->y){
					Node::Ptr nu = make_shared<Node>(0);
					nu->x = map_[j]->x;
					nu->y = map_[j]->y;
					node->next = nu;
					node = node->next;
				}
				// down
				if(dummy->x == map_[j]->x && dummy->y - 1== map_[j]->y){
					Node::Ptr nd = make_shared<Node>(0);
					nd->x = map_[j]->x;
					nd->y = map_[j]->y;
					node->next = nd;
					node = node->next;
				}
				node->next = NULL;
			}
		}
	}
}
