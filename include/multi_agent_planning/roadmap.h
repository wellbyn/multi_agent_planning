#ifndef ROAD_MAP_H_
#define ROAD_MAP_H_

#include "multi_agent_planning/common.h"
#include "multi_agent_planning/node.h"


class RoadMap
{
	public:
        RoadMap(const int& m = 11, const int& n = 11);
		RoadMap(const RoadMap& road_map);//copy constructor
		RoadMap& operator=(const RoadMap& road_map); //assignment operator
		RoadMap(RoadMap&& road_map);//move constructor
		RoadMap& operator=(const RoadMap&& road_map);// move assignment operator

		~RoadMap() = default;

        using map_Ptr = shared_ptr<RoadMap>;
        void buildMap();//build a map 
        vector<Node::Ptr> map_;
        vector<int> getSize(); // get the map shape
        vector<int> getXSize(); // get the map x-aixe size
        vector<int> getYSize(); // get the map y-aixe size

	private:
		int m_;
		int n_;
};

#endif
