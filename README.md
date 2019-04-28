# multi_agent_planning

## Goal:
- the project is to create a planner can perform multi-agent planning in a 10x10 map.

## Test Process:
1. catkin_make
2. source devel/setup.bash
3. roslaunch multi_agent_planning multi_agent_planning.launch
4. rosrun multi_agent_planning test_agent_1 to create path from (2, 0, 0) to (2, 5, 0)
5. rosrun multi_agent_planning test_agent_2 to create path from (0, 3, 0) to (6, 3, 0)

## Method:
1. Build Map:
- this part is designed in the "roadmap.cpp" node, which is contructed as class with "map_" member variable, also I define
"node.h" as the nodes to help construct the map, the idea of building the map is just based on the idea of sparse matrix for
graph search problem, which is faster and save more space than adjacent matrix.

2. Planner:
- The planner creates the shortest path from start position to goal position, and save the path in path files named by serial id.

3. Agent:
- The agent sets the start position by serial id and calls planner to create the path.


