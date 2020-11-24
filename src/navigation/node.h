
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"

#include "vector_map/vector_map.h"


#include "amrl_msgs/AckermannCurvatureDriveMsg.h"

#include <iostream>
#include <string> 
#include <math.h>
#include <vector>

// #define GRID_RES 0.25

using geometry::line2f;
using namespace std;
using Eigen::Vector2f;

class Node {
public:
	int i, j;
	float g, h;	
	string parent;	
	bool intersects;

	Node(int i, int j);
	Node()
	{

	}
	Node(int i, int j,const Node& goal);

	string GetState();

	void NodeIntersectsMap(vector<line2f> map_lines);
	// needs to be implemented
	float GetPriority();
	float GetValue(vector<line2f> map_lines);
	friend bool operator == (Node const &, Node const &);
	
};

