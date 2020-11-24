
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

#define GRID_RES 0.25

using geometry::line2f;
using namespace std;
using Eigen::Vector2f;

class Node {
public:
	int i, j;
	float g, h;	
	Node* parent;	

	Node(int i, int j) {
		this->i = i;
		this->j = j; 
		this->h = 0;
		//need to implement distance from end point as h
	}
	Node()
	{

	}
	Node(int i, int j,const Node& goal) {
		this->i = i;
		this->j = j; 

		float diff_i = abs(goal.i - this->i);
		float diff_j = abs(goal.j - this->j);
		float min_diff = diff_i;
		if(diff_j<min_diff)
		{
			min_diff = diff_j;
		}
		float max_diff = diff_j + diff_i - min_diff;
		this->h = min_diff*1.414 + (max_diff-min_diff);
		//need to implement distance from end point as h
	}

	// string GetState() {
	// 	pair<int,int> state;
	// 	state.first = this->i;
	// 	state.second = this->j;
	// 	// string istate = to_string(this->i);
	// 	// string jstate = to_string(this->j);
	// 	// string comma = ",";
	// 	// string state = istate + comma + jstate; 
	// 	return state;
	// }

	bool NodeIntersectsMap(vector<line2f> map_lines) {

		for(unsigned int i = 0; i < map_lines.size(); i++) {
			line2f map_line = map_lines[i];
			float left_x, right_x, top_y, bottom_y;
			left_x = this->i * GRID_RES;
			right_x = (this->i + 1) * GRID_RES;
			bottom_y = this->j * GRID_RES;
			top_y = (this->j + 1) * GRID_RES;
			const line2f line1(left_x, bottom_y, left_x, top_y);
			const line2f line2(left_x, top_y, right_x, top_y);
			const line2f line3(right_x, top_y, right_x, bottom_y);
			const line2f line4(right_x, bottom_y, left_x, bottom_y);

			if(map_line.Intersects(line1) || map_line.Intersects(line2) ||
				map_line.Intersects(line3) || map_line.Intersects(line4)) {
				return true;
			}
		}

		return false;

	}
	//value at a state
	// needs to be implemented
	float GetValue(vector<line2f> map_lines) {
		return 0;
		// float min_dist = std::numeric_limits<float>::max();
		// for(unsigned int i=0; i<map_lines.size(); i++) {
		// 	line2f line = map_lines[i];
		// 	Vector2f point( (this->i+0.5) * GRID_RES, (this->j+0.5) * GRID_RES);
		// 	float dist_points = pow(line.p0.x()-line.p1.x(), 2) + pow(line.p0.y()-line.p1.y(), 2);

			 
			
		// 	if(dist_points == 0) {
		// 		return -(pow(line.p0.x()-point.x(), 2) + pow(line.p0.y()-point.y(), 2));
		// 	}
		// 	float p0_point_x = point.x() - line.p0.x();
		// 	float p0_point_y = point.y() - line.p0.y();

		// 	float p0_p1_x = line.p1.x() - line.p0.x();
		// 	float p0_p1_y = line.p1.y() - line.p0.y();
		// 	float dot = p0_p1_x * p0_point_x + p0_p1_y * p0_point_y;
		// 	const float t = max((float)0.0, min((float)1.0, dot/dist_points));

		// 	float x_proj = line.p0.x() + t * p0_p1_x;
		// 	float y_proj = line.p0.y() + t * p0_p1_y;

		// 	float dist = (pow(x_proj - point.x(), 2) + pow(y_proj - point.y(), 2));

		// 	if(dist<min_dist) {
		// 		min_dist = dist;
		// 	}
			
		// }
		// return -min_dist;
	}
	friend bool operator == (Node const &, Node const &);
	
};

bool operator == (Node const &  first,Node const & second)
{
	return (first.i==second.i) && (first.j==second.j);
}