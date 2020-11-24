//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include <iostream>
#include <math.h>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <algorithm>
using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;



namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

const float car_width = 0.281;
const float car_length = 0.535;
const float car_height = 0.15;
const float wheel_base = 0.324;
const float clearance_length = 0.2;
const float clearance_width = 0.1;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
	// nav_goal_loc_ = loc;
	// nav_goal_angle_ = angle;
}


void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
	// not being called
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
	robot_loc_ = loc;
	robot_angle_ = angle;
	robot_vel_ = vel;
	robot_omega_ = ang_vel;
	// std::cout<<"robot_loc_.x() : "<<robot_loc_.x()<<", robot_loc_.y() : "<<robot_loc_.y()<<"robot_angle_ : "<<robot_angle_<<"\n";
}

void Navigation::ObservePointCloud(const vector<Vector2f> cloud,
                                   double time) {
	point_cloud_ = cloud;
}

float Navigation::CalcPointFPL(const Vector2f obstacle, float curvature, float &final_x, float &final_y, bool debug) { 
if(abs(curvature) < 0.0001) {
		if(nav_goal_loc_.x()<0)
		{
			final_x = 0;
			final_y = 0;
			return 0;
		}

		float obs_x, obs_y, width_clearance, length_end;
		obs_x = obstacle.x() + LIDAR_DIST;
		obs_y = obstacle.y();

		width_clearance = car_width/2 + clearance_width;
		length_end = (wheel_base + car_length)/2 + clearance_length;
		// need to consider lidar distance ?
		// if(debug)
		// {
		// 	cout<<"obs_x : "<<obs_x<<"length_end : "<<length_end<<endl;
		// 	cout<<"obs_y : "<<obs_y<<"width_clearance : "<<width_clearance<<endl;
		// }
		if(obs_y <= width_clearance && obs_y>=-width_clearance && obs_x<length_end){
			final_x = 0;
			final_y = 0;
			return 0;
		}
		else if(obs_y <= width_clearance && obs_y>=-width_clearance && obs_x>length_end) {
			final_x = obs_x - length_end;
			final_y = 0;
			// if(debug)
			// {
			// 	cout<<"inter final x : "<<final_x<<endl;
			// }

			if(final_x>nav_goal_loc_.x())
			{
				final_x = nav_goal_loc_.x();
				return nav_goal_loc_.x();
			}
			
			return obs_x - length_end;
		} else {
			// if(debug)
			// {
			// 	cout<<"in2"<<endl;
			// }
			final_x = nav_goal_loc_.x();
			final_y = 0;
			
			return nav_goal_loc_.x();
		}
	} else {
		
		float radius = 1/curvature;
		float rmax, rs, rmin, r_obstacle;
		float length_end;
		float obs_x, obs_y;
		length_end = (car_length + wheel_base)/2 + clearance_length;
		obs_x = obstacle.x() + LIDAR_DIST;
		obs_y = obstacle.y();
	 
		rmax = sqrt(pow(length_end, 2) + pow(car_width/2 + clearance_width + abs(radius), 2));
		rmin = abs(radius) - (car_width/2 + clearance_width);
		rs = sqrt(pow(length_end, 2) + pow(abs(radius) - (car_width/2+clearance_width), 2));

		r_obstacle = sqrt(pow(obs_x, 2) + pow(obs_y-radius, 2));
		

		float theta1, theta2, dotproduct1;
		dotproduct1 = -radius * (obs_y - radius);
		theta1 = acos(dotproduct1/(abs(radius) * r_obstacle));
		if(obs_x<0) {
			theta1 = 2 * M_PI - theta1;
		}


		// float theta_max, dotproduct;
		// bool in =false;
		// // r_goal = sqrt(pow(nav_goal_loc_.x(), 2) + pow(0 - radius, 2));
		// dotproduct = abs(radius)/sqrt((radius)*(radius) + nav_goal_loc_.x()*nav_goal_loc_.x());
		// if(nav_goal_loc_.x()>=0) {
		// 	theta_max = acos(dotproduct);
		// } else {
		// 	theta_max = 2 * M_PI - acos(dotproduct);
		// }

		// // std::cout<<"Theta max: "<<theta_max<<", Theta_1: "<<theta1<<"\n";

		// if (theta1 > theta_max)
		// {
		// 	in = true;
		// 	theta1 = theta_max;
		// }


		float theta_max, dotproductMax, goal_x, goal_y;
		goal_x = nav_goal_loc_.x();
		goal_y = nav_goal_loc_.y();
		dotproductMax = -radius * (goal_y - radius)/(abs(radius) * sqrt(goal_x*goal_x + (goal_y-radius)*(goal_y-radius)));
		theta_max = acos(dotproductMax);


		if(goal_x<0) {
			theta_max = 2 * M_PI - theta_max;
		}
		// theta_max = 2 * M_PI;


		

		if(rmin <= r_obstacle && r_obstacle <= rs) {
			// point hitting inner side

			float theta_final;

			theta2 = acos(rmin/r_obstacle);
			theta_final = min(theta1-theta2, theta_max);
			
			
			final_y = radius - radius * cos(theta_final);
			final_x = abs(radius) * sin(theta_final);
			
			return abs(radius * (theta_final));

			
		} else if(rs <= r_obstacle && r_obstacle <= rmax) {
			// point hitting front side
			float theta_final;
			theta2 = asin((length_end)/(r_obstacle));

			
			theta_final = min(theta1-theta2, theta_max);
			
			final_y = radius - radius * cos(theta_final);
			final_x = abs(radius) * sin(theta_final);
			
			return abs(radius * (theta_final));
		} else {
			
			final_y = radius - radius * cos(theta_max);
			final_x = abs(radius) * sin(theta_max);
			// std::cout<<"THeta_max: "<<theta_max<<"CUrvature: "<<curvature<<"Final x: "<<final_x<<endl;
			return abs(radius * (theta_max));
			
		}

		 
	}
	
}	

float Navigation::CalculateScore(float fpl, float clearance, float goal_dist) {
	return 10*fpl-goal_dist;
}

float Navigation::CalcGoalDistance(float x_goal, float y_goal, float x_final, float y_final) {
	return sqrt(pow(x_goal-x_final, 2) + pow(y_goal-y_final, 2));
}

vector<float> Navigation::SelectCurvature(vector<Vector2f> obstacles){
	// float cmin, cmax, step, 
	float max_score, max_score_fpl, max_score_curv;
	// float curvatures[] = {-15,-12,-10,-8,-6,-1,0.5,0,-0.5,1,6,8,10,12,15};
	float curvatures[] = {-6,-5,-4,-3,-2.5,-2,-1.5,-0.9,-0.7,-0.5,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.5,0.7,0.9,1.5,2,2.5,3,4,5,6};

	// cmin = -6.0;
	// cmax = 6.0;
	// step = 0.3;
	max_score = -std::numeric_limits<float>::max();
	// std::cout<<"min : "<<max_score<<"\n";
	max_score_fpl = -1;
	max_score_curv = 0;
	// std::cout<<"selecting curvature\n";
	
	// for(float c=cmin; c<=cmax; c=c+step){
	// 	// visualization::ClearVisualizationMsg(local_viz_msg_);
	// 	if(c!=0)
	// 	{
	// 		visualization::DrawArc(Vector2f(0, 1/c), abs(1/c),-3.14,3.14, 0x3b72d3, local_viz_msg_);
	// 	}
	// 	// viz_pub_.publish(local_viz_msg_);
	// }

	// for(float c=cmin; c<=cmax; c=c+step){
	for(unsigned int current=0; current<sizeof(curvatures)/sizeof(float); current++){
		float c = curvatures[current];
		// std::cout<<"curvature : "<<c<<",max_score_fpl : "<<max_score_fpl<<"\n";
		float min_fpl = std::numeric_limits<float>::max();
		float min_fpl_x, min_fpl_y;
		float final_x,final_y;
		min_fpl_x = -1;
		min_fpl_y = -1;
		int min_obstacle = -1;
		for(unsigned int i=0; i<obstacles.size(); i++){
			float fpl = CalcPointFPL(obstacles[i], c,final_x,final_y,false);
			// std::cout<<"i : "<<i<<", fpl : "<<fpl<<", min_fpl : "<<min_fpl<<"\n";
			if(fpl <= min_fpl) {
				min_obstacle = i;
				min_fpl = fpl;
				min_fpl_x = final_x;
				min_fpl_y = final_y;
			}
		}
		if (min_obstacle!=-1)
		{
			// std::cout<<"c : "<<c<<"min_i : "<<min_obstacle<<"\n";
			// std::cout<<"obstacle x : "<<obstacles[min_obstacle].x()<<", obstacle y : "<<obstacles[min_obstacle].y()<<"\n";
			CalcPointFPL(obstacles[min_obstacle], c,final_x,final_y,true);
		}
		//compare score here
		// might be sketchy
		// std::cout<<"min_fpl_x : "<<min_fpl_x<<", "<<min_fpl_y<<"\n";
		float distance_goal = CalcGoalDistance(nav_goal_loc_.x(), 0, min_fpl_x, min_fpl_y);
		float cur_score = CalculateScore(min_fpl, 0, distance_goal);
		// std::cout<<"min_fpl_x : "<<min_fpl_x<<", min_fpl_y : "<<min_fpl_y<<"\n";
		// std::cout<<"curvature : "<<c<<" cur_score: "<<cur_score<<", max_score : "<<max_score<<" max_score_c : "<<max_score_curv<<"\n";
		// std::cout<<"distance_goal : "<<distance_goal<<", min_fpl : "<<min_fpl<<"\n\n";
		if(cur_score > max_score){
			max_score = cur_score;
			max_score_fpl = min_fpl;
			max_score_curv = c;
		}
	}
	// probably 1D TOC 
	vector<float> ret_val;
	ret_val.push_back(max_score_fpl);
	ret_val.push_back(max_score_curv);
	return ret_val;
}

void Navigation::Run() {

	vector<float> ret_vals = SelectCurvature(point_cloud_);
	nav_goal_loc_ = Vector2f(10,0);
	// float v = OneDTOC(prev_velocity, 1, 4, -4, ret_vals[0]);
	// prev_velocity = v;

	drive_msg_.velocity = 0.5;
	// drive_msg_.curvature = ret_vals[1];
	// drive_msg_.velocity = 0.1;
	drive_msg_.curvature = -8;
	drive_pub_.publish(drive_msg_);
}

//a_min is -ve i.e maximum decelertion
//u_max is maximum speed allowed
//a_max is +ve i.e maximum acceleration
float Navigation::OneDTOC(float u,float u_max,float a_max,float a_min,float s)
{
	float timestep = 1.0/20;
	float s_min;
	float error_margin = 0;
	s_min = u*u/(2*-a_min);
	if(s<0.02)
	{
		return 0;
	}
	else if (s<=s_min-error_margin)
	{
		// std::cout<<"in\n";
		float out = u - (u*u/(2*s))*timestep;
		if(out<0)
		{
			std::cout<<"in2";
			out = 0;
		}
		return out;
		// return -1;
	}
	else if(u<u_max)//case 1 where the car isn't in maximum speed
	{
		return u + a_max*timestep;
	}
	else if(s>s_min)//case 2 where car is in maximum speed and doesn't require to decelerate yet
	{
		return u;
	}
	else//case 3 where car needs to decelerate
	{
		return u + a_min*timestep;//decelrated value
	}
}

}  // namespace navigation
