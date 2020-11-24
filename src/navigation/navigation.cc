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
//
using geometry::line2f;
//
using namespace math_util;
using namespace ros_helpers;

#define GRID_RES 0.25
#define LIDAR_DIST 0
#define LIMIT_ASTAR 0.1
#define AStar_recompute 35

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
const float clearance_length = 0.1;
const float clearance_width = 0.05;
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
  robot_loc_init_ = false;
  inside_run = false;
  goal_set = false;
  map_.Load("maps/GDC1.txt");

}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
	// nav_goal_loc_ = loc;
	// nav_goal_angle_ = angle;
	nav_goal_loc_global = loc;
	cout<<"set goal"<<endl;
	goal_set = true;
	if(inside_run)
	{	
		Node goal(((int) ( nav_goal_loc_global.x()/GRID_RES )  ),((int) ( ( nav_goal_loc_global.y()/GRID_RES ) )));
		Node start(((int) (robot_loc_.x()/GRID_RES)  ),((int) (robot_loc_.y()/GRID_RES)) ,goal);

		planned_path.clear();
		Astar(map_,start,goal,planned_path);
		RunAStar();
	}

  	
}


void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
	// not being called
	// cout<<"update location called"<<endl;	
	if(!robot_loc_init_) {
		robot_loc_init_ = true;
	}

	prev_robot_loc_ = robot_loc_;
	robot_loc_ = loc;

	Vector2f diff = robot_loc_ - prev_robot_loc_;
	if(goal_set && sqrt(diff.x()*diff.x() + diff.y()*diff.y())>3 )
	{
		// Node goal(((int) ( nav_goal_loc_global.x()/GRID_RES )  ),((int) ( ( nav_goal_loc_global.y()/GRID_RES ) )));

		
		goal_set = false;
		Node goal(((int) (robot_loc_.x()/GRID_RES)  ),((int) (robot_loc_.y()/GRID_RES)) );
		Node start(((int) (robot_loc_.x()/GRID_RES)  ),((int) (robot_loc_.y()/GRID_RES)) ,goal);

		planned_path.clear();
		Astar(map_,start,goal,planned_path);
		RunAStar();
		visualization::ClearVisualizationMsg(global_viz_msg_);
		viz_pub_.publish(global_viz_msg_);
	}
	robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
	
	// if(!robot_loc_init_) {
	// 	robot_loc_init_ = true;
	// }

	// prev_robot_loc_ = robot_loc_;
	// robot_loc_ = loc;

	// robot_angle_ = angle;
	// robot_vel_ = vel;
	// robot_omega_ = ang_vel;
	// std::cout<<"robot_loc_.x() : "<<robot_loc_.x()<<", robot_loc_.y() : "<<robot_loc_.y()<<"robot_angle_ : "<<robot_angle_<<"\n";
}

void Navigation::ObservePointCloud(const vector<Vector2f> cloud,
                                   double time) {
	// cout<<"changed point cloud\n";
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
	return 0*fpl-goal_dist;
}

float Navigation::CalcGoalDistance(float x_goal, float y_goal, float x_final, float y_final) {
	return sqrt(pow(x_goal-x_final, 2) + pow(y_goal-y_final, 2));
}

// vector<float> Navigation::SelectCurvature(vector<Vector2f> obstacles){
// 	float max_score, max_score_fpl, max_score_curv;
// 	float curvatures[] = {-6,-5,-4,-3,-2.5,-2,-1.5,-0.9,-0.7,-0.5,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.5,0.7,0.9,1.5,2,2.5,3,4,5,6};
// 	max_score = -std::numeric_limits<float>::max();
// 	max_score_fpl = -1;
// 	max_score_curv = 0;

// 	for(unsigned int current=0; current<sizeof(curvatures)/sizeof(float); current++){
// 		float c = curvatures[current];
// 		float min_fpl = std::numeric_limits<float>::max();
// 		float min_fpl_x, min_fpl_y;
// 		float final_x,final_y;
// 		min_fpl_x = -1;
// 		min_fpl_y = -1;
// 		int min_obstacle = -1;
// 		for(unsigned int i=0; i<obstacles.size(); i++){
// 			float fpl = CalcPointFPL(obstacles[i], c,final_x,final_y,false);
// 			if(fpl <= min_fpl) {
// 				min_obstacle = i;
// 				min_fpl = fpl;
// 				min_fpl_x = final_x;
// 				min_fpl_y = final_y;
// 			}
// 		}
// 		////////////////
// 		if (min_obstacle!=-1)
// 		{
// 			CalcPointFPL(obstacles[min_obstacle], c,final_x,final_y,true);
// 		}
// 		float distance_goal = CalcGoalDistance(nav_goal_loc_.x(), 0, min_fpl_x, min_fpl_y);
// 		float cur_score = CalculateScore(min_fpl, 0, distance_goal);
// 		if(cur_score > max_score){
// 			max_score = cur_score;
// 			max_score_fpl = min_fpl;
// 			max_score_curv = c;
// 		}
// 	}
// 	// probably 1D TOC 
// 	vector<float> ret_val;
// 	ret_val.push_back(max_score_fpl);
// 	ret_val.push_back(max_score_curv);
// 	return ret_val;
// }

vector<float> Navigation::SelectCurvature(vector<Vector2f> obstacles){
	float curvatures[] = {-15,-12,-10,-8,-6,-1,0.5,0,-0.5,1,6,8,10,12,15};
	// float curvatures[] = {-6,-5,-4,-3,-2.5,-2,-1.5,-0.9,-0.7,-0.5,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.5,0.7,0.9,1.5,2,2.5,3,4,5,6};
	// float curvatures[] = {0};
  	// const uint32_t bColor = 0x000000;
  	// float best_score = -std::numeric_limits<float>::max();
  	float best_curvature,best_fpl,best_score = -std::numeric_limits<float>::max();

	// visualization::ClearVisualizationMsg(local_viz_msg_);
	for(int i=0;i<((int)(sizeof(curvatures)/sizeof(float)));i++)
	{
		float cur_curvature = curvatures[i];
		// float cur_radius = 1/cur_curvature;	
		float min_fpl = std::numeric_limits<float>::max();
		float min_fpl_x=-1, min_fpl_y=-1;
		float min_obstacle = -1;
		if(cur_curvature!=0)
		{
			// visualization::DrawCross(Vector2f(0, cur_radius),0.02, bColor, local_viz_msg_);
			// visualization::DrawArc(Vector2f(0, cur_radius), abs(cur_radius),-3.14,3.14, bColor, local_viz_msg_);
		}
		for(unsigned int j=0; j<obstacles.size(); j++){
			float final_x,final_y;
			float fpl = CalcPointFPL(obstacles[j], cur_curvature,final_x,final_y,false);
			if(fpl < min_fpl) {
				// cout<<"in";
				min_obstacle = j;
				min_fpl = fpl;
				min_fpl_x = final_x;
				min_fpl_y = final_y;
			}
		}
		if(cur_curvature<0.001)
		{
			float final_x_check,final_y_check;
			CalcPointFPL(obstacles[min_obstacle], cur_curvature,final_x_check,final_y_check,true);
		}
		// cout<<"min_fpl_x : "<<min_fpl_x<<", min_fpl_y : "<<min_fpl_y<<endl;
		// cout<<"cur_curvature : "<<cur_curvature<<", fpl : "<<min_fpl<<", dist_goal : "<<CalcGoalDistance(nav_goal_loc_.x(),nav_goal_loc_.y(),min_fpl_x,min_fpl_y)<<endl;
		// visualization::DrawCross(Vector2f(min_fpl_x, min_fpl_y),0.1, 0x00FF00, local_viz_msg_);
		// cout<<"score : "<<CalculateScore(min_fpl,0,CalcGoalDistance(nav_goal_loc_.x(),nav_goal_loc_.y(),min_fpl_x,min_fpl_y))<<endl;
		float cur_score = CalculateScore(min_fpl,0,CalcGoalDistance(nav_goal_loc_.x(),nav_goal_loc_.y(),min_fpl_x,min_fpl_y));
		if(cur_score>best_score)
		{
			best_score = cur_score;
			best_curvature = cur_curvature;
			best_fpl = min_fpl;
		}
	}
	// viz_pub_.publish(local_viz_msg_);
	// cout<<"chosen : "<<best_curvature<<endl;
	vector<float> retval;
	retval.push_back(best_fpl);
	retval.push_back(best_curvature);
	return retval;
}

void Navigation::PlotCar(Vector2f start_point)
{

	Vector2f p0((wheel_base + car_length)/2 + clearance_length + start_point.x(),car_width/2 + clearance_width + start_point.y());
	Vector2f p1((wheel_base + car_length)/2 + clearance_length + start_point.x(),-(car_width/2 + clearance_width + start_point.y()));
	Vector2f p2(-((car_length - wheel_base)/2 + clearance_length)+ start_point.x(),car_width/2 + clearance_width + start_point.y());
	Vector2f p3(-((car_length - wheel_base)/2 + clearance_length)+ start_point.x(),-(car_width/2 + clearance_width) + start_point.y());
	visualization::DrawLine(p0,p1, 0x000000, local_viz_msg_);
	visualization::DrawLine(p1,p3, 0x000000, local_viz_msg_);
	visualization::DrawLine(p3,p2, 0x000000, local_viz_msg_);
	visualization::DrawLine(p2,p0, 0x000000, local_viz_msg_);	
	Vector2f p0_in((wheel_base + car_length)/2 + 0 + start_point.x(),car_width/2 + 0 + start_point.y());
	Vector2f p1_in((wheel_base + car_length)/2 + 0 + start_point.x(),-(car_width/2 + 0) + start_point.y());
	Vector2f p2_in(-((car_length - wheel_base)/2 + 0) + start_point.x(),car_width/2 + 0 + start_point.y());
	Vector2f p3_in(-((car_length - wheel_base)/2 + 0) + start_point.x(),-(car_width/2 + 0) + start_point.y());
	visualization::DrawLine(p0_in,p1_in, 0x000000, local_viz_msg_);
	visualization::DrawLine(p1_in,p3_in, 0x000000, local_viz_msg_);
	visualization::DrawLine(p3_in,p2_in, 0x000000, local_viz_msg_);
	visualization::DrawLine(p2_in,p0_in, 0x000000, local_viz_msg_);
}

void Navigation::RunAStar() {
	const uint32_t gColor = 0x00FF00;
  	const uint32_t rColor = 0xFF0000;
	visualization::ClearVisualizationMsg(global_viz_msg_);


	vector<pair<int,int>> answer;
	if( ((int) (planned_path.size())) > AStar_recompute )
	{
		Node goal(planned_path[((int) planned_path.size())-AStar_recompute].first,planned_path[((int) planned_path.size())-AStar_recompute].second);
		Node start(((int) (robot_loc_.x()/GRID_RES)  ),((int) (robot_loc_.y()/GRID_RES)) ,goal);
		Astar(map_,start,goal,answer);
		planned_path.resize(((int) planned_path.size())-AStar_recompute);
		for(auto const& x : answer)
		{
			planned_path.push_back(x);
		}
		answer.clear();
		answer = planned_path;
	}
	else
	{	
		Node goal(((int) ( nav_goal_loc_global.x()/GRID_RES )  ),((int) ( ( nav_goal_loc_global.y()/GRID_RES ) )));
		Node start(((int) (robot_loc_.x()/GRID_RES)  ),((int) (robot_loc_.y()/GRID_RES)) ,goal);
		Astar(map_,start,goal,answer);
		planned_path.clear();
		planned_path = answer;
	}







	for(int i =0;i<((int)answer.size()-1);i++)
	{
		//cout<<answer[i].first<<","<<answer[i].second<<endl;
		Vector2f first_point(((answer[i]).first + 0.5) * GRID_RES, (answer[i].second + 0.5) * GRID_RES);
		Vector2f second_point(((answer[i+1]).first + 0.5) * GRID_RES, (answer[i+1].second + 0.5) * GRID_RES);
		if(i%2==0) {
			visualization::DrawLine(first_point, second_point, gColor, global_viz_msg_);
		} else {
			visualization::DrawLine(first_point, second_point, rColor, global_viz_msg_);
		}
		
	}
	int answer_size = (int) answer.size();
	int count_ahead = 4;
	if(answer_size > count_ahead) {
		temp_goal = Vector2f((answer[answer_size-count_ahead-1].first + 0.5) * GRID_RES, (answer[answer_size-count_ahead-1].second + 0.5) * GRID_RES);
		// std::cout<<"temp x: "<<temp.x()<<" y: "<<
		// nav_goal_loc_ = Eigen::Rotation2Df(-robot_angle_) * temp;
	} else {
		temp_goal = (nav_goal_loc_global);
		// nav_goal_loc_ = Eigen::Rotation2Df(-robot_angle_) * (nav_goal_loc_global-robot_loc_);
	}
	viz_pub_.publish(global_viz_msg_);
}

void Navigation::Run() {


	if(!robot_loc_init_ or !goal_set) {
		return;
	}
	if(!inside_run)
	{
		prev_robot_loc_ = robot_loc_;
		RunAStar();
		nav_goal_loc_ = Eigen::Rotation2Df(-robot_angle_) * (temp_goal-robot_loc_);
		inside_run = true;
	}
	else{
		Vector2f diff = (robot_loc_-prev_robot_loc_);
		float distance = sqrt(diff.x()*diff.x() + diff.y()*diff.y());
		if( (distance > LIMIT_ASTAR)||(no_astar_called>30) )
		{
			// std::cout<<"inside second astar"<<endl;
			RunAStar();
			no_astar_called = 0;
			prev_robot_loc_ = robot_loc_;
		} 
		no_astar_called++;
		nav_goal_loc_ = Eigen::Rotation2Df(-robot_angle_) * (temp_goal-robot_loc_);
	}
	visualization::ClearVisualizationMsg(local_viz_msg_);
	// nav_goal_loc_global = Vector2f(-23,9);

		

	// Vector2f relative_vec = nav_goal_loc_global - robot_loc_;
	// nav_goal_loc_ = Eigen::Rotation2Df(-robot_angle_)*relative_vec;

	PlotCar(Vector2f(0,0));
	// PlotCar(nav_goal_loc_);
	// viz_pub_.publish(local_viz_msg_);
	vector<float> ret_vals;
	if(point_cloud_.size()==0)
	{
		ret_vals.push_back(0);
		ret_vals.push_back(0);
	}
	else
	{
		// cout<<"here"<<endl;
		ret_vals  = SelectCurvature(point_cloud_);
		visualization::DrawCross(nav_goal_loc_,0.2, 0xFF0000, local_viz_msg_);
		viz_pub_.publish(local_viz_msg_);
		// cout<<" nav_goal_loc_ : "<<nav_goal_loc_<<endl;
		//exit(0);
	}


	// visualization::ClearVisualizationMsg(local_viz_msg_);	
	// visualization::DrawCross(nav_goal_loc_,0.2, 0xFF0000, local_viz_msg_);
	// viz_pub_.publish(local_viz_msg_);
	// // visualization::DrawCross(Vector2f(2.92,0),2, 0xFF0000, local_viz_msg_);
	// // visualization::DrawCross(Vector2f(1.7957, 4.23938),2, 0xFF0000, local_viz_msg_);
	// // visualization::DrawCross(Vector2f(3.27161, -1.21892),2, 0xFF0000, local_viz_msg_);
	// // viz_pub_.publish(local_viz_msg_);
	// // std::cout<<"FPL: "<<ret_vals[0]<<", Curvature: "<<ret_vals[1]<<"\n";
	// // float u = sqrt(pow(robot_vel_.x(), 2) + pow(robot_vel_.y(), 2));
	// // std::cout<<"robot_vel_.x() : "<<robot_vel_.x()<<", robot_vel_.y() : "<<robot_vel_.y()<<"\n";
	// cout<<"fpl : "<<ret_vals[0]<<endl;
	float v = OneDTOC(prev_velocity, 1, 4, -4, ret_vals[0]);
	// cout<<"v : "<<v<<endl;
	// std::cout<<"u : "<<prev_velocity<<", v : "<<v<<"\n";
	if(sqrt(pow(nav_goal_loc_.x(), 2)+pow(nav_goal_loc_.y(), 2)) < 0.3) {
		v = 0;
	}
	drive_msg_.velocity = v;
	prev_velocity = v;

	//drive_msg_.velocity = 0;
	drive_msg_.curvature = ret_vals[1];
	// drive_msg_.velocity = 0;
	// drive_msg_.curvature = 0;
	drive_pub_.publish(drive_msg_);
}

float Navigation::OneDTOC(float u,float u_max,float a_max,float a_min,float s)
{
	float timestep = 1.0/20;
	float s_min;
	float error_margin = 0;
	s_min = u*u/(2*-a_min);
	// cout<<"s : "<<s<<endl;
	if(s<0.2)
	{	
		// cout<<"ret 0"<<endl;
		return 0;
	}
	else if (s<=s_min-error_margin)
	{
		// std::cout<<"in\n";
		float out = u - (u*u/(2*s))*timestep;
		if(out<0)
		{
			// std::cout<<"in2";
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

// i is along x axis and j is along y axis
void Navigation::CalculateGrid(Vector2f xy, int& i, int& j) {
	float x, y;
	x = xy.x();
	y = xy.y();

	i = floor(x/GRID_RES);
	j = floor(y/GRID_RES);
}

}  // namespace navigation


































































unordered_map<string, bool> state_intersects;
unordered_map<string,float> distance_dict;

bool operator == (Node const &  first,Node const & second)
{
	return (first.i==second.i) && (first.j==second.j);
}
Node::Node(int i, int j) {
	this->i = i;
	this->j = j; 
	this->h = 0;
	this->g = 0;
	//need to implement distance from end point as h
}
Node::Node(int i, int j,const Node& goal) {
	this->i = i;
	this->j = j; 
	this->g = 0;

	float diff_i = abs(goal.i - this->i);
	float diff_j = abs(goal.j - this->j);
	// float min_diff = diff_i;
	// if(diff_j<min_diff)
	// {
	// 	min_diff = diff_j;
	// }
	// float max_diff = diff_j + diff_i - min_diff;
	// this->h = min_diff*1.414 + (max_diff-min_diff);
	this->h = sqrt(diff_j*diff_j + diff_i*diff_i);
	//need to implement distance from end point as h
}
void Node::NodeIntersectsMap(vector<line2f> map_lines) {

	for(unsigned int i = 0; i < map_lines.size(); i++) {
		line2f map_line = map_lines[i];
		float left_x, right_x, top_y, bottom_y;
		left_x = this->i  * 1 * GRID_RES -0.6*GRID_RES;
		right_x = (this->i + 1) * 1 * GRID_RES+0.6*GRID_RES;
		bottom_y = this->j  * 1 * GRID_RES-0.6*GRID_RES;
		top_y = (this->j + 1) * 1  * GRID_RES+0.6*GRID_RES;
		const line2f line1(left_x, bottom_y, left_x, top_y);
		const line2f line2(left_x, top_y, right_x, top_y);
		const line2f line3(right_x, top_y, right_x, bottom_y);
		const line2f line4(right_x, bottom_y, left_x, bottom_y);

		float p0x, p0y, p1x, p1y;
		p0x = map_line.p0.x();
		p0y = map_line.p0.y();
		p1x = map_line.p1.x();
		p1y = map_line.p1.y();

		if(map_line.Intersects(line1) || map_line.Intersects(line2) ||
			map_line.Intersects(line3) || map_line.Intersects(line4) ||
			((p0x > left_x && p0x < right_x && p0y > bottom_y && p0y < top_y) &&
			(p1x > left_x && p1x < right_x && p1y > bottom_y && p1y < top_y))) {
			this->intersects = true;
			return;
		}
	}

	this->intersects = false;

}
float Node::GetValue(vector<line2f> map_lines) {

	if(distance_dict.find(this->GetState())!=distance_dict.end())
	{
		return distance_dict[this->GetState()];
	}
	
	float min_dist = std::numeric_limits<float>::max();
	for(unsigned int i=0; i<map_lines.size(); i++) {
	 	line2f line = map_lines[i];
	 	Vector2f point( (this->i+0.5) * GRID_RES, (this->j+0.5) * GRID_RES);
	 	float dist_points = pow(line.p0.x()-line.p1.x(), 2) + pow(line.p0.y()-line.p1.y(), 2);

		 
		
	 	if(dist_points == 0) {
	 		float answer = 1/(pow(line.p0.x()-point.x(), 2) + pow(line.p0.y()-point.y(), 2));
	 		distance_dict[this->GetState()] = answer;
	 		return answer;
	 	}
	 	float p0_point_x = point.x() - line.p0.x();
	 	float p0_point_y = point.y() - line.p0.y();

	 	float p0_p1_x = line.p1.x() - line.p0.x();
	 	float p0_p1_y = line.p1.y() - line.p0.y();
	 	float dot = p0_p1_x * p0_point_x + p0_p1_y * p0_point_y;
	 	const float t = max((float)0.0, min((float)1.0, dot/dist_points));

	 	float x_proj = line.p0.x() + t * p0_p1_x;
	 	float y_proj = line.p0.y() + t * p0_p1_y;

	 	float dist = (pow(x_proj - point.x(), 2) + pow(y_proj - point.y(), 2));

	 	if(dist<min_dist) {
	 		min_dist = dist;
	 	}
		
	 }
	 distance_dict[this->GetState()] = 1/min_dist;
	 return 1/min_dist;
}
string Node::GetState() {
	 	string istate = to_string(this->i);
	 	string jstate = to_string(this->j);
	 	string comma = ",";
	 	string state = istate + comma + jstate; 
	 	return state;
}
float Node::GetPriority()
{
	return -this->g - this->h;
}
#define WEIGHT_COST 0.1
// #include<iostream>
// #include<pair>
// #include "node.h"
#include "simple_queue.h"

#ifndef ASTAR
#define ASTAR
//returns 1 if path found
int Astar(vector_map::VectorMap map_,Node start_point,Node& end_point,vector<pair<int,int>>& answer)
{
	start_point.g = WEIGHT_COST * start_point.GetValue(map_.lines);
	SimpleQueue<string,float> open_queue;
	SimpleQueue<string,float> closed_queue;
	unordered_map<string, Node> node_map;
	node_map[start_point.GetState()] = start_point;
	open_queue.Push(start_point.GetState(),start_point.GetPriority());
	
	while(!open_queue.Empty())
	{
		// cout<<"inA\n";
		string q_str = open_queue.Pop();
		Node q = node_map[q_str];
		// cout<<"entered : "<<q.i<<","<<q.j<<endl;
		
		Node succesors[8];
		int succesors_x[] = {-1,-1,0,1,1,1,0,-1};//clockwise starting from left
		int succesors_y[] = {0,1,1,1,0,-1,-1,-1};
		float distances[] = {1,1.414,1,1.414,1,1.414,1,1.414};
		for(int neigh=0;neigh<8;neigh++)
		{

			
			Node cur_succesor = Node(q.i + succesors_x[neigh],q.j+succesors_y[neigh],end_point);
			if(state_intersects.find(cur_succesor.GetState()) == state_intersects.end()) {
				cur_succesor.NodeIntersectsMap(map_.lines);
				state_intersects[cur_succesor.GetState()] = cur_succesor.intersects;

			} else {
				cur_succesor.intersects = state_intersects[cur_succesor.GetState()];
			}
			succesors[neigh] = cur_succesor;
		}
		int i = 0;
		//potent
		for(auto& succesor : succesors)
		{
			if(succesor.intersects)
			{
				i++;
				continue;
			}
			if(succesor==end_point)
			{
				end_point.parent = q.GetState();
				Node cur_node = end_point;

				while(!(cur_node.GetState() == start_point.GetState()))
				{

					pair<int,int> node_pair(cur_node.i,cur_node.j);
					//cout<<"inside loop, i : "<<cur_node.i<<", j : "<<cur_node.j<<endl;
					answer.push_back(node_pair);
					cur_node = node_map[cur_node.parent];
				}
				pair<int,int> node_pair(cur_node.i,cur_node.j);
				answer.push_back(node_pair);
				return 1;
			}
			succesor.g = q.g + WEIGHT_COST*succesor.GetValue(map_.lines) + distances[i];//to implement cost as value?
			// succesor.h = abs(end_point.i-succesor.i) + abs(end_point.j-succesor.j);//what cost is best?
			if(open_queue.Exists(succesor.GetState()))
			{
				float cur_priority = open_queue.GetPriority(succesor.GetState());
				if(succesor.GetPriority() < cur_priority)
				{
					i++;
					continue;
					// open_queue.push(succesor,succesor.g+succesor.h);
				}
				else
				{
					succesor.parent = q.GetState();
					open_queue.Push(succesor.GetState(),succesor.GetPriority());
					node_map.erase(succesor.GetState());
					node_map[succesor.GetState()] = succesor;
					i++;
					continue;
				}
			}
			else if(closed_queue.Exists(succesor.GetState()))
			{
				float cur_priority = closed_queue.GetPriority(succesor.GetState());
				if(succesor.GetPriority() < cur_priority)
				{
					i++;
					continue;
					// open_queue.push(succesor,succesor.g+succesor.h);
				}
				// else
				// {
				// 	succesor.parent = q.GetState();
				// 	closed_queue.Push(succesor.GetState(),succesor.GetPriority());
				// 	node_map.erase(succesor.GetState());
				// 	node_map[succesor.GetState()] = succesor;
				// 	i++;
				// 	continue;
				// }
			}
			else
			{	
				succesor.parent = q.GetState();
				//cout<<"parent, i : "<<succesor.i<<",j : "<<succesor.j<<", parent is i : "<<succesor.parent->i<<", j : "<<succesor.parent->j<<endl;
				open_queue.Push(succesor.GetState(),succesor.GetPriority());
				node_map[succesor.GetState()] = succesor;
				// if(node_map.find(succesor.GetState()) != node_map.end())
				// {
				// 	// node_map.erase(succesor.GetState());
				// 	cout<<"prev_priority : "<<node_map.at(succesor.GetState()).GetPriority()<<endl;
				// 	node_map[succesor.GetState()] = succesor;
				// 	cout<<"after_priority : "<<node_map.at(succesor.GetState()).GetPriority()<<endl;
				// }
				// else
				// {
				// 	node_map[succesor.GetState()] = succesor;
				// 	// cout<<"after_priority : "<<node_map.at(succesor.GetState()).GetPriority()<<endl;
				// }
				
			}

			i++;
		}
		closed_queue.Push(q.GetState(),q.GetPriority());

	}
	return 0;
}	



#endif  // ASTAR
	