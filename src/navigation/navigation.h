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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"
// #include "node.cc"
#include "Astar.h"


#ifndef NAVIGATION_H
#define NAVIGATION_H

using std::vector;
using Eigen::Vector2f;

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f> cloud,
                         double time);
	float CalcPointFPL(const Vector2f obstacles, float radius, float &final_x, float &final_y,bool debug);
	float CalculateScore(float fpl, float clearance, float goal_dist);
  float CalcGoalDistance(float x_goal, float y_goal, float x_final, float y_final);
  vector<float> SelectCurvature(vector<Vector2f> obstacles);
	vector<float> SelectCurvature2(vector<Vector2f> obstacles);
  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);
  float OneDTOC(float u,float u_max,float a_max,float a_min,float s);
	void CalculateGrid(Vector2f xy, int& i, int& j);
  void PlotCar(Vector2f start_point);
  void RunAStar();

 private:

  // Current robot location.
  Eigen::Vector2f robot_loc_;
  Eigen::Vector2f prev_robot_loc_;
  Eigen::Vector2f temp_goal;
  bool robot_loc_init_;
  bool inside_run;
  bool goal_set;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;

  // Odometry-reported robot angle.
  float odom_angle_;

  // Map of the environment.
  vector_map::VectorMap map_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  Eigen::Vector2f nav_goal_loc_global;
  // Navigation goal angle.
  float nav_goal_angle_;
	// Pointcloud of obstacles
	vector<Vector2f> point_cloud_;

  float prev_velocity = 0;
  float cur_veclocity = 0;

  int no_astar_called = 0;
  vector<pair<int,int>> planned_path;
  
};

}  // namespace navigation

#endif  // NAVIGATION_H
