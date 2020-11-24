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
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include <iostream>
#include <fstream>
#include "slam.h"
#include <string>

#include "vector_map/vector_map.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

#define k1_x 0.2
#define k2_x 0.2
#define k1_y 0.05
#define k2_y 0.05
#define k3_theta 0.05
#define k4_theta 0.05

#define THETA_RES 0.05
#define NUM_THETA_VALS 5

#define X_RES 0.05
#define NUM_X_VALS 5

#define Y_RES 0.05
#define NUM_Y_VALS 5
// #define RASTER_SIZE 5
// #define RASTER_RES 0.05



#define ANGLE_THRESHOLD M_PI/6
#define TRANS_THRESHOLD 0.5


namespace slam {

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false), 
    execute_csm_(false),
    dx_(0),
    dy_(0),
    dtheta_(0),
    change_x(0),
    change_y(0),
    change_theta(0),
    curr_x(0),
    curr_y(0),
    curr_theta(0) {
      raster_table_.createRasterTable(RASTER_SIZE,RASTER_RES,SIGMA_RASTER);
    }

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = Vector2f(curr_x, curr_y);
  *angle = curr_theta;
}

void SLAM::CreatePointCloud(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max,
                        vector<Vector2f>& pointcloud) {

  int ranges_length = ranges.size();
  // float theta_inc = (angle_max - angle_min)/ranges_length;

  for(int i = 0; i < ranges_length; i++) {
    float theta_i, range_i, x_i, y_i;
    range_i = ranges[i];
    if( (range_i<=range_max-0.001) && (range_i>=range_min+0.001) )
    {
      theta_i = (angle_min*(ranges_length-1) + i * (angle_max-angle_min))/(ranges_length-1);
      x_i = range_i * cos(theta_i) + 0.2;
      y_i = range_i * sin(theta_i);
      pointcloud.push_back(Vector2f(x_i, y_i)); 
    }
  }

}

void SLAM::RotatePointCloud(const vector<Vector2f>& input,vector<Vector2f>& output, float angle)
{
  for(const auto& cur_point : input)
  {
    Vector2f temp = Eigen::Rotation2Df(angle)*cur_point;
    output.push_back(temp);
  }
}

void SLAM::TranslatePointCloud(const vector<Vector2f>& input,vector<Vector2f>& output, Vector2f translation)
{
  for(const auto& cur_point : input)
  {
    Vector2f temp = cur_point + translation;
    output.push_back(temp);
  }
}

void SLAM::FilterPointCloud(const vector<Vector2f>& input,vector<Vector2f>& output)
{
  for(const auto& cur_point : input)
  {
    float distance = sqrt(cur_point.x()*cur_point.x() + cur_point.y()*cur_point.y());
    if(distance <= (RASTER_SIZE-1) )
    {
      output.push_back(cur_point);
    }
  }
}


void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  // std::cout<<"1\n";
  if(execute_csm_) {
    execute_csm_ = false;

    Vector2f best_xy(curr_x+dx_,curr_y+dy_);
    float best_angle = curr_theta+dtheta_;
    float best_score = -(std::numeric_limits<float>::max()-1);
    int best_x_ind=-100,best_y_ind=-100,best_theta_ind=-100;

    vector<Vector2f> pointcloud_carframe;
    CreatePointCloud(ranges, range_min, range_max, angle_min, angle_max, pointcloud_carframe);

    for(int ind_x=-NUM_X_VALS;ind_x<=NUM_X_VALS;ind_x++)
    {
      for(int ind_y=-NUM_Y_VALS;ind_y<=NUM_Y_VALS;ind_y++)
      {
        for(int ind_theta=-NUM_THETA_VALS;ind_theta<=NUM_THETA_VALS;ind_theta++)
        {
          float err_theta;
          float change_theta;
          Vector2f err_carframe;
          Vector2f err_prevcarframe;
          Vector2f err;
          Vector2f dxy;
          Vector2f dxy_prevcarframe;
          Vector2f changexy_prevcarframe;
          Vector2f changexy;

          err_carframe.x() = ind_x*X_RES;
          err_carframe.y() = ind_y*Y_RES;
          err_theta = ind_theta*THETA_RES;
          err = Eigen::Rotation2Df(curr_theta+dtheta_)*err_carframe;
          err_prevcarframe = Eigen::Rotation2Df(dtheta_)*err_carframe;
          dxy.x() = dx_;
          dxy.y() = dy_;
          dxy_prevcarframe = Eigen::Rotation2Df(-curr_theta)*dxy;
          changexy_prevcarframe = dxy_prevcarframe + err_prevcarframe;
          changexy = Eigen::Rotation2Df(curr_theta)*changexy_prevcarframe;
          change_theta = dtheta_ + err_theta;
          // cout<<"dtheta_ : "<<dtheta_<<endl;
          // cout<<"err_theta : "<<err_theta<<endl;
          // cout<<"change_theta : "<<change_theta<<endl;

          vector<Vector2f> pointcloud_prevcarframe;
          vector<Vector2f> temp,filtered;
          RotatePointCloud(pointcloud_carframe,temp,change_theta);
          TranslatePointCloud(temp,pointcloud_prevcarframe,changexy_prevcarframe);
          FilterPointCloud(pointcloud_prevcarframe,filtered);
          float log_score_avg = raster_table_.lookup_pointcloud(filtered);


          // // cout<<"curr_theta+dtheta_ : "<<curr_theta+dtheta_;
          // cout<<"ind_x : "<<ind_x<<",";
          // // cout<<"err_x_car : "<<err_carframe.x()<<endl;
          // // cout<<"err_x : "<<err.x()<<endl;
          // cout<<"ind_y : "<<ind_y<<",";
          // // cout<<"err_y_car : "<<err_carframe.y()<<endl;
          // // cout<<"err_y : "<<err.y()<<endl;
          // cout<<"ind_theta : "<<ind_theta<<endl;
          // // cout<<"err_theta : "<<err_theta<<endl;
          // cout<<"avg log score : "<<log_score_avg<<endl<<endl;
          if(log_score_avg>best_score)
          {
            best_score = log_score_avg;
            best_xy.x() = curr_x + changexy.x();
            best_xy.y() = curr_y + changexy.y();
            best_angle = curr_theta + change_theta;
            best_x_ind = ind_x;
            best_y_ind = ind_y;
            best_theta_ind = ind_theta;
          }
        }
      }
    }
    // pointcloud_test.push_back(Vector2f(1,0));
    // pointcloud_test.push_back(Vector2f(0,1));
    // for(float x_val=-6,x_)

    //uncomment if not test
    raster_table_.populate_pointcloud(pointcloud_carframe);
    // raster_table_.print_table();
    // exit(0);


    //test
    // vector<Vector2f> pointcloud_test;
    // FilterPointCloud(pointcloud_carframe,pointcloud_test);
    // raster_table_.populate_pointcloud(pointcloud_test);
    // vector<Vector2f> translated;
    // vector<Vector2f> rotated;
    // Vector2f translation(0.2,0.2);
    // TranslatePointCloud(pointcloud_test,translated,translation);
    // RotatePointCloud(translated,rotated,0.2);
    // float max_score = -(std::numeric_limits<float>::max()-1);
    // int max_x=-10,max_y=-10,max_theta=-10;

    // // raster_table_.print_table();
    // // printf("%3d",0);
    // // for(int ind_y = -5;ind_y<=5;ind_y++)
    // // {
    // //   printf(" %7d",ind_y);
    // // }
    // // printf("\n");
    // for(int ind_x = -10;ind_x<=10;ind_x++)
    // {
    //   // printf("%3d",ind_x);
    //   for(int ind_y = -10;ind_y<=10;ind_y++)
    //   {
    //     for(int ind_theta = -10;ind_theta<=10;ind_theta++)
    //     {
    //       vector<Vector2f> new_pointcloud;
    //       vector<Vector2f> new_pointcloud2;
    //       float add_x = ind_x*0.1;
    //       float add_y = ind_y*0.1;
    //       float add_theta = ind_theta*0.1;
    //       // cout<<"add_x : "<<add_x<<endl;
    //       // cout<<"add_y : "<<add_y<<endl;
    //       // for(auto x : translated)
    //       // {
    //       //   Vector2f temp = x + Vector2f(add_x,add_y);
    //       //   new_pointcloud.push_back(temp);
    //       // }
    //       RotatePointCloud(rotated,new_pointcloud,add_theta);
    //       TranslatePointCloud(new_pointcloud,new_pointcloud2,Vector2f(add_x,add_y));
    //       float score = raster_table_.lookup_pointcloud(new_pointcloud2);
    //       if(score>max_score)
    //       {
    //         max_score = score;
    //         max_x = ind_x;
    //         max_y = ind_y;
    //         max_theta = ind_theta;
    //       }
    //       // cout<<"ind_x : "<<ind_x<<", ind_y : "<<ind_y<<", ind_theta : "<<ind_theta<<", score : "<<score<<"\n";
    //       // cout<<"avg log score : "<<score<<endl<<endl;
    //       // printf(" %6.4f",score);
    //     }
    //   }
    //   // printf("\n");
    // }
    // cout<<"max_x : "<<max_x<<", max_y : "<<max_y<<", max_theta : "<<max_theta<<endl;
    // exit(0);
    

    curr_x = best_xy.x();
    curr_y = best_xy.y();
    curr_theta = best_angle;
    for(auto& x : pointcloud_carframe)
    {
      Vector2f temp = Eigen::Rotation2Df(curr_theta)*x;
      temp.x() += curr_x;
      temp.y() += curr_y;
      map_.push_back(temp);
    }    
    std::cout<<"End of laser\n";
    cout<<"best_x_ind : "<<best_x_ind<<", best_y_ind : "<<best_y_ind<<", best_theta_ind"<<best_theta_ind<<endl;
    std::cout<<"curr_x : "<<curr_x<<endl;
    std::cout<<"curr_y : "<<curr_y<<endl;
    std::cout<<"curr_theta : "<<curr_theta<<endl;
    // std::cout<<"add_x : "<<add_x<<endl;
    // std::cout<<"add_y : "<<add_y<<endl;
    // std::cout<<"add_theta : "<<add_theta<<endl<<endl<<endl;
  }
}



void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
  float abs_dtheta;
  change_x = odom_loc.x() - prev_odom_loc_.x();
  change_y = odom_loc.y() - prev_odom_loc_.y();
  change_theta = odom_angle - prev_odom_angle_;
  abs_dtheta = std::min(abs(change_theta), float(2*M_PI)-abs(change_theta));



  if(abs(abs_dtheta)>ANGLE_THRESHOLD || sqrt(change_x * change_x + change_y * change_y) > TRANS_THRESHOLD) {
    execute_csm_ = true;
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    dx_ = change_x;
    dy_ = change_y;
    dtheta_ = change_theta;
    std::cout<<"inside observe odom thresh\n";
    std::cout<<"dx_ : "<<dx_<<endl;
    std::cout<<"dy_ : "<<dy_<<endl;
    std::cout<<"dtheta_ : "<<dtheta_<<endl;
  }

}

vector<Vector2f> SLAM::GetMap() {
  
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map_;
}

}  // namespace slam
