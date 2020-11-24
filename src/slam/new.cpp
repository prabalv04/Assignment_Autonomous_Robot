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

#define THETA_RES 0.09
#define NUM_THETA_VALS 10

#define X_RES 0.05
#define NUM_X_VALS 10

#define Y_RES 0.05
#define NUM_Y_VALS 10
// #define RASTER_SIZE 5
// #define RASTER_RES 0.05

#define SIGMA_RASTER 0.5

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
      for(int i=0;i<2*RASTER_SIZE/RASTER_RES;i++)
      {
        std::vector<float> temp;
        for(int j=0;j<2*RASTER_SIZE/RASTER_RES;j++)
        {
          temp.push_back(1);
        }
        raster_table_.push_back(temp);
      }
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
  float theta_inc = (angle_max - angle_min)/ranges_length;

  for(int i = 0; i < ranges_length; i++) {
    float theta_i, range_i, x_i, y_i;
    range_i = ranges[i];
    if( (range_i<=range_max-0.001) && (range_i>=range_min+0.001) )
    {
      theta_i = angle_min + i * theta_inc;
      x_i = range_i * cos(theta_i) + 0.2;
      y_i = range_i * sin(theta_i);
      pointcloud.push_back(Vector2f(x_i, y_i)); 
    }
  }

}

float SLAM::RasterLookup(float x, float y) {
  // raster_table_
  int x_index, y_index;
  x_index = ((int)RASTER_SIZE/RASTER_RES) + floor(x/RASTER_RES);
  y_index = ((int)RASTER_SIZE/RASTER_RES) + floor(y/RASTER_RES);

  return raster_table_[x_index][y_index];


}
// float SLAM::GetRaster(vector<Vector2f>& output_) {
//   // raster_table_

//   // int x_index, y_index;
//   // x_index = ((int)RASTER_SIZE/RASTER_RES) + floor(x/RASTER_RES);
//   // y_index = ((int)RASTER_SIZE/RASTER_RES) + floor(y/RASTER_RES);

//   // return raster_table_[x_index][y_index];


// }


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

    vector<Vector2f> pointcloud_poss;
    vector<Vector2f> pointcloud_map;

    curr_x = curr_x + dx_;
    curr_y = curr_y + dy_;
    curr_theta = curr_theta + dtheta_;

    CreatePointCloud(ranges, range_min, range_max, angle_min, angle_max, pointcloud_poss);
    for(auto& x : pointcloud_poss)
    {
      Vector2f temp = Eigen::Rotation2Df(curr_theta)*x;
      temp.x() += curr_x;
      temp.y() += curr_y;
      map_.push_back(temp);
    }
    



    std::cout<<"End of laser\n";
    std::cout<<"curr_x : "<<curr_x<<endl;
    std::cout<<"curr_y : "<<curr_y<<endl;
    std::cout<<"curr_theta : "<<curr_theta<<endl;
    // std::cout<<"add_x : "<<add_x<<endl;
    // std::cout<<"add_y : "<<add_y<<endl;
    // std::cout<<"add_theta : "<<add_theta<<endl<<endl<<endl;
  }
}

void SLAM::GetRasterTable(const Vector2f& odom_loc, const float odom_angle, vector<Vector2f>& scan_ptr)
{
  //scan_ptr has the point cloud, the raster table would be populated in the raster_table argument
  //the raster table will be aligned along odom_angle and cenetered at odom_loc

  //first we populate the table with 0 probs
  vector< vector<float> >& raster_table = raster_table_;
  for (int i = 0; i < ((int)raster_table.size()); i++)
  {
    // vector<float> temp;
    for (int j = 0; j < ((int)raster_table[0].size()); j++)
    {
        // temp.push_back(0);
      raster_table[i][j] = 0;
    }
    // raster_table.push_back(temp);
  }
  // cout<<"0 prob initialization done"<<endl;

  //iterate through the points in scan_ptr and update the raster_table
  vector<Vector2f>& scan = scan_ptr;
  // for(int j = 0; j< scan.size(); ++j)
  //   {

  //    //the obstacle position
  //    float obst_x = scan[j].x();
  //    float obst_y = scan[j].y();

  //    //get the indices of the obstacle in the raster table
  //     int raster_table_index_obstacle_x = int((obst_x - odom_loc.x())/granualarity) + int(raster_table.size()/2);
  //     int raster_table_index_obstacle_y = int((obst_y - odom_loc.y())/granualarity) + int(raster_table.size()/2);

  //    if(raster_table_index_obstacle_x >= int(raster_table.size()) || raster_table_index_obstacle_y >= int(raster_table.size()) ||
  //    raster_table_index_obstacle_x < 0 || raster_table_index_obstacle_y <0)
  //    {
  //      continue; //since obstacle is out of the grid
  //    }

  //    cout<<"obstacle indices: "<<raster_table_index_obstacle_x<<" "<<raster_table_index_obstacle_y<<endl;
  //    //update prob at obstacle indices
  //   //  float prob = 1/(M_PI * s); //gaussian value at mean
  //   //  raster_table[raster_table_index_obstacle_x][raster_table_index_obstacle_y] += prob; 


  //   }

  float s = 2 * SIGMA_RASTER * SIGMA_RASTER;
  float total_sum = 0.0;
  //iterate through grid and map the grid-points to actual map-coordinates to calculate prob
  for( int i=0; i<int(raster_table.size()); ++i)
  {
    for( int j= 0; j<int(raster_table[i].size());++j)
    {
      float map_x = (float(i)-raster_table.size()/2 + 0.5)*RASTER_RES*cos(odom_angle) + 
      (float(j)-raster_table.size()/2 + 0.5)*RASTER_RES*sin(odom_angle) + odom_loc.x();

      float map_y = (float(i)-raster_table.size()/2 + 0.5)*RASTER_RES*sin(odom_angle) + 
      (float(j)-raster_table.size()/2 + 0.5)*RASTER_RES*cos(odom_angle) + odom_loc.y();

      // cout<<map_x<<" "<<map_y<<endl;

      //compute prob using all obstacles
      for(int k = 0; k< ((int)scan.size()); ++k)
      {

        //the obstacle position
        float obst_x = scan[k].x();
        float obst_y = scan[k].y();

        //prob
        float r = (obst_x - map_x) * (obst_x - map_x) + (obst_y - map_y) * (obst_y - map_y);
        float prob = exp(-r/s)/(M_PI * s);

        raster_table[i][j] += prob; 
        total_sum += prob;
      }

    }
  }
  raster_count += 1;
  std::ofstream myfile;
  myfile.open("vals"+std::to_string(raster_count)+".txt");
  

  for(int i=0; i<((int)raster_table.size()); ++i)
  {
    for(int j= 0; j<((int)raster_table[i].size());++j){
      raster_table[i][j] /= total_sum;
      myfile << raster_table[i][j] << " ";
    }
    myfile << "\n";
      
  }
  myfile.close();
  // exit(0);
    
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
  // Eigen::Rotation2Df x(2*M_PI);
  // Eigen::Vector2f b(1,0);
  // Eigen::Vector2f a = x*b;
  // std::cout<<"a.x() : "<<a.x()<<", a.y() : "<<a.y()<<endl;
  // exit(0);


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
