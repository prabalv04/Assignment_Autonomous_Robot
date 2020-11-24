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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "raster_table.cc"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

#define RASTER_SIZE 6
#define RASTER_RES 0.05
#define SIGMA_RASTER 0.2

namespace slam {

class SLAM {
 public:
  // Default Constructor.
  SLAM();
  void CreatePointCloud(const std::vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max,
                        std::vector<Eigen::Vector2f>& pointcloud);
  void RotatePointCloud(const vector<Vector2f>& input,vector<Vector2f>& output, float angle);
  void TranslatePointCloud(const vector<Vector2f>& input,vector<Vector2f>& output, Vector2f translation);
  void FilterPointCloud(const vector<Vector2f>& input,vector<Vector2f>& output);
  float RasterLookup(float x, float y);

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  void GetRasterTable(const Eigen::Vector2f& odom_loc, 
                      const float odom_angle, 
                      std::vector<Eigen::Vector2f>& scan_ptr);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

 private:

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  bool execute_csm_;
  float dx_, dy_, dtheta_;
  float change_x,change_y,change_theta;
  std::vector<std::vector<float>> raster_table;

  float curr_x, curr_y, curr_theta;
  // std::vector<std::vector<float>> raster_table_;
  RasterTable raster_table_;

  std::vector<Eigen::Vector2f> map_;
  int raster_count = 0;
};
}  // namespace slam

#endif   // SRC_SLAM_H_
