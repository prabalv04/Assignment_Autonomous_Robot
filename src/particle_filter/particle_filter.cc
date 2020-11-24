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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <math.h>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"


#include "amrl_msgs/AckermannCurvatureDriveMsg.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

// namespace {
// ros::Publisher drive_pub_;
// ros::Publisher viz_pub_;
// VisualizationMsg local_viz_msg_;
// VisualizationMsg global_viz_msg_;
// AckermannCurvatureDriveMsg drive_msg_;
// // Epsilon value for handling limited numerical precision.
// const float kEpsilon = 1e-5;

// const float car_width = 0.281;
// const float car_length = 0.535;
// const float car_height = 0.15;
// const float wheel_base = 0.324;
// const float clearance_length = 0.2;
// const float clearance_width = 0.1;
// } 

DEFINE_double(num_particles, 50, "Number of particles");
#define k1_x 0.2
#define k2_x 0.2
#define k1_y 0.1
#define k2_y 0.1
#define k3_theta 0.15
#define k4_theta 0.15
#define lidar_dist 0.2
#define sigma 3000.0

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
  // *our_obstacles = our_obstacles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  float point_x, point_y, point_theta;
  point_x = loc.x();
  point_y = loc.y();
  point_theta = angle;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  // for (size_t i = 0; i < scan.size(); ++i) {
  //   scan[i] = Vector2f(0, 0);
  // }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  float laser_x, laser_y;
  laser_x = point_x + lidar_dist * cos(point_theta);
  laser_y = point_y + lidar_dist * sin(point_theta);
  Vector2f laser_vector(laser_x, laser_y);

  for (size_t angle_i = 0; angle_i < scan.size(); ++angle_i) {

    float angle_ls, final_x, final_y, final_distance; 
    angle_ls = point_theta + angle_min + angle_i * (angle_max-angle_min)/num_ranges;
    final_x = laser_x + range_max * cos(angle_ls);
    final_y = laser_y + range_max * sin(angle_ls);
    final_distance = range_max;

    for (size_t map_i = 0; map_i < map_.lines.size(); ++map_i) {
      const line2f map_line = map_.lines[map_i];
      // The line2f class has helper functions that will be useful.
      // You can create a new line segment instance as follows, for :
      // line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
      // Access the end points using `.p0` and `.p1` members:
      const line2f laser_line(laser_x, laser_y, final_x, final_y); 
      // printf("P0: %f, %f P1: %f,%f\n", 
      //        my_line.p0.x(),
      //        my_line.p0.y(),
      //        my_line.p1.x(),
      //        my_line.p1.y());

      // Check for intersections:
      bool intersects = map_line.Intersects(laser_line);
      // You can also simultaneously check for intersection, and return the point
      // of intersection:
      if(intersects) { 
        // std::cout<<"Intersected"<<"\n";
        Vector2f intersection_point;
        intersects = map_line.Intersection(laser_line, &intersection_point);
        float distance_intersection;
        distance_intersection = (laser_vector-intersection_point).norm();
        if(distance_intersection<final_distance) {
          final_distance = distance_intersection;
          final_x = intersection_point.x();
          final_y = intersection_point.y();
          // std::cout<<"Updated"<<"\n";
        }
      }
      // if (intersects) {
      //   printf("Intersects at %f,%f\n", 
      //          intersection_point.x(),
      //          intersection_point.y());
      // } else {
      //   printf("No intersection\n");
      // }
    }
    if(final_distance<range_min) {
      final_x = laser_x + range_min * cos(angle_ls);
      final_y = laser_y + range_min * sin(angle_ls);
      final_distance = range_min;
    }
    Vector2f final_intersection(final_x, final_y);
    scan[angle_i] = final_intersection;
  }
  
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr_arr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
  // Particle dummy[4];
  // dummy[0].loc = Vector2f(0,0);
  // dummy[0].angle = 0;
  // dummy[1].loc = Vector2f(-1,0);
  // dummy[1].angle = 0;
  // dummy[2].loc = Vector2f(0,0);
  // dummy[2].angle = 0;
  // dummy[3].loc = Vector2f(0,0);
  // dummy[3].angle = 0.1;
  // unsigned int number_of_elements = 4;

  for(unsigned int i=0;i<FLAGS_num_particles;i++)
  {
    vector<Vector2f> scan_ptr;
    Particle* p_ptr = &p_ptr_arr[i];
    // p_ptr = &dummy[i];//todo
    GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, ranges.size(),
                                              range_min,
                                              range_max,
                                              angle_min,
                                              angle_max,
                                              &scan_ptr);


    //computing distance from obstacle
    vector<float> point_distances;
    for(unsigned int j = 0; j< ranges.size(); ++j)
    {

     //the obstacle position
     float obst_x = scan_ptr[j].x();
     float obst_y = scan_ptr[j].y();

     float loc_x = p_ptr->loc.x();
     float loc_y = p_ptr->loc.y();

     float distance = sqrt(pow((obst_x - loc_x), 2) +  pow((obst_y - loc_y), 2));
     //cout<<distance<<endl;
     point_distances.push_back(distance);
    }

    //computing the L2 distance between point distances and scan_ptr
    float l2_distance_square = 0.0;
    for(unsigned int j = 0; j< ranges.size(); ++j)
    {
     l2_distance_square += pow(point_distances[j] - ranges[j], 2);
    }


    //computing weight
    float weight = exp(-l2_distance_square/(2 * sigma));
    p_ptr->weight = weight;
  } 
  // float sumval = 0;
  // for(unsigned int i=0;i<FLAGS_num_particles;i++)
  // {
  //   sumval += p_ptr_arr[i].weight;
  // } 
  // for(unsigned int i=0;i<FLAGS_num_particles;i++)
  // {
  //   p_ptr_arr[i].weight/=sumval;
  // } 
  // float meanval = 1/FLAGS_num_particles,stddev=0;
  // for(unsigned int i=0;i<FLAGS_num_particles;i++)
  // {
  //   stddev = (p_ptr_arr[i].weight-meanval)*(p_ptr_arr[i].weight-meanval);
  // } 
  // stddev/=FLAGS_num_particles;
  // stddev = sqrt(stddev);
  // std::cout<<"stddev : "<<stddev<<"\n";
  // for(unsigned int i=0;i<FLAGS_num_particles;i++)
  // {
  //   float val = (p_ptr_arr[i].weight-meanval)/stddev;
  //   if(val<0)
  //   {
  //     val = 0;
  //   }
  //   p_ptr_arr[i].weight = val;
  // } 
  // for(unsigned int i=0;i<FLAGS_num_particles;i++)
  // {
  //   std::cout<<"i : "<<i<<", "<<p_ptr_arr[i].weight<<"\n";
  // }

}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  // float x = rng_.UniformRandom(0, 1);
  // printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
  //        x);
  vector<Particle> new_particles;
  float total_weight = 0;
  for(unsigned int i=0; i<FLAGS_num_particles; i++){
    total_weight += particles_[i].weight;
  }

  for(unsigned int i=0; i<FLAGS_num_particles; i++){
    float rand_num = rng_.UniformRandom(0, 1);
    float running_sum = 0,j;
    for(j=0; j<FLAGS_num_particles; j++) {
      running_sum += particles_[j].weight/total_weight;
      if(rand_num<running_sum) {
        new_particles.push_back(particles_[j]);
        new_particles[i].weight = 1/FLAGS_num_particles;
        break;
      }
    }
    if(j==FLAGS_num_particles)
    {
      new_particles.push_back(particles_[FLAGS_num_particles-1]);
      new_particles[i].weight = 1/FLAGS_num_particles;
    }
  }
  particles_ = new_particles;

}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  // piazza post: check it
  // std::cout<<"inside2 : "<<"\n";
  // std::cout<<particles_.size()<<"\n";
  // std::cout<<"inside3 : "<<"\n";
  if(particles_.size()!=0) {
    Update(ranges, range_min, range_max, angle_min, angle_max, &particles_[0]);
    Resample();
  }
  
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.


  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  // float x = rng_.Gaussian(0.0, 2.0);
  // printf("Random number drawn from Gaussian distribution with 0 mean and "
  //        "standard deviation of 2 : %f\n", x);

  // try tharun's idea  
  
  // std::cout<<"x : "<<odom_loc.x()<<"\n";
  // std::cout<<"y : "<<odom_loc.y()<<"\n";
  // std::cout<<"odom_angle : "<<odom_angle<<"\n";
  if(!odom_initialized_) {
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
    return;
  }
  else
  {
    float dx, dy, dtheta,abs_dtheta;
    dx = odom_loc.x() - prev_odom_loc_.x();
    dy = odom_loc.y() - prev_odom_loc_.y();
    dtheta = odom_angle - prev_odom_angle_;

    float angle_estimate;
    Vector2f location_estimate;
    GetLocation(&location_estimate,&angle_estimate);

    // Vector2f loc_base;
    Vector2f loc_base = Eigen::Rotation2Df(-prev_odom_angle_)*Vector2f(dx,dy);

    // loc_base = 



    abs_dtheta = std::min(abs(dtheta), float(2*M_PI)-abs(dtheta));

    for(unsigned int i=0; i<particles_.size(); i++) {
      float ds, random_x, random_y, random_theta;
      ds = sqrt(dx*dx + dy*dy);
      random_x = rng_.Gaussian(0.0, k1_x * ds + k2_x * abs_dtheta);
      random_y = rng_.Gaussian(0.0, k1_y * ds + k2_y * abs_dtheta);
      Vector2f change_vec(random_x+loc_base.x(),random_y + loc_base.y());
      Vector2f rotated_map = Eigen::Rotation2Df(angle_estimate)*change_vec;
      random_theta = rng_.Gaussian(0, k3_theta * ds + k4_theta * abs_dtheta);
      particles_[i].loc.x() = particles_[i].loc.x() + rotated_map.x();
      particles_[i].loc.y() = particles_[i].loc.y() + rotated_map.y();
      particles_[i].angle = particles_[i].angle + dtheta + random_theta;
      // std::cout<<"Update "<<"\n";
    }
    prev_odom_loc_.x() = odom_loc.x();
    prev_odom_loc_.y() = odom_loc.y();
    prev_odom_angle_ = odom_angle;
    
  }

  
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  // std::cout<<"map file  : "<<map_file<<"\n";
  map_.Load("maps/GDC1.txt");

  // visualization::ClearVisualizationMsg(local_viz_msg_);
  robot_x = loc.x();
  robot_y = loc.y();
  robot_angle = angle;

  odom_initialized_ = false;
  // std::cout<<"robot_angle: "<<robot_angle<<"\n";


  particles_.clear();
  for(unsigned int i=0; i<FLAGS_num_particles; i++) {
      Particle particle;
      Vector2f predict_loc;
      // replace 0
      predict_loc.x() = robot_x + rng_.Gaussian(0.0, 0.1);
      predict_loc.y() = robot_y + rng_.Gaussian(0.0, 0.1);

      // visualization::DrawCross(loc,2, 0xFF0000, local_viz_msg_);

      particle.loc = predict_loc;
      particle.angle = robot_angle + rng_.Gaussian(0, 0.1);
      particle.weight = 1/FLAGS_num_particles;
      particles_.push_back(particle);

  }
  // viz_pub_.publish(local_viz_msg_);


}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  if(particles_.size()!=0) {
    Vector2f& loc = *loc_ptr;
    float& angle = *angle_ptr;
    // Compute the best estimate of the robot's location based on the current set
    // of particles. The computed values must be set to the `loc` and `angle`
    // variables to return them. Modify the following assignments:
    float mean_loc_x, mean_loc_y, mean_angle, weight_sum;
    mean_loc_x = 0;
    mean_loc_y = 0;
    mean_angle = 0;
    weight_sum = 0;
    for(unsigned int i=0; i<FLAGS_num_particles; i++) {
      float weight_i = particles_[i].weight;
      mean_loc_x += weight_i*particles_[i].loc.x();
      mean_loc_y += weight_i*particles_[i].loc.y();
      mean_angle += weight_i*particles_[i].angle;
      weight_sum += weight_i;
    }


    loc = Vector2f(mean_loc_x/weight_sum, mean_loc_y/weight_sum);
    angle = mean_angle/weight_sum;
  }
}


}  // namespace particle_filter
