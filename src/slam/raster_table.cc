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
#include <string>

#include "vector_map/vector_map.h"

#define TIMES_SIGMA 7

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


class RasterTable{
public:
	float raster_size,raster_resolution,sigma_raster,times_sigma;
	int num_rows,num_columns;
	vector<vector<float>> table;
	RasterTable()
	{

	}
	void createRasterTable(float raster_size,float raster_resolution,float sigma_raster)
	{
		this->raster_size = raster_size;
		this->raster_resolution = raster_resolution;
		this->sigma_raster = sigma_raster;
		this->times_sigma = TIMES_SIGMA;
		num_rows = 2*((int) (raster_size/raster_resolution) ) + 1;
		num_columns = num_rows;
		for(int i = 0;i<num_rows;i++)
		{
			vector<float> temp;
			for(int j=0;j<num_columns;j++)
			{
				temp.push_back(1.0);
			}
			table.push_back(temp);
		}
	}
	void clear()
	{
		for(int i = 0;i<num_rows;i++)
		{
			for(int j=0;j<num_columns;j++)
			{
				table[i][j] = 0;
			}
		}
	}
	void normalize()
	{
		float sumval = 0;
		for(int i = 0;i<num_rows;i++)
		{
			for(int j=0;j<num_columns;j++)
			{
				sumval += table[i][j];
			}
		}
		for(int i = 0;i<num_rows;i++)
		{
			for(int j=0;j<num_columns;j++)
			{
				table[i][j] /= sumval;
			}
		}
	}
	void lookup_indices(Vector2f point,int& x_index,int& y_index)
	{
		x_index = floor( (point.x() + raster_size + 0.5*raster_resolution) / raster_resolution );
		y_index = floor( (point.y() + raster_size + 0.5*raster_resolution) / raster_resolution );
	}
	float lookup(Vector2f point)
	{
		int x_index,y_index;
		this->lookup_indices(point,x_index,y_index);
		// cout<<"x_index : "<<x_index<<",y_index : "<<y_index<<endl;
		// cout<<"score : "<<table[y_index][x_index]<<endl;
		return table[y_index][x_index];
	}
	// void populate_point(Vector2f point)
	// {
	// 	for(int y_i = -((int) (raster_size/raster_resolution) );y_i<=((int) (raster_size/raster_resolution) );y_i++)
	// 	{
	// 		for(int x_i = -((int) (raster_size/raster_resolution) );x_i<=((int) (raster_size/raster_resolution) );x_i++)
	// 		{
	// 			float x_center = x_i*raster_resolution;
	// 			float y_center = y_i*raster_resolution;
	// 			float xdiff = x_center - point.x();
	// 			float ydiff = y_center - point.y();
	// 			float distance = sqrt(xdiff*xdiff+ydiff*ydiff);
	// 			float value = exp( -(distance*distance)/(2*sigma_raster*sigma_raster) )/(sqrt(2*M_PI)*sigma_raster);
	// 			table[y_i+((int) ((num_rows-1)/2))][x_i+((int) ((num_rows-1)/2))] += value;
	// 		}
	// 	}
	// }
	void populate_point(Vector2f point)
	{
		float x_left = point.x()-this->times_sigma*sigma_raster;
		float x_right = point.x()+this->times_sigma*sigma_raster;
		float y_bottom = point.y()-this->times_sigma*sigma_raster;
		float y_top = point.y()+this->times_sigma*sigma_raster;
		int x_left_ind,x_right_ind,y_bottom_ind,y_top_ind;
		this->lookup_indices(Vector2f(x_left,y_bottom),x_left_ind,y_bottom_ind);
		this->lookup_indices(Vector2f(x_right,y_top),x_right_ind,y_top_ind);
		if(x_left_ind<0)
		{
			x_left_ind = 0;
		}
		if(x_right_ind>(num_columns-1))
		{
			x_right_ind = num_columns - 1;
		}
		if(y_bottom_ind<0)
		{
			y_bottom_ind = 0;
		}
		if(y_top_ind>(num_rows-1))
		{
			y_top_ind = num_rows - 1;
		}

		for(int y_i = y_bottom_ind;y_i<=y_top_ind;y_i++)
		{
			for(int x_i = x_left_ind;x_i<=x_right_ind;x_i++)
			{
				float x_center = (x_i - ((int) ((num_columns-1)/2)) )*raster_resolution;
				float y_center = (y_i - ((int) ((num_rows-1)/2)) )*raster_resolution;
				float xdiff = x_center - point.x();
				float ydiff = y_center - point.y();
				float distance = sqrt(xdiff*xdiff+ydiff*ydiff);
				float value = exp( -(distance*distance)/(2*sigma_raster*sigma_raster) )/(sqrt(2*M_PI)*sigma_raster);
				if(value>table[y_i][x_i])
				{
					table[y_i][x_i] = value;
				}
			}
		}
	}
	void populate_pointcloud(vector<Vector2f> point_cloud)
	{
		this->clear();
		for(auto cur_point : point_cloud)
		{
			if( (abs(cur_point.x()) <= raster_size ) && (abs(cur_point.y()) <= raster_size ) )
			{
				populate_point(cur_point);
			}
		}
		this->normalize();
	}
	float lookup_pointcloud(vector<Vector2f> point_cloud)
	{
		int num_accepted = 0;
		float log_score_sum = 0;
		for(auto cur_point : point_cloud)
		{
			if( (abs(cur_point.x()) <= (raster_size) ) && (abs(cur_point.y()) <= (raster_size) ) )
			{
				num_accepted += 1;
				log_score_sum += log(this->lookup(cur_point));
			}
		}
		if(num_accepted == 0)
		{
			// cout<<"num_accepted : "<<0<<endl;
			return -1;
		}
		else
		{
			// cout<<"num_accepted : "<<(num_accepted)<<endl;
			return log_score_sum/num_accepted;
		}
	}
	void print_table()
	{
		printf("%3d",1);
		for(int i=0;i<num_columns;i++)
		{
			printf(" %6d",i);
		}
		printf("\n");
		for(int row=(num_rows-1);row>=0;row--)
		{
			printf("%3d",row);
			for(int column=0;column<num_columns;column++)
			{
				printf(" %4.4f",table[row][column]);
			}
			printf("\n");
		}
	}

};

