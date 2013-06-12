#include "ros/ros.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"

#include "kinect_utils/timer.h"

#include <vector>
#include <cmath>

#pragma once

/// resample sensor_msgs::PointCloud2 using pcl::VoxelGrid
void resample(const sensor_msgs::PointCloud2 &inputPC, sensor_msgs::PointCloud2& outputPC, float x_resolution=0.1, float y_resolution=0.1, float z_resolution=0.1);

/// resample sensor_msgs::PointCloud2 and convert to pcl::PointCloud
/**
Resample sensor_msgs::PointCloud2 and convert to pcl::PointCloud

The maximum values of the points are not checked! It will probably crash if there are points outside the given grid range. Points with NaN x-coordinates are ignored. 

@param x_resolution resolution (size of grid cell, e.g. in meters)
@param y_resolution resolution (size of grid cell, e.g. in meters)
@param z_resolution resolution (size of grid cell, e.g. in meters)

@param x_max x_max is maximum x value of point, -x_max is minimum x_value of point (e.g. in meters)
@param y_max y_max is maximum x value of point, -y_max is minimum y_value of point (e.g. in meters)
@param z_max z_max is maximum x value of point, 0 is minimum z_value of point (e.g. in meters)

@param estimated_number_of_output_points Size of preallocated memory, a good value can yield a (small) speedup. Bad estimates does not cause any damage.

@param insertGridMidpoints_flag If this flag is set, the points in outputPC are midpoints of grid cells, otherwise they are a subset of the inputPC, one representative for each covered cell (the first point of input cloud that is parsed)

*/
void resample(const sensor_msgs::PointCloud2 &inputPC, pcl::PointCloud<pcl::PointXYZ> &outputPC,  float x_resolution=0.1, float y_resolution=0.1, float z_resolution=0.1, uint32_t x_max=10, uint32_t y_max=10, uint32_t z_max=10, uint32_t estimated_number_of_output_points=3000, uint8_t insertGridMidpoints_flag=0);

