#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include <cmath>

#include "kinect_utils/resamplePointCloud.h"

ros::Publisher pointcloud_pub;

using namespace std;

/** 
resamples point cloud of a PointCloud2  message and
publishes resulting cloud as topic: resampled_cloud

Uses default parameters of resample() function.

example usage: rosrun kinect_utils resamplePointCloud_node input_cloud:=/camera/points2 _resample_method:=bitGrid
This should provide a topic /resampled_cloud with the resampled data

*/

// resample point cloud 
void resamplePointCloud_node_bitGrid_callback(const sensor_msgs::PointCloud2& msg)
{
  // resample
  pcl::PointCloud<pcl::PointXYZ> resampledPC; // storage for resampled and converted point cloud
  resample(msg, resampledPC);
  
  // convert back to message and publish
  sensor_msgs::PointCloud2 msg2;  
  pcl::toROSMsg(resampledPC, msg2);
  pointcloud_pub.publish(msg2);
}

// resample point cloud 
void resamplePointCloud_node_voxelGrid_callback(const sensor_msgs::PointCloud2& msg)
{
  // resample
  sensor_msgs::PointCloud2 msg2; // storage for resampled point cloud
  resample(msg, msg2);
  
  // publish
  msg2.header.stamp = ros::Time::now();
  pointcloud_pub.publish(msg2);
}

int main(int argc, char **argv)
{
  std::string point_cloud_topic_;
  std::string resample_method_;
  
  // init
  ros::init(argc, argv, "resamplePointCloud_node");
  ros::NodeHandle n("~");
  n.param("point_cloud_topic", point_cloud_topic_, std::string("/input_cloud"));
//   n.param("resample_method", resample_method_, std::string("haha"));
  
  
  // subscribe
  ros::Subscriber sub;
  
  // get parameter 
  string resample_method_value;
  if(n.getParam("resample_method", resample_method_value))
  {
    if( resample_method_value == "bitGrid")
    {
      sub = n.subscribe(point_cloud_topic_, 1, resamplePointCloud_node_bitGrid_callback);
      ROS_INFO("Using bitGrid method");
    }
    else if( resample_method_value == "voxelGrid")
    {
      sub = n.subscribe(point_cloud_topic_, 1, resamplePointCloud_node_voxelGrid_callback);
      ROS_INFO("Using voxelGrid method");
    }
  }
  else
  {
    ROS_INFO("Using default (and slow) voxelGrid method, try _resample_method:=bitGrid");
    sub = n.subscribe(point_cloud_topic_, 1, resamplePointCloud_node_voxelGrid_callback);
  }
   
  // publish
  pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>("/resampled_cloud", 1);
    
  // spin
  ros::spin();

  return 0;
}