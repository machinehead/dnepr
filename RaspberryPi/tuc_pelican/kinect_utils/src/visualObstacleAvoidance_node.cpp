#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include "kinect_utils/visualObstacleAvoidance.h"

#include <iostream>


// The ROS publisher class for visualization of directions
ros::Publisher pub_marker;

using namespace std;
using namespace cv;

// test node for visualObstacleAvoidance: subscribes depth image: /camera/depth/image_raw and publishes valid flying direction: obstacle_avoidance_direction

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
  //================ get image and convert to cv::Mat ==============
  sensor_msgs::CvBridge bridge;
  Mat input;
  try
  {
    input = bridge.imgMsgToCv(msg, "mono8");
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
  
//   input = imread("with_obstacle.png", CV_8UC1);
//   input = imread("no_way_out.png", CV_8UC1);
//   imwrite("image.png", input); return;
  
  Point3f direction;
  int status;
    
  double t1 = startTimeMeasure();
  visualObstacleAvoidance::visualObstacleAvoidance(input, direction, status);
  cout << "status: "<<status << "    --> direction: "<< direction.x << " "<<direction.y << " "<< direction.z << "      TIME: "<<stopTimeMeasure(t1)<< endl;
  
  //============== publish new direction ==================
  visualization_msgs::Marker marker;
  
  // init marker
  marker.header.frame_id = "/kinect_depth";
  marker.header.stamp = ros::Time::now();
  marker.ns = "obstacle_avoidance_direction";
  
  marker.points.clear();
  geometry_msgs::Point startPoint; startPoint.x=0; startPoint.y=0; startPoint.z=0;
  geometry_msgs::Point endPoint; endPoint.x=direction.x; endPoint.y=direction.y; endPoint.z=direction.z;
  
  marker.points.push_back( startPoint );
  marker.points.push_back( endPoint );
    
  marker.scale.x = 0.1;
  marker.scale.y = 0.3;
  marker.scale.z = 0.1;
    // Set the color 
  switch(status)
  {
    case 0: // free
      marker.color.r = 0.0f; marker.color.g = 1.0f;  marker.color.b = 0.0f; marker.color.a = 1;break;
    case 1: // obstacle
      marker.color.r = 0.0f; marker.color.g = 0.0f;  marker.color.b = 1.0f; marker.color.a = 1;break;
    case 2: // non avoidable obstacle
      marker.color.r = 1.0f; marker.color.g = 0.0f;  marker.color.b = 0.0f; marker.color.a = 1;break;
    default:
      marker.color.r = 1.0f; marker.color.g = 1.0f;  marker.color.b = 1.0f; marker.color.a = 1;break;
  }
  pub_marker.publish(marker);
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cvNamedWindow("input");
  cvNamedWindow("mask");
  
  cvStartWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/depth/image_raw", 1, imageCallback);
  
  pub_marker = nh.advertise<visualization_msgs::Marker>("obstacleAvoidance_direction_marker", 1);
  
  ros::spin();
  cvDestroyWindow("view");
}