#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <iostream>

#include "kinect_utils/helper.h"
#include "kinect_utils/timer.h"

namespace visualObstacleAvoidance
{

  /**
  returns 1 if all pixel of mask in roi are non zero, otherwise 0
  */
  inline int isAllSet(cv::Mat& mask, cv::Rect roi);

  /**
  returns 1 if all pixel in roi are 255
  */
  inline int isAllSetIntegral(cv::Mat& integral_mask, cv::Rect roi);

  /**
  try to find alternative route
  An alternative route is found by searching for the non-obstancle image reagion that is closest to the target. 
  Distance measure is sum of shift in x and y direction. For each shift sum (in increasing order) it is iterated 
  over all combinations of shifts in x and y direction. The first obstacle-free region that is found is the closest
  one and becomes the new route
  */
  int findAlternativeRoute(cv::Mat& mask, int cur_vehicle_w, int cur_vehicle_h, int target_x, int target_y, int& shift_x, int& shift_y);


  /** 
    Check for obstacles
    
    Input image should be a grey value depth image (0..255, 255 indicating unknown state)
    
    There are a lot of parameters inside this function:
      - take care of valid sensor_depth_max (maximum depth range of the sensor)
      - size of vehicle is hard coded
      - focal length is hard coded
      
    Return values:
      - return status (also stored in provided parameter)
          0 ... no obstacle in target direction (-> direction is set to target direction at maximum checked distance)
          1 ... obstacle and alternative route in Point3f direction (-> direction yields to the point on the alternative route next to the obstacle)
          2 ... obstacle, but no suggestion for alternative Route (-> direction yields to a point vehicle_d/2 in front of the obstacle)
      - direction (in meters) is in local coordinate system: x to the right, y down, z in camera viewing direction, origin is in image mid point
      
  */
  int visualObstacleAvoidance(cv::Mat& input, cv::Point3f& direction, int& status, float target_x=-1, float target_y=-1);



  /**
    Get coordinates of a point in 3d coordinates in image plane. 
    @param worldCO right hand CO, centered at focal point, x to the right, y to bottom, z in viewing direction of the camera
    @param imageCO right hand CO, centered at top left point in image, x to the right, y to bottom
    @param focal_length camera focal length in pixels
    @param imageSize size of the image
    
  */
  void world2imageCoord(cv::Point3f& worldCO, cv::Point2f& imageCO, float focal_length = 580, cv::Size imageSize = cv::Size(640, 480));
  
}