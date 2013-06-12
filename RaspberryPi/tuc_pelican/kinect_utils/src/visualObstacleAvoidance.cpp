
#include "kinect_utils/visualObstacleAvoidance.h"

using namespace std;
using namespace cv;

inline int visualObstacleAvoidance::isAllSet(Mat& mask, Rect roi)
{
  // test if roi is outside
  if( roi.x<0 || roi.y<0 || roi.x+roi.width>=mask.cols || roi.y+roi.height>=mask.rows )
  {
    return -1;
  }
    
  // test if all pixel are set
  Scalar n_non_zero = countNonZero( mask(roi) );
//   cout << n_non_zero.val[0] << " of "<<roi.width*roi.height<<endl;
  if( n_non_zero.val[0] == roi.width*roi.height )
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

inline int visualObstacleAvoidance::isAllSetIntegral(Mat& integral_mask, Rect roi)
{
  // test if roi is outside
  if( roi.x<0 || roi.y<0 || roi.x+roi.width>=(integral_mask.cols-1) || roi.y+roi.height>=(integral_mask.rows-1) )
    return -1;
  
  // test if all pixel are set
  int n_non_zero = integralRect(integral_mask, roi.x, roi.y, roi.x+roi.width, roi.y+roi.height)/255;
//   cout << "integral rect="<<n_non_zero<<endl;
//     cout << n_non_zero << " of "<<roi.width*roi.height<<endl;
  if( n_non_zero == roi.width*roi.height )
  {
    return 1;
  }
  else
  {
    return 0;
  }
}


// try to find alternative route
// An alternative route is found by searching for the non-obstancle image reagion that is closest to the target. 
// Distance measure is sum of shift in x and y direction. For each shift sum (in increasing order) it is iterated 
// over all combinations of shifts in x and y direction. The first obstacle-free region that is found is the closest
// one and becomes the new route
int visualObstacleAvoidance::findAlternativeRoute(Mat& mask, int cur_vehicle_w, int cur_vehicle_h, int target_x, int target_y, int& shift_x, int& shift_y)
{
  
  // get integral image of mask (is used for fast checking if a certain recangular area is completely covered with value 255 in isAllSetIntegral() )
  Mat integral_mask;
  integral(mask, integral_mask);
  
  // find alternative free route with minimum distance from target
  //  While loop over increasing shift distance
  //  in each iteration is a for loop that divides the shift distance in x and y part and test if this yields a free route
  int shift_sum=1;
  int foundRoute_flag=0;   // is set, when a alternative route has been found and the search can be aborted
  int notAllOutside_flag=1; // is set if there has been a shift combination at the previous iteration where the vehicle rect was not outside image
  int isAllSet_flag;
  while( !foundRoute_flag && notAllOutside_flag)
  {
    notAllOutside_flag=0;
    // divide in shift x and shift y
    for(shift_y = 0; shift_y<=shift_sum; shift_y++)
    {
      shift_x = shift_sum-shift_y;
      
      // test for shift_x positive, shift_y positive
      isAllSet_flag = isAllSetIntegral(integral_mask, Rect(target_x-cur_vehicle_w/2+shift_x, target_y-cur_vehicle_h/2+shift_y, cur_vehicle_w, cur_vehicle_h) );
      if( isAllSet_flag>0 )
      {
        foundRoute_flag=1;
        break;
      }
      else if( isAllSet_flag>=0 )
      {
        notAllOutside_flag = 1;
      }
      
      // test for shift_x positive, shift_y negative
      isAllSet_flag = isAllSetIntegral(integral_mask, Rect(target_x-cur_vehicle_w/2+shift_x, target_y-cur_vehicle_h/2-shift_y, cur_vehicle_w, cur_vehicle_h) );
      if( isAllSet_flag>0 )
      {
        shift_y *= -1;
        foundRoute_flag=1;
        break;
      }
      else if( isAllSet_flag>=0 )
      {
        notAllOutside_flag = 1;
      }
      
      // test for shift_x negative, shift_y positive
      isAllSet_flag = isAllSetIntegral(integral_mask, Rect(target_x-cur_vehicle_w/2-shift_x, target_y-cur_vehicle_h/2+shift_y, cur_vehicle_w, cur_vehicle_h) );
      if( isAllSet_flag>0 )
      {
        shift_x *= -1;
        foundRoute_flag=1;
        break;
      }
      else if( isAllSet_flag>=0 )
      {
        notAllOutside_flag = 1;
      }
      
      // test for shift_x negative, shift_y negative
      isAllSet_flag = isAllSetIntegral(integral_mask, Rect(target_x-cur_vehicle_w/2-shift_x, target_y-cur_vehicle_h/2-shift_y, cur_vehicle_w, cur_vehicle_h) );
      if( isAllSet_flag>0 )
      {
        shift_x *= -1;
        shift_y *= -1;
        foundRoute_flag=1;
        break;
      }
      else if( isAllSet_flag>=0 )
      {
        notAllOutside_flag = 1;
      }
      
    } // for shift_y
    shift_sum++;
  } // while( !foundRoute_flag && notAllOutside_flag )
  
  return foundRoute_flag;
}

// Take care of valid sensor_depth_max
// Target direction is hard coded to image center
//
// status
// 0 ... no obstacle in target direction (-> direction is set to target direction at distance sensor_depth_max)
// 1 ... obstacle and alternative route in Point3f direction (-> direction yields to the point on the alternative route next to the obstacle)
// 2 ... obstacle, but no suggestion for alternative Route (-> direction yields to a point vehicle_d/2 in front of the obstacle)
// 
// direction (in meters) is in local coordinate system: x to the right, y down, z in camera viewing direction, origin is in image mid point
int visualObstacleAvoidance::visualObstacleAvoidance(Mat& input, Point3f& direction, int& status, float target_x, float target_y)
{
  // if set, input and resulting route are shown
  int visualization_flag = 0;
    
  //===================== parameter =====================
  float scale = 0.25; // scale image
    
  int mask_thresh_min = 35; // must be >0
  int mask_thresh_step = 10;
  int mask_thresh_max = 255;
  
  float focal_length = 580*scale; // focal length of IR camera in pixel
  float vehicle_h = 0.7;      // vehicle heigh in meters
  float vehicle_w = 1;      // vehicle width in meters
  float vehicle_d = 1;      // vehicle depth in meters (only used if there is an non avoidable obstacle and the returned dirction yields to a point in formt of the obstacle)
  
  // Parameters and equations for computing size of the vehicle in the image (see: http://www.ros.org/wiki/kinect_calibration/technical ):
  //      size_in_image = size_in_world * focal_length / depth
  // A linear connection betwwen pixel value in depth image and the depth is assumed:
  //      depth = (sensor_depth_max / 254)  * depth_image_value 
  // The maximum depth image value is 254, representing sensor_depth_max, 255 represents a non valid measurement (e.g. out of sensor range)
  float sensor_depth_max = 7;     // maximum depth in meters  
  float depth_to_pixel_scale = focal_length / (sensor_depth_max / 254); // transformation between depth and size in image
  
  // target direction/position in image
  if(target_x<0) target_x = input.cols/2;
  if(target_y<0) target_y = input.rows/2;
      
  // scale image TODO use min() function of area for downscaling
  Mat scaled_input;
  resize(input, scaled_input, Size(), scale, scale);
  
  // scale target
  target_x *= scale;
  target_y *= scale;
  
  //===================== search for obstacles ============================
  // iterate over different depth values and search for obstacles
  // interrupt if there is no alternative route (=obstacle is too large)
  status = 0;
  int nonAvoidableObstacle_flag=0; // is set if there is an obstacle that is too large to find an alternative route
  int mask_thresh;
  Mat mask, mask_visualization;
  int cur_vehicle_h, cur_vehicle_w, shift_x, shift_y;
  
  for(mask_thresh = mask_thresh_min; mask_thresh<=mask_thresh_max && !nonAvoidableObstacle_flag; mask_thresh += mask_thresh_step)
  {       
    // create binary mask with all pixel set that are smaller than mask_thresh (closer to camera than corresponding distance)
    threshold(scaled_input, mask, mask_thresh, 255, THRESH_BINARY);
    
    // vehicle size in pixel at this iteration
    cur_vehicle_h = ceil(vehicle_h*depth_to_pixel_scale / mask_thresh);
    cur_vehicle_w = ceil(vehicle_w*depth_to_pixel_scale / mask_thresh);
    
    // is obstacle detection possible at mask_thresh or is it too close?
    if(cur_vehicle_h>scaled_input.rows || cur_vehicle_w>scaled_input.cols)
    {
      cout << "visualObstacleAvoidance(): no obstacle detection possible at mask_thresh="<<mask_thresh<<" ("<<mask_thresh*sensor_depth_max / 254<<" meters), it is too close"<<endl;
    }
    else
    {      
      // is there an obstacle in target direction?
      if( !isAllSet(mask, Rect(target_x-cur_vehicle_w/2, target_y-cur_vehicle_h/2, cur_vehicle_w, cur_vehicle_h) ) > 0 )
      {
        // try to find alternative route
        if( findAlternativeRoute( mask, cur_vehicle_w, cur_vehicle_h, target_x, target_y, shift_x, shift_y) )
        {
          // found alternative route
          status = 1;        
        }
        else
        {
          // there is no alternative route
          status = 2;
          nonAvoidableObstacle_flag = 1;
        }
        
        // stop at this mask_thresh -> if this is not done, one can get a complete route to the maximum depth by fetching shift_x and shift_y after each iteration
        break;
        
      } // if obstacle in target direction
    } // if-else obstacle detection possible
    
  } // for mask_thresh
  
  
  //===================== set direction ============================
  if(visualization_flag)
    cvtColor(mask, mask_visualization, CV_GRAY2RGB);
      
  // if there was no obstacle
  if(status == 0)
  {
    // direction is set to target direction at distance sensor_depth_max in 3d Space
    int x_image_center_frame = target_x - scaled_input.cols/2;
    int y_image_center_frame = target_y - scaled_input.rows/2;
    
    float shift_x_3d = x_image_center_frame*sensor_depth_max/focal_length;
    float shift_y_3d = y_image_center_frame*sensor_depth_max/focal_length;
    
  
    direction= Point3f( shift_x_3d, shift_y_3d, sensor_depth_max*mask_thresh_max/254);
    
    // draw rectangle to visualize current vehicle size and position
    if(visualization_flag)
      rectangle(mask_visualization, Point( target_x-cur_vehicle_w/2, target_y-cur_vehicle_h/2),Point( target_x+cur_vehicle_w/2, target_y+cur_vehicle_h/2), Scalar(0, 255, 255), 4);  
  }
  
  // if there was an obstacle and an alternative route
  if(status == 1)
  {
    // direction yields to the point on the alternative route next to the obstacle in 3d Space
    int x_image_center_frame = target_x + shift_x - scaled_input.cols/2;
    int y_image_center_frame = target_y + shift_y - scaled_input.rows/2;
    float distance_to_obstacle = mask_thresh * sensor_depth_max/254;
    
    float shift_x_3d = x_image_center_frame*distance_to_obstacle/focal_length;
    float shift_y_3d = y_image_center_frame*distance_to_obstacle/focal_length;
    
    direction= Point3f( shift_x_3d, shift_y_3d, distance_to_obstacle);
    
    // draw rectangle to visualize alternative Route
    if(visualization_flag)
      rectangle(mask_visualization, Point( target_x-cur_vehicle_w/2 + shift_x, target_y-cur_vehicle_h/2 + shift_y),Point( target_x+cur_vehicle_w/2 + shift_x, target_y+cur_vehicle_h/2 + shift_y), Scalar(0, 255, 0), 4);
  }
  
  // if there was an obstacle and NO alternative route
  if(status == 2)
  {
    // direction yields to a point vehicle_d/2 in front of the obstacle in 3d Space
    float distance_to_obstacle = mask_thresh *  sensor_depth_max/254;
    direction = Point3f(0, 0, distance_to_obstacle - vehicle_d/2);
    
    // draw red rectangle to visualize that there is no route
    if(visualization_flag)
      rectangle(mask_visualization, Point( target_x-cur_vehicle_w/2, target_y-cur_vehicle_h/2),Point( target_x+cur_vehicle_w/2, target_y+cur_vehicle_h/2), Scalar(0, 0, 255), 4);
    
    cout << status << endl;
  }
  
  //===================== visualization ============================ 
  if(visualization_flag)
  {
    cvShowImage("input", &IplImage(scaled_input));
    cvShowImage("mask", &IplImage(mask_visualization));
  }
  
  return status;
}

void visualObstacleAvoidance::world2imageCoord(Point3f& worldCO, Point2f& imageCO, float focal_length, Size imageSize)
{
  imageCO.x = (focal_length * worldCO.x / worldCO.z) + imageSize.width/2;
  imageCO.y = (focal_length * worldCO.y / worldCO.z) + imageSize.height/2;
}

