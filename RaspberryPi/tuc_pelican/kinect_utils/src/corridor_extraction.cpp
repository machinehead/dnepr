#include <iostream>
#include <fstream>
#include <libtimer/timer.h>

#include "kinect_utils/corridor_extraction.h"
#include "kinect_utils/resamplePointCloud.h"
#include "kinect_utils/visualObstacleAvoidance.h"

CorridorExtraction::CorridorExtraction()
{
  ros::NodeHandle nh;

  // set up publishers  
  pub_kinectPose = nh.advertise<kinect_utils::kinectPose>("kinectPose", 1);
  pub_unmatched = nh.advertise<sensor_msgs::PointCloud2>("unmatchedPoints", 1);
  pub_wall_marker = nh.advertise<visualization_msgs::Marker>("wall_marker", 1);
  pub_command_marker = nh.advertise<visualization_msgs::Marker>("flightCommand_marker", 1);
  pub_command = nh.advertise<kinect_utils::flightCommand>("flightCommand",1);


  // fill default values
  kinectPose.distance_left=-1;    kinectPose.distance_right=-1;
  kinectPose.distance_floor=-1;   kinectPose.distance_ceiling=-1;
  kinectPose.distance_front=-1;

  kinectPose.roll=0; kinectPose.pitch=0; kinectPose.yaw=0;

  max_n_planes = 20;
}


// ===================================================================
void CorridorExtraction::rangeImageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{  
  try {
    mutex.lock();
    cv::Mat tempMat = cvBridge.imgMsgToCv(image_msg, "mono8");
    rangeImage = tempMat.clone();
    mutex.unlock();
  }
  catch (sensor_msgs::CvBridgeException& e) {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", image_msg->encoding.c_str());
  }
    
}


// ===================================================================
void CorridorExtraction::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
  
  // ========== extract and downsample the point cloud =============

  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
 /*  
  // use Peer's faster function for combined message conversion and point cloud downsampling  
  startTimer();
  resample(*cloudMsg, cloud_filtered,0.1,0.1,0.1,5,5,5);
  double t0=stopTimer(); 
  */

  
  
  // or use PCL's voxel grid filter, this is much slower
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*cloudMsg, cloud);
  vg.setInputCloud(cloud.makeShared()); 
  vg.setFilterLimits(0.2,5.0);
  vg.setFilterFieldName("z");
  vg.setLeafSize (0.1, 0.1, 0.1);   // keep one point every x*y*z meter or something
  vg.filter(cloud_filtered);
  double t0=0;
  

  // process the data (results are saved in member variables)  
  // after the function returns, cloud_filtered contains only those points that could not be matched to any plane
  startTimer();
  
  extractPlanes(cloud_filtered);
  if (pub_unmatched.getNumSubscribers() > 0) pcl::toROSMsg(cloud_filtered, unmatchedPointsMsg);
  double t1=stopTimer();

  startTimer();
  extractPlaneOrientations(); 
  double t2=stopTimer();

  startTimer();
  extractPlaneTypes();
  double t3=stopTimer();

  startTimer();
  estimateCameraOrientationAndDistances();
  double t4=stopTimer();


  // determine the best flight path along the corridor
  startTimer();
  findFlightPath(); 
  double t5=stopTimer();

  // publish the results
  startTimer();
  publish();
  double t6=stopTimer();

  // write times to file
  ofstream logfile;
  logfile.open ("logfile_time.txt",fstream::out |fstream::app );
  logfile << ros::Time::now() << " "<<cloudMsg->header.stamp<< " "<<ros::Time::now()-cloudMsg->header.stamp<< endl;
  logfile.close();
  
  
  
  ROS_INFO("Run-time information:");
  ROS_INFO("\tconvert and downsample point cloud message: %f", t0); 
  ROS_INFO("\textractPlanes: %f", t1);
  ROS_INFO("\textractPlaneOrientations: %f", t2);
  ROS_INFO("\textractPlanesTypes: %f", t3);
  ROS_INFO("\testimateCameraOrientationAndDistances: %f", t4);
  ROS_INFO("\tflightPath: %f", t5);
  ROS_INFO("\tpublish: %f", t6);
  ROS_INFO("\toverall: %f",t0+t1+t2+t3+t4+t5+t6);
}

// ===================================================================
void CorridorExtraction::findFlightPath()
{  
  float dx, dy_left, dy_right, dy, dz, dyaw;

  const double minFloorDistance=0.2;
  const double minCeilingDistance=1.5;
  const double minWallDistance=1.00;
  const double minDistanceFront=1.0;
  const double floorWidth=2.0;
        
  
  // === horizontal control ===

  // calculate dy according to left and right wall measurements and fuse both using the weights (number of 3d points supporting the plane)
  if (kinectPose.distance_left>=0) dy_left = floorWidth/2 - kinectPose.distance_left;
  else dy_left=0;
            
  if (kinectPose.distance_right>=0) dy_right = -floorWidth/2 + kinectPose.distance_right;
  else dy_right=0;
  
  dy = (dy_left*weight_left + dy_right*weight_right) / (weight_left+weight_right);
  

/*
  // if we see both the left and right wall we want to fly right in the middle between both
  if (kinectPose.distance_right>0 && kinectPose.distance_left>0) {  
    dy = (weight_right*kinectPose.distance_right - weight_left*kinectPose.distance_left) / (weight_right+weight_left);
  }
  // if we only see one wall, we only keep a certain minimum distance from it
  else if (kinectPose.distance_right>0) {
    dy = min(0.0, (kinectPose.distance_right - minWallDistance));
  }
  else if (kinectPose.distance_left>0) {
    dy = -min(0.0, (kinectPose.distance_left - minWallDistance));
  }
  // if we do not see any of the left or right wall, we are pretty helpless here
  else {
    dy=0;
  }
*/

  // === vertical control ===
        
  // same as above, if we see both floor and ceiling ...
  if (kinectPose.distance_floor>0 and kinectPose.distance_ceiling>0) {
    dz = (weight_floor*kinectPose.distance_floor - weight_ceiling*kinectPose.distance_ceiling) / (weight_floor+weight_ceiling);
  }
  // if we only see one of them, we only keep a certain minimum distance from it
  else if (kinectPose.distance_floor>0) {
    dz = min(0.0, (kinectPose.distance_floor - minFloorDistance));
  }
  else if (kinectPose.distance_ceiling>0) {
    dz = -min(0.0, (kinectPose.distance_ceiling - minCeilingDistance));
  }
  // if we do not see the floor nor the ceiling, we are pretty helpless here
  else {
    dz=0;
  }
  
  // === lateral control ===
  
  // if there is no wall in front of us, just go on
  if (kinectPose.distance_front < 0 ) { 
    dx=1;
  }
  // if a wall is coming up in front of us, approach slowly
  else if (kinectPose.distance_front > minDistanceFront) {
    dx=min(1.0,kinectPose.distance_front/2.0);
  }
  // in all other cases, do not proceed any further 
  else {
    dx=0;
  }

  
  // === yaw control ===
  // this is fairly easy, just try to get yaw to 0
  if (!isnan(kinectPose.yaw)) {
    dyaw = -kinectPose.yaw;
  }
  else {
    dyaw = 0;
  }
 
  // at this point, (dx, dy) is in the corridor coord sys, i.e. X points down the corridor, Y is to the right, perpendicular to the walls 
  // so we ...
  // === rotate (dx, dy) into the 'external' camera coordinate system, which is X in viewing direction, Y to the right ===
  float dxr=cos(dyaw)*dx - sin(dyaw)*dy;
  float dyr=sin(dyaw)*dx + cos(dyaw)*dy;
  dx=dxr; dy=dyr;
  
  // now (dx, dy) is in external camera coordinates, which is X front, Y right
  ROS_INFO("dx: %0.3f dy: %0.3f dz: %0.3f dyaw: %0.3f", dx, dy, dz, dyaw);
 
  // === build raw flightCommand message ===
  flightCommand.dx_raw=dx;
  flightCommand.dy_raw=dy;
  flightCommand.dz_raw=dz;
  flightCommand.dyaw_raw=dyaw;

  
  // === convert 3D coords into 2D image coords ===
  cv::Point2f imageDirection;
  cv::Point3f p(dy, dz, dx);   // turn (dx, dy, dz) into the internal camera frame coord sys (Z front, X right, Y down)
  visualObstacleAvoidance::world2imageCoord(p,imageDirection);
  
  // === check for obstacles ===  
  cv::Point3f direction;
  int status;
  
  
  // if there is an image and the projected point coordinates are inside the image, check if there is an obstacle, otherwise: do not move
  mutex.lock();
  if (rangeImage.cols>0 && imageDirection.x>=0 && imageDirection.x<rangeImage.cols && imageDirection.y>=0 && imageDirection.y<rangeImage.rows)
  {
    visualObstacleAvoidance::visualObstacleAvoidance(rangeImage, direction, status, imageDirection.x, imageDirection.y);
  }
  else
  {
    direction = cv::Point3f(0,0,0);
  }
  mutex.unlock();
  
  
  // === build flightCommand message that is going to be published ===
  // direction is given in the internal camera frame coord sys (Z front, X right, Y down), turn it around into external camera coord sys (X front, Y right, Z down)
  flightCommand.dx=direction.z;
  flightCommand.dy=direction.x;
  flightCommand.dz=direction.y;
  flightCommand.dyaw=dyaw;

  ofstream logfile;
  logfile.open ("logfile_cmd.txt",fstream::out |fstream::app );
  logfile << ros::Time::now() <<" "<< flightCommand.dx << " "<<flightCommand.dy << " "<<flightCommand.dz << " "<<flightCommand.dyaw << " "<< flightCommand.dx_raw << " "<<flightCommand.dy_raw << " "<<flightCommand.dz_raw << " "<<flightCommand.dyaw_raw << endl; 
  logfile.close();

  ROS_INFO("direction: %0.3f %0.3f %0.3f", direction.x, direction.y, direction.z);
}



// ===================================================================
void CorridorExtraction::publish() 
{    
  // publish combined orientation and wall distances
  if (pub_kinectPose.getNumSubscribers() >0)
  {
    kinectPose.header.stamp = ros::Time::now();
    pub_kinectPose.publish(kinectPose);     
  }

  // publish point cloud of unmatched points
  if (pub_unmatched.getNumSubscribers() >0) pub_unmatched.publish(unmatchedPointsMsg);
  

  // publish plane visualization
  if (pub_wall_marker.getNumSubscribers() >0) publish_plane_marker();
  
  // publish flight command visualization
  if (pub_command_marker.getNumSubscribers() >0) publish_command_marker();
  
  // publish commands for the UAV
  if (pub_command.getNumSubscribers() >0)
  {
    flightCommand.header.stamp = ros::Time::now();
    pub_command.publish(flightCommand);
  }

} 
//======================================================================
void CorridorExtraction::publish_plane_marker()
{
  // clear old marker 
  visualization_msgs::Marker del_marker;
  del_marker.header.frame_id = "/kinect_depth";
  del_marker.header.stamp = ros::Time::now();
  del_marker.ns = "corridor_extraction_planes";
  del_marker.type = visualization_msgs::Marker::LINE_LIST;
  del_marker.action = visualization_msgs::Marker::DELETE;
  for(int i=0; i<max_n_planes; ++i)
  {
    del_marker.id = i;
    pub_wall_marker.publish(del_marker);
  }
  
  // create new marker
  visualization_msgs::Marker marker;
  
  // init marker
  marker.header.frame_id = "/kinect_depth";
  marker.header.stamp = ros::Time::now();
  marker.ns = "corridor_extraction_planes";
  
  // publish marker for each plane
  for(unsigned int i=0; i<plane_coefficients.size(); ++i)
  {
    // fill the marker with a visualization_msgs::Marker::TRIANGLE_LIST that represents the convex hull of the projected plane_cloud on the plane defined by the plane_coefficients
    createMarkerForConvexHullOfPlane(plane_cloud[i], plane_coefficients[i], marker);
    marker.id = i; 
    
    // Set the color 
    switch(plane_type[i])
    {
      case floor:
        marker.color.r = 0.0f; marker.color.g = 1.0f;  marker.color.b = 0.0f; marker.color.a = 0.5;break;
      case ceiling:
        marker.color.r = 0.0f; marker.color.g = 0.0f;  marker.color.b = 1.0f; marker.color.a = 0.5;break;
      case left:
        marker.color.r = 1.0f; marker.color.g = 0.0f;  marker.color.b = 0.0f; marker.color.a = 0.5;break;
      case right:
        marker.color.r = 0.5f; marker.color.g = 0.5f;  marker.color.b = 0.0f; marker.color.a = 0.5;break;
      case front:
        marker.color.r = 1.0f; marker.color.g = 0.0f;  marker.color.b = 1.0f; marker.color.a = 0.5;break;
      default:
        marker.color.r = 1.0f; marker.color.g = 1.0f;  marker.color.b = 1.0f; marker.color.a = 0.5;break;
    }
    pub_wall_marker.publish(marker);
  }
}

// ===================================================================
void CorridorExtraction::publish_command_marker()
{
  // publish arrow for raw command (green) and if it's different the command after obstacle avoidance (red)
  
  // create new marker
  visualization_msgs::Marker marker;
  
  // init marker
  marker.header.frame_id = "/kinect_depth";
  marker.header.stamp = ros::Time::now();
  marker.ns = "flightCommand_marker";
  marker.action = visualization_msgs::Marker::ADD;
  
  // raw command
  marker.id = 0; 
  marker.points.clear();
  geometry_msgs::Point startPoint; startPoint.x=0; startPoint.y=0; startPoint.z=0;
  geometry_msgs::Point endPoint; endPoint.x=flightCommand.dy_raw; endPoint.y=flightCommand.dz_raw; endPoint.z=flightCommand.dx_raw;
//   cout << "raw: " << flightCommand.dx_raw << " "<< flightCommand.dy_raw << " "<< flightCommand.dz_raw << endl;
//   cout << "VOA: " << flightCommand.dx << " "<< flightCommand.dy << " "<< flightCommand.dz << endl;
  marker.points.push_back( startPoint );
  marker.points.push_back( endPoint );
  
  marker.scale.x = 0.1;
  marker.scale.y = 0.2;
  
  marker.color.r = 0.0f; marker.color.g = 1.0f;  marker.color.b = 0.0f; marker.color.a = 1;
  
  pub_command_marker.publish(marker);
  
  // obstacle avoidance command
  if( abs(flightCommand.dx-flightCommand.dx_raw)>0 || abs(flightCommand.dy-flightCommand.dy_raw)>0 || abs(flightCommand.dz-flightCommand.dz_raw)>0 )
  {
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 1; 
    marker.points.clear();
    startPoint.x=0; startPoint.y=0; startPoint.z=0;
    endPoint.x=flightCommand.dy; endPoint.y=flightCommand.dz; endPoint.z=flightCommand.dx;
    
    marker.points.push_back( startPoint );
    marker.points.push_back( endPoint );
    
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    
    marker.color.r = 1.0f; marker.color.g = 0.0f;  marker.color.b = 0.0f; marker.color.a = 1;
    
    pub_command_marker.publish(marker);
  }
  else
  {
    // delete old marker
    marker.id = 1; 
    marker.action = visualization_msgs::Marker::DELETE;
    pub_command_marker.publish(marker);
  }
  
  

}

// ===================================================================
int CorridorExtraction::extractPlanes(pcl::PointCloud<pcl::PointXYZ>  &cloud)
{

  plane_cloud.clear();
  plane_coefficients.clear();
  plane_orientation.clear();
  plane_distance.clear();
  plane_type.clear();

  // fill default values
  kinectPose.distance_left=-1;    kinectPose.distance_right=-1;
  kinectPose.distance_floor=-1;   kinectPose.distance_ceiling=-1;
  kinectPose.distance_front=-1;  
  
  weight_left=0; weight_right=0; weight_ceiling=0; weight_floor=0; weight_front=0; 
  
  kinectPose.roll=0; kinectPose.pitch=0; kinectPose.yaw=0;


  // prepare plane segmentation
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.05);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i=0;
  int nPoints=cloud.points.size();


  // While 10% of the original cloud is still there and no more than max_n_planes planes have been extracted
  while ((i<max_n_planes) && (cloud.points.size () > 0.1 * nPoints)){

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud.makeShared());
    seg.segment (inliers, coefficients);

    if (inliers.indices.size () == 0 ) {           
      return plane_coefficients.size();
    }

    // save plane coefficients
    plane_coefficients.push_back(coefficients);

    // insert a new, yet empty point cloud into our vector of planes
    plane_cloud.push_back(pcl::PointCloud<pcl::PointXYZ>());

    // Extract the inliners of the found plane
    extract.setInputCloud (cloud.makeShared());
    extract.setIndices (boost::make_shared<pcl::PointIndices> (inliers));
    extract.setNegative (false);
    extract.filter (plane_cloud[i]);
   
    // remove the inliers from the filtered cloud
    extract.setNegative(true);
    extract.filter(cloud);

    i++;
  } 

  return plane_coefficients.size();
}
// ===================================================================

void CorridorExtraction::extractPlaneOrientations() 
{
  for (unsigned int i=0; i<plane_cloud.size(); i++) {

    // divide the model parameters by the last parameter, so that d=1
    for (int j=0; j<4; j++) plane_coefficients[i].values[j]/=plane_coefficients[i].values[3];

    // calculate the three orientation angles of the extracted planes in the XY, XZ and YZ plane    
    double x=plane_coefficients[i].values[0];
    double y=plane_coefficients[i].values[1];
    double z=plane_coefficients[i].values[2];

    Euler a;
    a.roll=atan2(y,x);  // XY
    a.yaw=atan2(z,x);   // XZ
    a.pitch=atan2(y,z);   // YZ
    if(isnan(a.yaw)) a.yaw=0;
    
    // save orientation angles   
    plane_orientation.push_back(a);               
  } 
}

// ===================================================================
void CorridorExtraction::extractPlaneTypes()
{
  for (unsigned int i=0; i<plane_coefficients.size(); i++) {

    // divide the model parameters by the last parameter, so that d=1
    for (int j=0; j<4; j++) plane_coefficients[i].values[j]/=plane_coefficients[i].values[3];
    
    WallType t=unknown;
    
    // find the largest component of the plane's normal vector
    double x=plane_coefficients[i].values[0];
    double y=plane_coefficients[i].values[1];
    double z=plane_coefficients[i].values[2];

    double x_abs=fabs(x);
    double y_abs=fabs(y);
    double z_abs=fabs(z);

    // is it x? then it is either a left or a right wall
    if ( (x_abs>y_abs) && (x_abs>z_abs) ) {
      if (x>0) t=left;
      else t=right;
    }
    // is it y? then it is either the ceiling or the floor
    else if ( (y_abs>x_abs) && (y_abs>z_abs) ) {
      if (y>0) t=ceiling;
      else t=floor;
    }
    // is it z? then it is in front of us
    else if ( (z_abs>x_abs) && (z_abs>y_abs) ) {
      if (z<0) t=front;
    }
    
    // save the plane type
    plane_type.push_back(t);

  }

}

// ===================================================================
double CorridorExtraction::extractPlaneDistance(const pcl::ModelCoefficients &coeff)
{
  return extractPlaneDistance(coeff.values[0], coeff.values[1], coeff.values[2], coeff.values[3]);
}

// ===================================================================
double CorridorExtraction::extractPlaneDistance(double a, double b, double c, double d)
{    
  double n=sqrt(a*a + b*b + c*c);
  return fabs(d/n);
}

// ===================================================================
void CorridorExtraction::estimateCameraOrientationAndDistances()
{
  
  // find the largest floor plane, ceiling plane, etc.
  unsigned int planeSizes[6]={0,0,0,0,0,0};
  int largestPlane[6]={-1,-1,-1,-1,-1,-1};

  for (unsigned int i=0; i<plane_cloud.size(); i++) {
    if (planeSizes[plane_type[i]]<plane_cloud[i].points.size()) {
      planeSizes[plane_type[i]]=plane_cloud[i].points.size();
      largestPlane[plane_type[i]]=i;
    }
  }

  vector<double> pitch_orientation, yaw_orientation, roll_orientation;
  vector<int> pitch_weight, yaw_weight, roll_weight;
  
  
  
  
  // iterate over the largest ceiling, floor, left, right and front wall
  for (int k=0; k<5; k++) 
  {         
    int i=largestPlane[k];
    if (i<0) continue;

    // get the distance to the wall
    double d=extractPlaneDistance(plane_coefficients[i]);
    
    if (k==left) {
      kinectPose.distance_left=d;
      weight_left=plane_cloud[i].points.size();
    }
    else if (k==right) {      
      kinectPose.distance_right=d;
      weight_right=plane_cloud[i].points.size();
    }    
    else if (k==floor) {
      kinectPose.distance_floor=d;
      weight_floor=plane_cloud[i].points.size();
    }
    else if (k==ceiling) {
      kinectPose.distance_ceiling=d;
      weight_ceiling=plane_cloud[i].points.size();
    }
    else if (k==front) {
      kinectPose.distance_front=d;
      weight_front=plane_cloud[i].points.size();
    }
    
    WallType t=WallType(k);
    
    Euler a=plane_orientation[i];
    // save the orientation angles relative to the floor/ceiling and left/right wall for later use   
    if (t==left || t==right) {   
      // for yaw
      if (t==left) yaw_orientation.push_back(a.yaw);
      else {
        if (a.yaw>0) yaw_orientation.push_back(a.yaw-M_PI);
        else yaw_orientation.push_back(a.yaw+M_PI);	
      }     
      yaw_weight.push_back(plane_cloud[i].points.size());
      
      
      // for roll
      if (t==left)roll_orientation.push_back(a.roll);
      else {
        if (a.roll>0) roll_orientation.push_back(a.roll-M_PI);
        else roll_orientation.push_back(a.roll+M_PI);	
      }
      roll_weight.push_back(plane_cloud[i].points.size());
    }

    else if (t==floor || t==ceiling)  {
      // for pitch
      if (t==ceiling) pitch_orientation.push_back(a.pitch-M_PI_2);
      else pitch_orientation.push_back(a.pitch+M_PI_2);
      pitch_weight.push_back(plane_cloud[i].points.size());
      
      // for roll
      if (t==ceiling) roll_orientation.push_back(a.roll-M_PI_2);
      else roll_orientation.push_back(a.roll+M_PI_2);
      roll_weight.push_back(plane_cloud[i].points.size());
    } 

    else if (t==front) {
      // for pitch
      pitch_orientation.push_back(a.pitch-M_PI);
      pitch_weight.push_back(plane_cloud[i].points.size());
      
      // for yaw
      /*
      if (a.yaw<-M_PI_2) {
        yaw_orientation.push_back(a.yaw+M_PI);
      }
      else {
        yaw_orientation.push_back(a.yaw);
      }
      */

      yaw_orientation.push_back(a.yaw+M_PI_2);
      yaw_weight.push_back(plane_cloud[i].points.size());
    }

    
    // print some info on this plane
    ROS_INFO("Planar component:");
    ROS_DEBUG("\t%d data points.", (int)plane_cloud[i].points.size());
    ROS_INFO("\tModel coefficients: %f %f %f %f", 
    plane_coefficients[i].values[0],
    plane_coefficients[i].values[1],
    plane_coefficients[i].values[2],
    plane_coefficients[i].values[3]);

    string ts;
    if (t==ceiling) ts="ceiling";
    else if (t==floor) ts="floor";
    else if (t==left) ts="left";
    else if (t==right) ts="right";
    else if (t==front) ts="front";    
    else if (t==unknown) ts="unknown";       

    ROS_INFO("\twall type: %s", ts.c_str());
    ROS_INFO("\tdistance to wall: %f", extractPlaneDistance(plane_coefficients[i]));
    ROS_INFO("\tOrientations (pitch, yaw, roll): %f %f %f deg", a.pitch*180/M_PI, a.yaw*180/M_PI, a.roll*180/M_PI);
    
  }


  // retrieve sensor orientation relative to the walls
  float yaw=0, pitch=0, roll=0;
  int sum_weight=0;
  
  for (unsigned int i=0; i<yaw_orientation.size(); i++) {
    yaw+=yaw_weight[i]*yaw_orientation[i];   
    sum_weight+=yaw_weight[i];
  }
  
  if (yaw_orientation.size()>0) yaw=yaw/sum_weight;
  else  yaw=NAN;

  sum_weight=0;
  for (unsigned int i=0; i<pitch_orientation.size(); i++) {
    pitch+=pitch_orientation[i];
    sum_weight+=pitch_weight[i];
  }

  if (pitch_orientation.size()>0) pitch=pitch/sum_weight;
  else pitch=NAN;
  
  sum_weight=0;
  for (unsigned int i=0; i<roll_orientation.size(); i++) {
    roll+=roll_orientation[i];   
    sum_weight+=roll_weight[i];
  }
  if (roll_orientation.size()>0) roll=roll/sum_weight;
  else roll=NAN;

  // save results in our member variable
  kinectPose.roll=roll;
  kinectPose.yaw=yaw;
  kinectPose.pitch=pitch;
  


  // print some information
  ROS_INFO("Orientation:");
  ROS_INFO("\tPitch: %f deg", kinectPose.pitch/M_PI*180);
  ROS_INFO("\tYaw: %f deg", kinectPose.yaw/M_PI*180);    
  ROS_INFO("\tRoll: %f deg", kinectPose.roll/M_PI*180);    

  ROS_INFO("Distances:");
  ROS_INFO("\tleft: %f m", kinectPose.distance_left);
  ROS_INFO("\tright: %f m", kinectPose.distance_right);
  ROS_INFO("\tceiling: %f m", kinectPose.distance_ceiling);
  ROS_INFO("\tfloor: %f m", kinectPose.distance_floor);
  ROS_INFO("\tfront: %f m", kinectPose.distance_front);
 
  
  // append to log file
  ofstream logfile;
  logfile.open ("logfile_pose.txt",fstream::out |fstream::app );
  logfile << ros::Time::now() << " "<<kinectPose.distance_left*1000<<" "<<kinectPose.distance_right*1000<<" "<<kinectPose.distance_ceiling*1000<<" "<<kinectPose.distance_floor*1000<<" "<<kinectPose.distance_front*1000 << " "<< kinectPose.yaw*180/M_PI*100 << " "<< kinectPose.pitch*180/M_PI*100 << " "<< kinectPose.roll*180/M_PI*100 << endl;
  logfile.close();
  
}


// ===================================================================
int main(int argc, char **argv)
{
  ros::init(argc, argv, "corridor_extraction");

  CorridorExtraction ce;

  ros::NodeHandle nh;  

  // subscribe to the topics
  ros::Subscriber subscriber_pointCloud = nh.subscribe("camera/depth/points", 1, &CorridorExtraction::pointCloudCallback, &ce);
  ros::Subscriber subscriber_rangeImage = nh.subscribe("camera/depth/image", 1, &CorridorExtraction::rangeImageCallback, &ce);
    
  ros::spin();

  return 0;
}
