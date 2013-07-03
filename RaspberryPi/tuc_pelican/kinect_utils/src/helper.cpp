# include <kinect_utils/helper.h>

void createMarkerForConvexHullOfPlane(pcl::PointCloud<pcl::PointXYZ>& plane_cloud, pcl::ModelCoefficients& plane_coefficients, visualization_msgs::Marker& marker)
{
  // init marker
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  // project the points of the plane on the plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (plane_cloud.makeShared());
  proj.setModelCoefficients (boost::make_shared<pcl::ModelCoefficients> (plane_coefficients));
  proj.filter(*cloud_projected);      
  
  // create the convex hull in the plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::ConvexHull<pcl::PointXYZ > chull;
  chull.setInputCloud (cloud_projected);
  chull.reconstruct(*cloud_hull);
  
  // work around known bug in ROS Diamondback perception_pcl: convex hull is centered around centroid of input cloud (fixed in pcl svn revision 443)
  // thus: we shift the mean of cloud_hull to the mean of cloud_projected (fill dx, dy, dz and apply when creating the marker points)
  Eigen::Vector4f meanPointCH, meanPointCP;
  pcl::compute3DCentroid(*cloud_projected, meanPointCP);
  pcl::compute3DCentroid(*cloud_hull, meanPointCH);
  float dx = meanPointCP[0]-meanPointCH[0];
  float dy = meanPointCP[1]-meanPointCH[1];
  float dz = meanPointCP[2]-meanPointCH[2];
       
  // create colored part of plane by creating marker for each triangle between neighbored points on contour of convex hull an midpoint
  marker.points.clear();
  for (unsigned int j = 0; j < cloud_hull->points.size(); ++j)
  {
    geometry_msgs::Point p;
    
    p.x = cloud_hull->points[j].x+dx; p.y = cloud_hull->points[j].y+dy; p.z = cloud_hull->points[j].z+dz; 
    marker.points.push_back( p );
    
    p.x = cloud_hull->points[(j+1)%cloud_hull->points.size() ].x+dx; p.y = cloud_hull->points[(j+1)%cloud_hull->points.size()].y+dy; p.z = cloud_hull->points[(j+1)%cloud_hull->points.size()].z+dz; 
    marker.points.push_back( p );
    
    p.x = meanPointCP[0]; p.y = meanPointCP[1]; p.z = meanPointCP[2]; 
    marker.points.push_back( p );
   
  }
  
  // scale of the marker
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;

}
