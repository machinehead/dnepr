#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <opencv/cv.h>
#pragma once
// ==========================================================
//! Fill marker.points with triangle list that represents the convex hull of points in plane_cloud projected on plane given by plane_coefficients
void createMarkerForConvexHullOfPlane(pcl::PointCloud<pcl::PointXYZ>& plane_cloud, pcl::ModelCoefficients& plane_coefficients, visualization_msgs::Marker& marker);

// ==========================================================
//! print min and max value of IplImage
inline void showMinMax(IplImage *img)
{
  double gMin, gMax;
  for(int i=0; i<img->nChannels; i++)
  {
    cvSetImageCOI(img, i+1);
    double min, max;
    cvMinMaxLoc(img, &min, &max);
    if(i==0)
    {
      gMin=min;
      gMax=max;
    }
    else
    {
      if(min<gMin) gMin = min;
      if(max>gMax) gMax = max;
    }
  }
  cvSetImageCOI(img, 0);
  cout << gMin << " " << gMax << endl;
}

// ==========================================================
/**
* Little helper function.
* Interprets mat as integral image and returns the pixel sum inside the rect(x1,y1,x2,y2).
*/
inline double integralRect(cv::Mat& mat, int x1, int y1, int x2, int y2)
{
  return mat.at<int>(y2,x2) + mat.at<int>(y1,x1) - mat.at<int>(y2,x1) - mat.at<int>(y1,x2);
}

