/*
 * CrossFinderSettings.cpp
 *
 *  Created on: Jun 2, 2013
 *      Author: tony
 */

#include "ImageProcessingSettings.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

CImageProcessingSettings::CImageProcessingSettings()
{
}

CImageProcessingSettings::~CImageProcessingSettings()
{
}

void CImageProcessingSettings::Load( const string& filename )
{
	using boost::property_tree::ptree;
	ptree pt;

	if( access( filename.c_str(), F_OK ) != -1 ) {
		read_json(filename, pt);
	}

	blurRadius = pt.get( "Cross.Canny.Blur.Radius", 7 );
	blurSigma = pt.get( "Cross.Canny.Blur.Sigma", 3.0 );
	cannyThreshold1 = pt.get( "Cross.Canny.Threshold1", 80 );
	cannyThreshold2 = pt.get( "Cross.Canny.Threshold2", 30 );
	houghDistRes = pt.get( "Cross.Hough.DistanceResolution", 1.0 );
	houghAngleRes = pt.get( "Cross.Hough.AngleResolution", 1.0 );
	houghThreshold = pt.get( "Cross.Hough.Threshold", 100 );
	maxParallelDiffAngle = pt.get( "Cross.Parallel.MaxAngleDiff", 15 );
	maxPerpendicularDiffAngle = pt.get( "Cross.Intersection.MaxAngleDiff", 15 );
	maxPerpendicularDiffRadius = pt.get( "Cross.Intersection.MaxRadiusDiff", 10 );
	videoMode = pt.get( "Video.Mode", 0 );
	videoInput = pt.get( "Video.Input", 1 );
	imageWidth = pt.get( "Video.Width", 320 );
	speedWindowPercent = pt.get( "Speed.WindowPercent", 50 );
}

void CImageProcessingSettings::Save( const string& filename )
{
	using boost::property_tree::ptree;
	ptree pt;

	pt.put( "Cross.Canny.Blur.Radius", blurRadius );
	pt.put( "Cross.Canny.Blur.Sigma", blurSigma );
	pt.put( "Cross.Canny.Threshold1", cannyThreshold1 );
	pt.put( "Cross.Canny.Threshold2", cannyThreshold2 );
	pt.put( "Cross.Hough.DistanceResolution", houghDistRes );
	pt.put( "Cross.Hough.AngleResolution", houghAngleRes );
	pt.put( "Cross.Hough.Threshold", houghThreshold );
	pt.put( "Cross.Parallel.MaxAngleDiff", maxParallelDiffAngle );
	pt.put( "Cross.Intersection.MaxAngleDiff", maxPerpendicularDiffAngle );
	pt.put( "Cross.Intersection.MaxRadiusDiff", maxPerpendicularDiffRadius );
	pt.put( "Video.Mode", videoMode );
	pt.put( "Video.Input", videoInput );
	pt.put( "Video.Width", imageWidth );
	pt.put( "Speed.WindowPercent", speedWindowPercent );

	write_json(filename, pt);
}

