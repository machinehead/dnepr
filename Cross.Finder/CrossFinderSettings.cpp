/*
 * CrossFinderSettings.cpp
 *
 *  Created on: Jun 2, 2013
 *      Author: tony
 */

#include "CrossFinderSettings.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

CCrossFinderSettings::CCrossFinderSettings()
{
}

CCrossFinderSettings::~CCrossFinderSettings()
{
}

void CCrossFinderSettings::Load( const string& filename )
{
	using boost::property_tree::ptree;
	ptree pt;

	if( access( filename.c_str(), F_OK ) != -1 ) {
		read_json(filename, pt);
	}

	imageWidth = pt.get( "Image.Width", 320 );
	blurRadius = pt.get( "Canny.Blur.Radius", 7 );
	blurSigma = pt.get( "Canny.Blur.Sigma", 3.0 );
	cannyThreshold1 = pt.get( "Canny.Threshold1", 80 );
	cannyThreshold2 = pt.get( "Canny.Threshold2", 30 );
	houghDistRes = pt.get( "Hough.DistanceResolution", 1.0 );
	houghAngleRes = pt.get( "Hough.AngleResolution", 1.0 );
	houghThreshold = pt.get( "Hough.Threshold", 100 );
	maxParallelDiffAngle = pt.get( "Parallel.MaxAngleDiff", 15 );
	maxPerpendicularDiffAngle = pt.get( "Cross.MaxAngleDiff", 15 );
	maxPerpendicularDiffRadius = pt.get( "Cross.MaxRadiusDiff", 10 );
}

void CCrossFinderSettings::Save( const string& filename )
{
	using boost::property_tree::ptree;
	ptree pt;

	pt.put( "Image.Width", imageWidth );
	pt.put( "Canny.Blur.Radius", blurRadius );
	pt.put( "Canny.Blur.Sigma", blurSigma );
	pt.put( "Canny.Threshold1", cannyThreshold1 );
	pt.put( "Canny.Threshold2", cannyThreshold2 );
	pt.put( "Hough.DistanceResolution", houghDistRes );
	pt.put( "Hough.AngleResolution", houghAngleRes );
	pt.put( "Hough.Threshold", houghThreshold );
	pt.put( "Parallel.MaxAngleDiff", maxParallelDiffAngle );
	pt.put( "Cross.MaxAngleDiff", maxPerpendicularDiffAngle );
	pt.put( "Cross.MaxRadiusDiff", maxPerpendicularDiffRadius );


	write_json(filename, pt);
}

