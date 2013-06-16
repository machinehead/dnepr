/*
 * HoughTransform.cpp
 *
 *  Created on: Jun 5, 2013
 *      Author: tony
 */

#include "HoughTransform.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

CHoughTransform::CHoughTransform( const CImageProcessingSettings& _settings )
{
	settings = _settings;
	fillAnglesCache();
}

CHoughTransform::~CHoughTransform()
{

}

void CHoughTransform::Process( const Mat& edges )
{
	fillHoughMatrix( edges );
	findLines();
}

void CHoughTransform::fillAnglesCache()
{
	anglesCount = round( 180.0 / settings.HoughAngleRes() ) + 1;

	sinCache.assign( anglesCount + 1, 0 );
	cosCache.assign( anglesCount + 1, 0 );

	for( int i = 0; i <= anglesCount; i++ ) {
		const double angle = CV_PI * i * settings.HoughAngleRes() / 180;
		sinCache[i] = sin( angle );
		cosCache[i] = cos( angle );
	}
}

void CHoughTransform::fillHoughMatrix( const Mat& edges )
{
	assert( edges.dims == 2 );

	maxRadius = sqrt( edges.cols * edges.cols + edges.rows * edges.rows );
	const int radCount = 2 * ceil( maxRadius / settings.HoughDistRes() ) + 1;

	houghData = Mat::zeros( anglesCount, radCount, CV_32SC1 );
	for( int i = 0; i < edges.rows; i++ ) {
	    const unsigned char* rowData = edges.ptr<unsigned char>(i);
	    for( int j = 0; j < edges.cols; j++ ) {
	    	if( rowData[j] > 0 ) {
	    		for( int k = 0; k < anglesCount; k++ ) {
	    			const double r1 = j * cosCache[k] + i * sinCache[k] + maxRadius;
	    			const double r2 = j * cosCache[k+1] + i * sinCache[k+1] + maxRadius;

	    			const int rCell1 = round( r1 / settings.HoughDistRes() );
	    			const int rCell2 = round( r2 / settings.HoughDistRes() );

	    			for( int l = min( rCell1, rCell2 ); l <= max( rCell1, rCell2 ); l++ ) {
	    				houghData.at<int>( k, l ) += 1;
	    			}
	    		}
	    	}
	    }
	}
	imshow( "hough", houghData );
}

void CHoughTransform::findLines()
{
	lines.clear();

	for( int i = 1; i < houghData.rows - 1; i++ ) {
		const int* prevRow = houghData.ptr<int>(i - 1);
		const int* rowData = houghData.ptr<int>(i);
	    const int* nextRow = houghData.ptr<int>(i + 1);
	    for( int j = 1; j < houghData.cols - 1; j++ ) {
	    	if( rowData[j] > settings.HoughThreshold() &&
	    		rowData[j] > prevRow[j] &&
	    		rowData[j] > nextRow[j] &&
	    		rowData[j] > rowData[j-1] &&
	    		rowData[j] > nextRow[j+1] )
	    	{
	    		Vec2f line;
	    		line[0] = j * settings.HoughDistRes() - maxRadius;
	    		line[1] = CV_PI * i * settings.HoughAngleRes() / 180;
	    		lines.push_back( line );
	    	}
	    }
	}
}






