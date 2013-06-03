/*
 * CrossFinder.cpp
 *
 *  Created on: Jun 2, 2013
 *      Author: tony
 */

#include "CrossFinder.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

CCrossFinder::CCrossFinder()
{
	settings.Load( "CrossFinderSettings.json" );
}

CCrossFinder::~CCrossFinder()
{
	settings.Save( "CrossFinderSettings.json" );
}

void CCrossFinder::Process( const Mat& image )
{
	Mat smallImage = resizeImage( image );
	imshow( "gray", smallImage );

	Mat edges = findEdges( smallImage );
	imshow( "edges", edges );

	vector<Vec2f> lines;
	findLines( edges, lines );

	vector<Vec3f> parallelLines;
	findParallelLines( lines, parallelLines );

	Vec2f center;
	float radius;
	bool crossFound = findCross( parallelLines, center, radius );

	Mat lines_image;
	cvtColor( edges, lines_image, CV_GRAY2BGR );
	drawLines( lines_image, lines );
	drawParallelLines( lines_image, parallelLines );
	if( crossFound ) {
		drawCircle( lines_image, center, radius );
	}
	imshow( "Detected Lines", lines_image );
}

Mat CCrossFinder::resizeImage( const Mat& image ) const
{
	Mat smallImage;
	cvtColor( image, smallImage, CV_BGR2GRAY );
	Size imageSize = smallImage.size();
	if( imageSize.width != settings.ImageWidth() ) {
		double scale = 1.0 * settings.ImageWidth() / imageSize.width;
		resize( smallImage, smallImage, Size( 0, 0 ), scale, scale,
				INTER_NEAREST );
	}
	return smallImage;
}

Mat CCrossFinder::findEdges( const Mat& image ) const
{
	Mat edges;
	GaussianBlur( image, edges,
			Size( settings.BlurRadius(), settings.BlurRadius() ),
			settings.BlurSigma(), settings.BlurSigma() );
	Canny( edges, edges, settings.CannyThreshold1(),
			settings.CannyThreshold2() );
	return edges;
}

void CCrossFinder::findLines( const Mat& edges, vector<Vec2f>& lines ) const
{
	HoughLines( edges, lines, settings.HoughDistRes(),
			settings.HoughAngleRes() * CV_PI / 180, settings.HoughThreshold() );
}

void CCrossFinder::findParallelLines( const vector<Vec2f>& lines,
		vector<Vec3f>& parallelLines ) const
{
	vector<bool> pairedLines;
	pairedLines.resize( lines.size(), false );
	for( size_t i = 0; i + 1 < lines.size(); i++ ) {
		if( pairedLines[i] ) {
			continue;
		}
		float minDistance;
		int foundPair = -1;
		for( size_t j = i + 1; j < lines.size(); j++ ) {
			if( pairedLines[j] ) {
				continue;
			}
			if( isParallelLines( lines[i][1], lines[j][1] ) ) {
				float distance = abs( lines[i][0] - lines[j][0] ) / 2;
				if( foundPair == -1 || distance < minDistance ) {
					minDistance = distance;
					foundPair = j;

				}
			}
		}
		if( foundPair != -1 ) {
			Vec3f parLine;
			parLine[0] = ( lines[i][0] + lines[foundPair][0] ) / 2;
			parLine[1] = middleAngle( lines[i][1], lines[foundPair][1] );
			parLine[2] = minDistance;
			parallelLines.push_back( parLine );
			pairedLines[i] = true;
			pairedLines[foundPair] = true;
		}
	}
}

bool CCrossFinder::findCross( vector<Vec3f>& parallelLines, Vec2f& center,
		float& radius ) const
{
	float minDiff = 0;
	int firstLine = -1;
	int secondLine = -1;

	for( size_t i = 0; i + 1 < parallelLines.size(); i++ ) {
		for( size_t j = i + 1; j < parallelLines.size(); j++ ) {
			if( isPerpendicularLines( parallelLines[i][1], parallelLines[j][1] ) ) {
				float radDiff = abs( parallelLines[i][2] - parallelLines[j][2] );
				if( firstLine == -1 || radDiff < minDiff ) {
					minDiff = radDiff;
					firstLine = i;
					secondLine = j;
				}
			}
		}
	}
	if( firstLine != -1 && minDiff < settings.MaxPerpendicularDiffRadius() ) {
		radius = ( parallelLines[firstLine][2] + parallelLines[secondLine][2] ) / 2;

		const float r1 = parallelLines[firstLine][0];
		const float r2 = parallelLines[secondLine][0];
		const float a1 = parallelLines[firstLine][1];
		const float a2 = parallelLines[secondLine][1];

		const float sin12 = sin( a1 - a2 );

		center[0] = ( r2 * sin( a1 ) - r1 * sin( a2 ) ) / sin12;
		center[0] = ( r1 * cos( a2 ) - r2 * cos( a1 ) ) / sin12;
		return true;
	}

	return false;
}

bool CCrossFinder::isParallelLines( float angle1, float angle2 ) const
{
	if( angle1 < CV_PI / 4 && angle2 > 3 * CV_PI / 4 ) {
		angle2 -= CV_PI;
	} else if( angle2 < CV_PI / 4 && angle1 > 3 * CV_PI / 4 ) {
		angle1 -= CV_PI;
	}
	const float diff = abs( angle1 - angle2 );
	if( 180 * diff > settings.MaxParallelDiffAngle() * CV_PI ) {
		return false;
	}

	return true;
}

bool CCrossFinder::isPerpendicularLines( float angle1, float angle2 ) const
{
	const float diff = abs( angle1 - angle2 );
	const float rightDiff = abs( CV_PI / 2 - diff );
	return rightDiff < settings.MaxPerpendicularDiffAngle();
}

float CCrossFinder::middleAngle( float angle1, float angle2 ) const
{
	if( angle1 < CV_PI / 4 && angle2 > 3 * CV_PI / 4 ) {
		angle2 -= CV_PI;
	} else if( angle2 < CV_PI / 4 && angle1 > 3 * CV_PI / 4 ) {
		angle1 -= CV_PI;
	}

	return ( angle1 + angle2 ) / 2;
}

void CCrossFinder::drawLines( Mat& image, const vector<Vec2f>& lines ) const
{
	for( size_t i = 0; i < lines.size(); i++ ) {
		float rho = lines[i][0];
		float theta = lines[i][1];
		double a = cos( theta ), b = sin( theta );
		double x0 = a * rho, y0 = b * rho;
		const Point begin( cvRound( x0 + 1000 * ( -b ) ),
				cvRound( y0 + 1000 * ( a ) ) );
		const Point end( cvRound( x0 - 1000 * ( -b ) ),
				cvRound( y0 - 1000 * ( a ) ) );

		line( image, begin, end, Scalar( 255, 0, 0 ), 2, 8 );
	}
}

void CCrossFinder::drawParallelLines( Mat& image,
		const vector<Vec3f>& parallelLines ) const
{
	for( size_t i = 0; i < parallelLines.size(); i++ ) {
		float rho = parallelLines[i][0];
		float theta = parallelLines[i][1];
		float radius = parallelLines[i][2];
		double a = cos( theta ), b = sin( theta );
		{
			double x0 = a * rho, y0 = b * rho;
			const Point begin( cvRound( x0 + 1000 * ( -b ) ),
					cvRound( y0 + 1000 * ( a ) ) );
			const Point end( cvRound( x0 - 1000 * ( -b ) ),
					cvRound( y0 - 1000 * ( a ) ) );

			line( image, begin, end, Scalar( 0, 255, 0 ), 3, 8 );
		}
		{
			double x0 = a * ( rho + radius ), y0 = b * ( rho + radius );
			const Point begin( cvRound( x0 + 1000 * ( -b ) ),
					cvRound( y0 + 1000 * ( a ) ) );
			const Point end( cvRound( x0 - 1000 * ( -b ) ),
					cvRound( y0 - 1000 * ( a ) ) );

			line( image, begin, end, Scalar( 0, 0, 255 ), 2, 8 );
		}
		{
			double x0 = a * ( rho - radius ), y0 = b * ( rho - radius );
			const Point begin( cvRound( x0 + 1000 * ( -b ) ),
					cvRound( y0 + 1000 * ( a ) ) );
			const Point end( cvRound( x0 - 1000 * ( -b ) ),
					cvRound( y0 - 1000 * ( a ) ) );

			line( image, begin, end, Scalar( 0, 0, 255 ), 2, 8 );
		}
	}
}

void CCrossFinder::drawCircle( Mat& image, const Vec2f& center,
		float radius ) const
{
	Point intCenter( round( center[0] ), round( center[1] ) );
	circle( image, intCenter, round( radius ), Scalar( 0, 0, 255 ), 2, 8 );
}

