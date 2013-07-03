/*
 * SpeedFinder.cpp
 *
 *  Created on: Jun 10, 2013
 *      Author: tony
 */

#include "SpeedFinder.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <string>
#include <sstream>

CSpeedFinder::CSpeedFinder( const CImageProcessingSettings& _settings )
{
	settings = _settings;
}

CSpeedFinder::~CSpeedFinder()
{
}

// Возвращает 4 значения скорости в массиве.
// Расположение данных в массиве:
// ( смещение по X, смещение по Y, изменение масштаба, изменение угла )
Vec4d CSpeedFinder::CalculateTransition( const Mat& nextImage )
{
	Mat transMat = Mat();

	if( currImage.empty() ) {
		// Первый запуск, записываем координаты окна, которое будем вырезать из изображения
		const int cornerCoef = ( 100 - settings.SpeedWindowPercent() );
		cropRect = Rect( cornerCoef * nextImage.cols / 200,
				cornerCoef * nextImage.rows / 200,
				settings.SpeedWindowPercent() * nextImage.cols / 100,
				settings.SpeedWindowPercent() * nextImage.rows / 100 );
		currImage = nextImage( cropRect );
		return Vec4d( 1, 0, 0, 0 );
	}

	Mat croppedNext = nextImage( cropRect );
	transMat = estimateRigidTransform( currImage, croppedNext, false );
	currImage = croppedNext;

	Vec4d result( 1, 0, 0, 0 );
	if( !transMat.empty() ) {
		result[0] = sqrt(
				transMat.at<double>( 0, 0 ) * transMat.at<double>( 0, 0 )
						+ transMat.at<double>( 1, 0 )
								* transMat.at<double>( 1, 0 ) );
		result[1] = transMat.at<double>( 0, 2 ) / result[0];
		result[2] = transMat.at<double>( 1, 2 ) / result[0];
		result[3] = fastAtan2( transMat.at<double>( 1, 0 ),
				transMat.at<double>( 0, 0 ) );
	}

	drawTransitionMatrix( transMat, result );
	imshow( "Crop", currImage );

	return result;
}

Point pointAffineTransform( const Point& p, const Mat& transMat )
{
	const float x = p.x * transMat.at<double>( 0, 0 )
			+ p.y * transMat.at<double>( 0, 1 ) + transMat.at<double>( 0, 2 );
	const float y = p.x * transMat.at<double>( 1, 0 )
			+ p.y * transMat.at<double>( 1, 1 ) + transMat.at<double>( 1, 2 );
	return Point( round( x ), round( y ) );
}

void CSpeedFinder::drawTransitionMatrix( const Mat& transMat,
		const Vec4d& result ) const
{
	Mat drawMatrix = Mat::zeros( 300, 200, CV_8UC3 );

	const Point yAxisBegin( 100, 50 );
	const Point yAxisEnd( 100, 150 );
	line( drawMatrix, yAxisBegin, yAxisEnd, Scalar( 255, 255, 255 ), 1, 8 );

	const Point xAxisBegin( 50, 100 );
	const Point xAxisEnd( 150, 100 );
	line( drawMatrix, xAxisBegin, xAxisEnd, Scalar( 255, 255, 255 ), 1, 8 );

	if( !transMat.empty() ) {
		assert( transMat.rows == 2 && transMat.cols == 3 );

		const Point yAxisBeginNew = pointAffineTransform( yAxisBegin,
				transMat );
		const Point yAxisEndNew = pointAffineTransform( yAxisEnd, transMat );
		line( drawMatrix, yAxisBeginNew, yAxisEndNew, Scalar( 255, 0, 0 ), 3,
				8 );

		const Point xAxisBeginNew = pointAffineTransform( xAxisBegin,
				transMat );
		const Point xAxisEndNew = pointAffineTransform( xAxisEnd, transMat );
		line( drawMatrix, xAxisBeginNew, xAxisEndNew, Scalar( 255, 0, 0 ), 3,
				8 );
	}

	ostringstream os;
	os << "Scale: " << result[0];
	putText( drawMatrix, os.str(), Point( 0, 220 ), FONT_HERSHEY_PLAIN, 1,
			Scalar( 0, 255, 0 ) );

	os.str( "" );
	os.clear();
	os << "Vx: " << result[1];
	putText( drawMatrix, os.str(), Point( 0, 240 ), FONT_HERSHEY_PLAIN, 1,
			Scalar( 0, 255, 0 ) );

	os.str( "" );
	os.clear();
	os << "Vy: " << result[2];
	putText( drawMatrix, os.str(), Point( 0, 260 ), FONT_HERSHEY_PLAIN, 1,
			Scalar( 0, 255, 0 ) );

	os.str( "" );
	os.clear();
	os << "AngleDiff: " << result[3];
	putText( drawMatrix, os.str(), Point( 0, 280 ), FONT_HERSHEY_PLAIN, 1,
			Scalar( 0, 255, 0 ) );

	imshow( "Transform", drawMatrix );
}

