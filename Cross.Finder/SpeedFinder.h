/*
 * SpeedFinder.h
 *
 *  Created on: Jun 10, 2013
 *      Author: tony
 */

#ifndef SPEEDFINDER_H_
#define SPEEDFINDER_H_

#include "opencv2/core/core.hpp"
#include "opencv2/video/tracking.hpp"
#include "ImageProcessingSettings.h"

using namespace cv;
using namespace std;

class CSpeedFinder {
public:
	CSpeedFinder( const CImageProcessingSettings& settings );
	virtual ~CSpeedFinder();

	// Возвращает 4 значения скорости в массиве.
	// Расположение данных в массиве:
	// ( изменение масштаба, смещение по X, смещение по Y, изменение угла )
	Vec4d CalculateTransition( const Mat& next );

private:
	CImageProcessingSettings settings;
	Mat currImage;
	Rect cropRect;

	void drawTransitionMatrix( const Mat& transMat, const Vec4d& result ) const;
};

#endif /* SPEEDFINDER_H_ */
