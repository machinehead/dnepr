/*
 * HoughTransform.h
 *
 *  Created on: Jun 5, 2013
 *      Author: tony
 */

#ifndef HOUGHTRANSFORM_H_
#define HOUGHTRANSFORM_H_

#include "opencv2/core/core.hpp"
#include "ImageProcessingSettings.h"

using namespace cv;

class CHoughTransform {
public:
	CHoughTransform( const CImageProcessingSettings& settings );
	virtual ~CHoughTransform();

	void Process( const Mat& edges );

	void GetLines( vector<Vec2f>& _lines ) const
	{
		_lines = lines;
	}

private:
	CImageProcessingSettings settings;

	double maxRadius;
	Mat houghData;
	vector<Vec2f> lines;

	int anglesCount;
	vector<float> sinCache;
	vector<float> cosCache;

	void fillAnglesCache();
	void fillHoughMatrix( const Mat& edges );
	void findLines();
};

#endif /* HOUGHTRANSFORM_H_ */
