/*
 * CrossFinderSettings.h
 *
 *  Created on: Jun 2, 2013
 *      Author: tony
 */

#ifndef IMAGEPROCESSINGSETTINGS_H_
#define IMAGEPROCESSINGSETTINGS_H_

#include <iostream>

using namespace std;

class CImageProcessingSettings {
public:
	CImageProcessingSettings();
	virtual ~CImageProcessingSettings();

	void Load( const string& filename );
	void Save( const string& filename );

	int SpeedWindowPercent() const
	{
		return speedWindowPercent;
	}

	void SetSpeedWindowPercent( int speedWindowPercent )
	{
		this->speedWindowPercent = speedWindowPercent;
	}

	int VideoInput() const
	{
		return videoInput;
	}

	void SetVideoInput( int videoInput )
	{
		this->videoInput = videoInput;
	}

	int VideoMode() const
	{
		return videoMode;
	}

	void SetVideoMode( int videoMode )
	{
		this->videoMode = videoMode;
	}

	int BlurRadius() const
	{
		return blurRadius;
	}

	void SetBlurRadius( int blurRadius )
	{
		this->blurRadius = blurRadius;
	}

	double BlurSigma() const
	{
		return blurSigma;
	}

	void SetBlurSigma( double blurSigma )
	{
		this->blurSigma = blurSigma;
	}

	int CannyThreshold1() const
	{
		return cannyThreshold1;
	}

	void SetCannyThreshold1( int cannyThreshold1 )
	{
		this->cannyThreshold1 = cannyThreshold1;
	}

	int CannyThreshold2() const
	{
		return cannyThreshold2;
	}

	void SetCannyThreshold2( int cannyThreshold2 )
	{
		this->cannyThreshold2 = cannyThreshold2;
	}

	double HoughAngleRes() const
	{
		return houghAngleRes;
	}

	void SetHoughAngleRes( double houghAngleRes )
	{
		this->houghAngleRes = houghAngleRes;
	}

	double HoughDistRes() const
	{
		return houghDistRes;
	}

	void SetHoughDistRes( double houghDistRes )
	{
		this->houghDistRes = houghDistRes;
	}

	int HoughThreshold() const
	{
		return houghThreshold;
	}

	void SetHoughThreshold( int houghThreshold )
	{
		this->houghThreshold = houghThreshold;
	}

	int ImageWidth() const
	{
		return imageWidth;
	}

	void SetImageWidth( int imageWidth )
	{
		this->imageWidth = imageWidth;
	}

	int MaxParallelDiffAngle() const
	{
		return maxParallelDiffAngle;
	}

	void SetMaxParallelDiffAngle( int maxParallelDiffAngle )
	{
		this->maxParallelDiffAngle = maxParallelDiffAngle;
	}

	int MaxPerpendicularDiffAngle() const
	{
		return maxPerpendicularDiffAngle;
	}

	void SetMaxPerpendicularDiffAngle( int maxPerpendicularDiffAngle )
	{
		this->maxPerpendicularDiffAngle = maxPerpendicularDiffAngle;
	}

	int MaxPerpendicularDiffRadius() const
	{
		return maxPerpendicularDiffRadius;
	}

	void SetMaxPerpendicularDiffRadius( int maxPerpendicularDiffRadius )
	{
		this->maxPerpendicularDiffRadius = maxPerpendicularDiffRadius;
	}

private:
	int imageWidth;
	int blurRadius;
	double blurSigma;
	int cannyThreshold1;
	int cannyThreshold2;
	double houghDistRes;
	double houghAngleRes;
	int houghThreshold;
	int maxParallelDiffAngle;
	int maxPerpendicularDiffAngle;
	int maxPerpendicularDiffRadius;
	int videoMode;
	int videoInput;
	int speedWindowPercent;
};

#endif /* IMAGEPROCESSINGSETTINGS_H_ */
