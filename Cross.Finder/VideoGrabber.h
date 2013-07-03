/*
 * VideoGrabber.h
 *
 *  Created on: Jun 2, 2013
 *      Author: tony
 */

#ifndef VIDEOGRABBER_H_
#define VIDEOGRABBER_H_

#include <iostream>
#include "opencv2/core/core.hpp"
#include "ImageProcessingSettings.h"

using namespace std;
using namespace cv;

class CVideoGrabber {
public:
	CVideoGrabber( const string& filename );
	virtual ~CVideoGrabber();

	void Process();

private:
    string filename;
    CImageProcessingSettings settings;

	Mat resizeImage( const Mat& image ) const;
};

#endif /* VIDEOGRABBER_H_ */
