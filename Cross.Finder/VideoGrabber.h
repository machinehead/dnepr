/*
 * VideoGrabber.h
 *
 *  Created on: Jun 2, 2013
 *      Author: tony
 */

#ifndef VIDEOGRABBER_H_
#define VIDEOGRABBER_H_

#include <iostream>
#include "CrossFinder.h"

using namespace std;

class CVideoGrabber {
public:
	CVideoGrabber( int  imageMode, const string& filename );
	virtual ~CVideoGrabber();

	void Process();

private:
    int imageMode;
    string filename;

    CCrossFinder finder;
};

#endif /* VIDEOGRABBER_H_ */
