/*
 * VideoGrabber.cpp
 *
 *  Created on: Jun 2, 2013
 *      Author: tony
 */

#include "VideoGrabber.h"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

CVideoGrabber::CVideoGrabber(
	int  _imageMode, const string& _filename ) :
	imageMode( _imageMode ),
	filename( _filename )
{
}

CVideoGrabber::~CVideoGrabber()
{
}

void CVideoGrabber::Process()
{
    cout << "Device opening ..." << endl;
    VideoCapture capture;
    if( !filename.empty() )
        capture.open( filename );
    else
        capture.open(1);

    cout << "done." << endl;

    if( !capture.isOpened() ) {
        cout << "Can not open a capture object." << endl;
        return;
    }

    if( filename.empty()) {
        bool modeRes=false;
        switch ( imageMode ) {
        case 0:
            modeRes = capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );
            break;
        case 1:
            modeRes = capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_SXGA_15HZ );
            break;
        case 2:
            modeRes = capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_SXGA_30HZ );
            break;
            //The following modes are only supported by the Xtion Pro Live
        case 3:
            modeRes = capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_QVGA_30HZ );
            break;
        case 4:
            modeRes = capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_QVGA_60HZ );
            break;
        default:
            CV_Error( CV_StsBadArg, "Unsupported image mode property.\n");
            break;
        }
        if (!modeRes)
            cout << "\nThis image mode is not supported by the device, the default value (CV_CAP_OPENNI_SXGA_15HZ) will be used.\n" << endl;
    }

    // Print some avalible device settings.
    cout << "\nDepth generator output mode:" << endl <<
         "FRAME_WIDTH      " << capture.get( CV_CAP_PROP_FRAME_WIDTH ) << endl <<
         "FRAME_HEIGHT     " << capture.get( CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
         "FRAME_MAX_DEPTH  " << capture.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH ) << " mm" << endl <<
         "FPS              " << capture.get( CV_CAP_PROP_FPS ) << endl <<
         "REGISTRATION     " << capture.get( CV_CAP_PROP_OPENNI_REGISTRATION ) << endl;
    if( capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR_PRESENT ) ) {
        cout <<
             "\nImage generator output mode:" << endl <<
             "FRAME_WIDTH   " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH ) << endl <<
             "FRAME_HEIGHT  " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
             "FPS           " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS ) << endl;
    } else {
        cout << "\nDevice doesn't contain image generator." << endl;
    }

    for(;;) {
        Mat image;

        if( !capture.grab() ) {
            cout << "Can not grab images." << endl;
            return;
        } else {
            capture.retrieve( image );

            finder.Process( image );
        }

        if( waitKey( 30 ) >= 0 )
            break;
    }
}

