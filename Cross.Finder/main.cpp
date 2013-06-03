/*
 * main.cpp
 *
 *  Created on: Jun 2, 2013
 *      Author: tony
 */

#include <iostream>
#include <cstring>
#include <cstdlib>
#include "VideoGrabber.h"

using namespace std;

static void printCommandLineParams()
{
    cout << "-mode     image mode: resolution and fps, supported three values (0 by default):" << endl;
    cout << "          0 - CV_CAP_OPENNI_VGA_30HZ" << endl;
    cout << "          1 - CV_CAP_OPENNI_SXGA_15HZ" << endl;
    cout << "          2 - CV_CAP_OPENNI_SXGA_30HZ" << endl;
    cout << "          Ignored if rgb image or gray image are not selected to show." << endl;
    cout << "-file     Filename of .avi video file. The grey image will grabbed from it." << endl ;
}

static bool parseCommandLine(
	int argc, char* argv[], int& imageMode, string& filename )
{
    // set defaut values
    imageMode = 0;
    filename.clear();

    if( argc == 1 ) {
    	printCommandLineParams();
    } else {
        for( int i = 1; i < argc; i++ ) {
            if( !strcmp( argv[i], "--help" ) || !strcmp( argv[i], "-h" ) ) {
                printCommandLineParams();
                return false;
            } else if( !strcmp( argv[i], "-mode" ) ) {
                imageMode = atoi(argv[++i]);
            } else if( !strcmp( argv[i], "-file" ) ) {
                filename = argv[++i];
            } else {
                cout << "Unsupported command line argument: " << argv[i] << "." << endl;
                return false;
            }
        }
    }

    return true;
}

int main( int argc, char* argv[] )
{
    int imageMode;
    string filename;
    const bool parsingOk = parseCommandLine(
    	argc, argv, imageMode, filename );

    if( !parsingOk ) {
    	return 0;
    }

    CVideoGrabber grabber( imageMode, filename );
    grabber.Process();

    return 0;
}





