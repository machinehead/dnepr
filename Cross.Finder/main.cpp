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
    cout << "-file     Filename of .avi video file. The grey image will grabbed from it." << endl ;
}

static bool parseCommandLine(
	int argc, char* argv[], string& filename )
{
    // set defaut values
    filename.clear();

    if( argc == 1 ) {
    	printCommandLineParams();
    } else {
        for( int i = 1; i < argc; i++ ) {
            if( !strcmp( argv[i], "--help" ) || !strcmp( argv[i], "-h" ) ) {
                printCommandLineParams();
                return false;
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
    string filename;
    const bool parsingOk = parseCommandLine( argc, argv, filename );

    if( !parsingOk ) {
    	return 0;
    }

    CVideoGrabber grabber( filename );
    grabber.Process();

    return 0;
}





