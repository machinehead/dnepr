/*
 * CrossFinder.h
 *
 *  Created on: Jun 2, 2013
 *      Author: tony
 */

#ifndef CROSSFINDER_H_
#define CROSSFINDER_H_

#include "opencv2/core/core.hpp"
#include "CrossFinderSettings.h"

using namespace cv;

class CCrossFinder {
public:
	CCrossFinder();
	virtual ~CCrossFinder();

	void Process( const Mat& image );

private:
	CCrossFinderSettings settings;

	Mat resizeImage( const Mat& image ) const;
	Mat findEdges( const Mat& image ) const;
	void findLines( const Mat& edges, vector<Vec2f>& lines ) const;
	void findParallelLines( const vector<Vec2f>& lines,
			vector<Vec3f>& parallelLines ) const;
	bool findCross( vector<Vec3f>& parallelLines, Vec2f& center,
			float& radius ) const;

	bool isParallelLines( float angle1, float angle2 ) const;
	bool isPerpendicularLines( float angle1, float angle2 ) const;
	float middleAngle( float angle1, float angle2 ) const;

	void drawLines( Mat& image, const vector<Vec2f>& lines ) const;
	void drawParallelLines( Mat& image,
			const vector<Vec3f>& parallelLines ) const;
	void drawCircle( Mat& image, const Vec2f& center, float radius ) const;
};

#endif /* CROSSFINDER_H_ */
