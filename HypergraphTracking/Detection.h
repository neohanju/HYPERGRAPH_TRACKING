#pragma once

#include "defines.h"
#include "opencv2\opencv.hpp"

class CDetection
{
	//////////////////////////////////////////////////////////////////////////
	// METHODS
	//////////////////////////////////////////////////////////////////////////
public:
	CDetection(void);
	~CDetection(void);

	//////////////////////////////////////////////////////////////////////////
	// VARIABLES
	//////////////////////////////////////////////////////////////////////////
public:
	int id_;
	int camIdx_;
	int frameIdx_;
	
	cv::Rect2d  rect_;
	cv::Rect2d  partRects_[NUM_DPM_PARTS];
	cv::Point2d bottomCenter_;
	cv::Point3d location3D_;
	double      score_;
private:

};

typedef std::vector<CDetection*> DetectionSet;

//()()
//('')HAANJU.YOO

