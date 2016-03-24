#include "Detection.h"


CDetection::CDetection(void)
	: id_(0), camIdx_(0), frameIdx_(0), rect_(cv::Rect2d(0.0, 0.0, 0.0, 0.0)), score_(0.0)
{
}


CDetection::~CDetection(void)
{
}

//()()
//('')HAANJU.YOO

