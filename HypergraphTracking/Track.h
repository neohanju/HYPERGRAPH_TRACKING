#pragma once
#include "Reconstruction.h"

class CTrack
{
public:
	CTrack(int nIdx, int nTimeStart);
	~CTrack(void);

	int id_;
	int timeStart_;
	int timeEnd_;
	double cost_;
	ReconstructionSet reconstructions_;	
	std::vector<cv::Point3d> locations_;
	std::vector<std::vector<cv::Rect>> viewRects_;
};

