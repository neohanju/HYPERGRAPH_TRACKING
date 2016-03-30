#pragma once

#include "Detection.h"
#include "cameraModel.h"

class CReconstruction
{
	//////////////////////////////////////////////////////////////////////////
	// METHODS
	//////////////////////////////////////////////////////////////////////////
public:
	CReconstruction(const DetectionSet &detections,
		            const std::vector<Etiseo::CameraModel> &vecCamModels,
					const std::vector<cv::Mat> &vecMatProjectionSensitivity,
					const std::vector<cv::Mat> &vecMatDistanceFromBoundary,
					const double fpRatio, const double fnRatio,
					const double tauEnter, const double tauExit,
					const double minDetectionHeight);
	~CReconstruction(void);

	static double GetTransitionCost(const CReconstruction &currR, const CReconstruction &nextR, const int deltaMax, const double fnRatio);
	static bool IsCompatible(const CReconstruction &recons1, const CReconstruction &recons2);

	//////////////////////////////////////////////////////////////////////////
	// VARIABLES
	//////////////////////////////////////////////////////////////////////////
public:
	bool bValid_;
	int id_;
	int numDetections_;
	int frameIdx_;
	DetectionSet detections_;
	cv::Point3d location3D_;
	double costEnter_;
	double costExit_;
	double costReconstruction_;	
};

typedef std::vector<CReconstruction*> ReconstructionSet;

//()()
//('')HAANJU.YOO

