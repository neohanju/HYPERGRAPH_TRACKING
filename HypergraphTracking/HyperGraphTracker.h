#pragma once

#include "Detection.h"
#include "Reconstruction.h"
#include "Setting.h"

#include <vector>
#include <list>

class CHyperGraphTracker
{
	//////////////////////////////////////////////////////////////////////////
	// METHODS
	//////////////////////////////////////////////////////////////////////////
public:
	CHyperGraphTracker(void);
	~CHyperGraphTracker(void);

	bool Initialize(const CSetting &SET);
	bool Finalize(void);
	bool ConstructHyperGraph(void);

	bool LoadGraph(const std::string strGraphPath);
	bool LoadDetections(void);
	bool GenerateReconstructions(void);
	

private:

	//////////////////////////////////////////////////////////////////////////
	// VARIABLES
	//////////////////////////////////////////////////////////////////////////
private:
	bool bInit_;
	CSetting SET_;	
	std::vector<Etiseo::CameraModel> vecCamModels_;
	std::vector<cv::Mat> vecMatProjectionSensitivity_;
	std::vector<cv::Mat> vecMatDistanceFromBoundary_;
	std::list<CDetection> listDetections_;
	std::list<CReconstruction> listReconstructions_;
	std::vector<std::vector<DetectionSet>> vecvecPtDetectionSets_;      // frame / cam / detection
	std::vector<std::vector<CReconstruction*>> vecvecPtReconstructions_; // frame / reconstruction

	int numDetections_;
	double minDetectionHeight_;
};

//()()
//('')HAANJU.YOO


