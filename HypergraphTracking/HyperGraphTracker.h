#pragma once

#include "Detection.h"
#include "Setting.h"
#include "cameraModel.h"

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

private:

	//////////////////////////////////////////////////////////////////////////
	// VARIABLES
	//////////////////////////////////////////////////////////////////////////
private:
	bool bInit_;
	CSetting SET_;
	std::vector<std::vector<DetectionSet>> vecvecDetectionSets_; // frame / cam / detection
	std::vector<Etiseo::CameraModel> vecCamModels_;

	int numDetections_;
};

//()()
//('')HAANJU.YOO

