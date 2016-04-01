#pragma once

#include "Track.h"
#include "Setting.h"

#include <vector>
#include <list>

class GRBEnv;
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
	bool Run(void);
	bool SaveTrackingResult(const std::string strFilePath);
	void Visualization(void);

private:
	bool ConstructGraphAndSolving(void);
	bool LoadGraph(const std::string strGraphPath);
	bool LoadDetections(void);
	void GenerateReconstructions(void);
	void GenerateDetectionCombinations(DetectionSet curCombination, 
		                               const int curCamIdx, 
									   const std::vector<DetectionSet> &entireDetectionsAtEachCame,
									   std::deque<DetectionSet> &outputCombinationQueue);

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
	std::vector<ReconstructionSet> vecvecPtReconstructions_; // frame / reconstruction	
	ReconstructionSet vecPtReconstructions_; // for fast access at the end of graph solving
	std::deque<CTrack> queueTracks_;
	std::vector<std::deque<std::pair<int, cv::Rect>>> vecQueueRectsOnTime_;

	int numDetections_;
	int numReconstructions_;
	double minDetectionHeight_;

	// optimization related
	GRBEnv *pGRBEnv_;
	double timeProblemConstruction_;
	double timeProblemSolving_;
};

//()()
//('')HAANJU.YOO


