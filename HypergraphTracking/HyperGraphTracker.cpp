#include "HyperGraphTracker.h"
#include "hjlib.h"

CHyperGraphTracker::CHyperGraphTracker()
	: bInit_(false)
{
}

CHyperGraphTracker::~CHyperGraphTracker(void)
{
}

/************************************************************************
 Method Name: Initialize
 Description: 
	- initialization routine for the tracker
 Input Arguments:
	- SET: class containing setting information
 Return Values:
	- bool variable indicating the proper initialization
************************************************************************/
bool CHyperGraphTracker::Initialize(const CSetting &SET)
{
	if (bInit_) { return false; }
	SET_   = SET;
	bInit_ = true;

	///////////////////////////////////////////////////////////
	// CAMERA MODEL INITIALIZATION
	///////////////////////////////////////////////////////////	
	vecCamModels_.resize(SET_.numCams());
	vecMatProjectionSensitivity_.resize(SET_.numCams());
	vecMatDistanceFromBoundary_.resize(SET_.numCams());
	for (int cIdx = 0; cIdx < SET_.numCams(); cIdx++)
	{
		//----------------------------------------------------
		// READ CALIBRATION INFORMATION
		//----------------------------------------------------
		std::string strCalibrationFilePath = 
			hj::fullfile(SET_.GetCalibrationPath(), hj::sprintf("View_%03d.xml", SET_.GetCamIdx(cIdx)));
		std::cout << "Reading calibration information of camera " << SET_.GetCamIdx(cIdx) << "...";
		if (vecCamModels_[cIdx].fromXml(strCalibrationFilePath))
		{
			std::cout << " done" << std::endl;
			std::cout << " Camera position : (" << vecCamModels_[cIdx].cposx() << ", " 
				                                << vecCamModels_[cIdx].cposy() << ", " 
                                                << vecCamModels_[cIdx].cposz() << ")" << std::endl;
		}
		else
		{
			std::cout << " fail!" << std::endl;
		}

		//----------------------------------------------------
		// READ PROJECTION SENSITIVITY MATRIX
		//----------------------------------------------------
		std::cout << " Read projection sensitivity : ";
		std::string strProjectionMatrix = 
			hj::fullfile(SET_.GetCalibrationPath(), hj::sprintf("ProjectionSensitivity_View%03d.txt", SET_.GetCamIdx(cIdx)));
		if (!vecMatProjectionSensitivity_[cIdx].empty()) { vecMatProjectionSensitivity_[cIdx].release(); };
		vecMatProjectionSensitivity_[cIdx] = hj::ReadMatrix(strProjectionMatrix);
		if (vecMatProjectionSensitivity_[cIdx].empty())
		{
			std::cout << " fail" << std::endl;
		}
		else
		{
			std::cout << " success" << std::endl;
		}

		//----------------------------------------------------
		// READ DISTANCE FROM BOUNDARY MATRIX
		//----------------------------------------------------	
		std::cout << " Read distance from boundary : ";
		std::string strDistanceFromBoundaryMatrix = 
			hj::fullfile(SET_.GetCalibrationPath(), hj::sprintf("DistanceFromBoundary_View%03d.txt", SET_.GetCamIdx(cIdx)));
		if (!vecMatDistanceFromBoundary_[cIdx].empty()) { vecMatDistanceFromBoundary_[cIdx].release(); };
		vecMatDistanceFromBoundary_[cIdx] = hj::ReadMatrix(strDistanceFromBoundaryMatrix);
		if (vecMatDistanceFromBoundary_[cIdx].empty())
		{
			std::cout << " fail" << std::endl;
		}
		else
		{
			std::cout << " success" << std::endl;
		}
	}

	///////////////////////////////////////////////////////////
	// DETECTION INITIALIZATION
	///////////////////////////////////////////////////////////	
	numDetections_ = 0;
	vecvecPtDetectionSets_.resize(SET_.numFrames());
	for (int fIdx = 0; fIdx < SET_.numFrames(); fIdx++)
	{
		vecvecPtDetectionSets_[fIdx].resize(SET_.numCams());
	}

	return true;
}

/************************************************************************
 Method Name: Finalize
 Description: 
	- termination routine for the tracker
 Input Arguments:
	- none
 Return Values:
	- bool variable indicating the proper finalization
************************************************************************/
bool CHyperGraphTracker::Finalize(void)
{
	if (!bInit_) { return true; }

	///////////////////////////////////////////////////////////
	// CAMERA MODEL FINALIZATION
	///////////////////////////////////////////////////////////		
	vecCamModels_.clear();
	for (int cIdx = 0; cIdx < SET_.numCams(); cIdx++)
	{
		if (!vecMatProjectionSensitivity_[cIdx].empty()) { vecMatProjectionSensitivity_[cIdx].release(); }
		if (!vecMatDistanceFromBoundary_[cIdx].empty())  { vecMatDistanceFromBoundary_[cIdx].release(); }
	}	
	vecMatProjectionSensitivity_.clear();
	vecMatDistanceFromBoundary_.clear();

	///////////////////////////////////////////////////////////
	// VECTOR FINALIZATION
	///////////////////////////////////////////////////////////	
	for (int fIdx = 0; fIdx < vecvecPtDetectionSets_.size(); fIdx++)
	{		
		for (int cIdx = 0; cIdx < vecvecPtDetectionSets_[fIdx].size(); cIdx++)
		{
			vecvecPtDetectionSets_[fIdx][cIdx].clear();
		}		
		vecvecPtDetectionSets_[fIdx].clear();
		vecvecPtReconstructions_[fIdx].clear();
	}
	vecvecPtDetectionSets_.clear();
	vecvecPtReconstructions_.clear();

	return true;
}

/************************************************************************
 Method Name: ConstructHyperGraph
 Description: 
	- construct hypergraph for 3D tracking
 Input Arguments:
	- none
 Return Values:
	- bool variable indicating the proper construction of the graph
************************************************************************/
bool CHyperGraphTracker::ConstructHyperGraph(void)
{

	return true;
}

/************************************************************************
 Method Name: LoadGraph
 Description: 
	- load the hypergraph for 3D tracking
 Input Arguments:
	- strGraphPath: path for the graph file
 Return Values:
	- bool variable indicating the proper loading of the graph 
************************************************************************/
bool CHyperGraphTracker::LoadGraph(const std::string strGraphPath)
{
	return true;
}

/************************************************************************
 Method Name: LoadDetections
 Description: 
	- 
 Input Arguments:
	- 
 Return Values:
	- bool variable indicating the proper loading of detections
************************************************************************/
bool CHyperGraphTracker::LoadDetections(void)
{
	if (!bInit_) { return false; }

	numDetections_ = 0;
	minDetectionHeight_ = DBL_MAX;

	int vecIdx = 0;	
	int numDetections = 0;
	float x, y, w, h;
	for (int fIdx = SET_.startFrameIdx(); fIdx <= SET_.endFrameIdx(); fIdx++, vecIdx++)	
	{
		for (int cIdx = 0; cIdx < SET_.numCams(); cIdx++)
		{
			printf("\rRead detection of cam %02d/%02d at frame %04d/%04d ... ", cIdx+1, SET_.numCams(), vecIdx+1, SET_.numFrames());
			std::string strDetectionFilePath = hj::fullfile(SET_.GetDetectionPath(cIdx), hj::sprintf("frame_%04d.txt", fIdx));
			FILE *fid;
			try
			{		
				fopen_s(&fid, strDetectionFilePath.c_str(), "r");
				if (NULL == fid)
				{
					return false; 
				}

				// read # of detections
				fscanf_s(fid, "numBoxes:%d\n", &numDetections);
				vecvecPtDetectionSets_[vecIdx][cIdx].resize(numDetections, NULL);

				// read box infos
				for (int dIdx = 0; dIdx < numDetections; dIdx++)
				{
					CDetection newDetection;
					fscanf_s(fid, "{\n\tROOT:{%f,%f,%f,%f}\n", &x, &y, &w, &h);
					newDetection.id_           = numDetections_++;
					newDetection.camIdx_       = cIdx;
					newDetection.frameIdx_     = fIdx;
					newDetection.rect_         = cv::Rect2d((double)x-1.0, (double)y-1.0, (double)w, (double)h);
					newDetection.bottomCenter_ = cv::Point2d((double)x-1.0 + 0.5*(double)w, (double)y - 1.0 + (double)h);
					vecCamModels_[cIdx].imageToWorld(x-1.0, y-1.0, 0.0, newDetection.location3D_.x, newDetection.location3D_.y);

					// read part info			
					for (unsigned int partIdx = 0; partIdx < NUM_DPM_PARTS; partIdx++)
					{
						char strPartName[20];
						sprintf_s(strPartName, "\t%s:", DETCTION_PART_NAME[partIdx]);
						fscanf_s(fid, strPartName);
						fscanf_s(fid, "{%f,%f,%f,%f}\n", &x, &y, &w, &h);
						newDetection.partRects_[partIdx] = cv::Rect2d((double)x-1.0, (double)y-1.0, (double)w, (double)h);
					}
					fscanf_s(fid, "}\n");			

					listDetections_.push_back(newDetection);
					vecvecPtDetectionSets_[vecIdx][cIdx][dIdx] = &listDetections_.back();

					// minimum detection size
					if (minDetectionHeight_ > (double)h) { minDetectionHeight_ = (double)h; }
				}
				fclose(fid);
			}
			catch (int nError)
			{
				printf("\n[ERROR] file open error with detection result reading: %d\n", nError);
				return false;
			}
		}
	}

	printf("done\n");
	return true;
}

/************************************************************************
 Method Name: LoadDetections
 Description: 
	- 
 Input Arguments:
	- 
 Return Values:
	- 
************************************************************************/
bool CHyperGraphTracker::GenerateReconstructions(void)
{
	for (int tIdx = 0; tIdx < vecvecPtDetectionSets_.size(); tIdx++)
	{
		// generate combinations

		// 
	}
	return true;
}



//()()
//('')HAANJU.YOO

