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
	SET_ = SET;
	bInit_ = true;	

	///////////////////////////////////////////////////////////
	// CAMERA MODEL INITIALIZATION
	///////////////////////////////////////////////////////////	
	vecCamModels_.resize(SET_.numCams());
	for (int camIdx = 0; camIdx < SET_.numCams(); camIdx++)
	{
		//----------------------------------------------------
		// READ CALIBRATION INFORMATION
		//----------------------------------------------------
		std::string strCalibrationFilePath = hj::fullfile(SET_.GetCalibrationPath(), hj::sprintf("View_%03d.xml", SET_.GetCamIdx(camIdx)));
		std::cout << "Reading calibration information of camera " << SET_.GetCamIdx(camIdx) << "...";
		if (vecCamModels_[camIdx].fromXml(strCalibrationFilePath))
		{
			std::cout << " done" << std::endl;
			std::cout << " Camera position : (" << vecCamModels_[camIdx].cposx() << ", " 
				                                << vecCamModels_[camIdx].cposy() << ", " 
                                                << vecCamModels_[camIdx].cposz() << ")" << std::endl;
		}
		else
		{
			std::cout << " fail!" << std::endl;
		}
	}

	///////////////////////////////////////////////////////////
	// DETECTION INITIALIZATION
	///////////////////////////////////////////////////////////	
	numDetections_ = 0;
	vecvecDetectionSets_.resize(SET_.numFrames());
	for (int fIdx = 0; fIdx < SET_.numFrames(); fIdx++)
	{
		vecvecDetectionSets_[fIdx].resize(SET_.numCams());
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
	// DETECTION FINALIZATION
	///////////////////////////////////////////////////////////	
	for (int fIdx = 0; fIdx < vecvecDetectionSets_.size(); fIdx++)
	{		
		for (int cIdx = 0; cIdx < vecvecDetectionSets_[fIdx].size(); cIdx++)
		{
			vecvecDetectionSets_[fIdx][cIdx].clear();
		}		
		vecvecDetectionSets_[fIdx].clear();
	}
	vecvecDetectionSets_.clear();

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
				vecvecDetectionSets_[vecIdx][cIdx].resize(numDetections);

				// read box infos
				for (int dIdx = 0; dIdx < numDetections; dIdx++)
				{
					fscanf_s(fid, "{\n\tROOT:{%f,%f,%f,%f}\n", &x, &y, &w, &h);
					vecvecDetectionSets_[vecIdx][cIdx][dIdx].id_       = numDetections_++;
					vecvecDetectionSets_[vecIdx][cIdx][dIdx].camIdx_   = cIdx;
					vecvecDetectionSets_[vecIdx][cIdx][dIdx].frameIdx_ = fIdx;
					vecvecDetectionSets_[vecIdx][cIdx][dIdx].rect_     = 
						cv::Rect2d((double)x, (double)y, (double)w, (double)h);

					// read part info			
					for (unsigned int partIdx = 0; partIdx < NUM_DPM_PARTS; partIdx++)
					{
						char strPartName[20];
						sprintf_s(strPartName, "\t%s:", DETCTION_PART_NAME[partIdx]);
						fscanf_s(fid, strPartName);
						fscanf_s(fid, "{%f,%f,%f,%f}\n", &x, &y, &w, &h);
						vecvecDetectionSets_[vecIdx][cIdx][dIdx].partRects_[partIdx] = 
							cv::Rect2d((double)x, (double)y, (double)w, (double)h);
					}
					fscanf_s(fid, "}\n");			
				}
				fclose(fid);
			}
			catch (int dwError)
			{
				printf("\n[ERROR] file open error with detection result reading: %d\n", dwError);
				return false;
			}
		}
	}

	printf("done\n");
	return true;
}


//()()
//('')HAANJU.YOO

