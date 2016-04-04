#include "HyperGraphTracker.h"
#include "Linkage.h"
#include "hjlib.h"
#include "SGSmooth.h"
#include "gurobi_c++.h"
#include "opencv2\highgui\highgui.hpp"
#include <time.h>

CHyperGraphTracker::CHyperGraphTracker()
	: bInit_(false), pGRBEnv_(NULL)
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

	///////////////////////////////////////////////////////////
	// RECONSTRUCTION INITIALIZATION
	///////////////////////////////////////////////////////////	
	numReconstructions_ = 0;

	///////////////////////////////////////////////////////////
	// (GUROBI) SOLVER INITIALIZATION
	///////////////////////////////////////////////////////////	
	try 
	{
		if(NULL == pGRBEnv_)
		{
			pGRBEnv_ = new GRBEnv();
			pGRBEnv_->set(GRB_IntParam_LogToConsole, 0);
		}
	} 
	catch (GRBException e) 
	{
		std::cout << "[ERROR] (GUROBI) Error code = " << e.getErrorCode() << std::endl;
		std::cout << e.getMessage() << std::endl;
		return false;
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

	///////////////////////////////////////////////////////////
	// (GUROBI) SOLVER FINALIZATION
	///////////////////////////////////////////////////////////
	try 
	{
		if(NULL != pGRBEnv_)
		{
			delete pGRBEnv_;
		}
	} 
	catch(GRBException e) 
	{
		std::cout << "[ERROR] (GUROBI) Error code = " << e.getErrorCode() << std::endl;
		std::cout << e.getMessage() << std::endl;
		return false;
	}

	///////////////////////////////////////////////////////////
	// TRACKING RESULT
	///////////////////////////////////////////////////////////	
	queueTracks_.clear();	

	return true;
}

/************************************************************************
 Method Name: Run
 Description: 
	- 
 Input Arguments:
	- none
 Return Values:
	- 
************************************************************************/
bool CHyperGraphTracker::Run(void)
{
	if (!bInit_) { return false; }
	if (!LoadDetections()) { return false; }
	if (!ConstructGraphAndSolving()) { return false; }
	return true;
}

/************************************************************************
 Method Name: Run
 Description: 
	- 
 Input Arguments:
	- none
 Return Values:
	- 
************************************************************************/
bool CHyperGraphTracker::SaveTrackingResultToFile(const std::string strFilePath)
{
	if (0 == queueTracks_.size())
	{
		printf("[ERROR] there is no tracking result to save\n");
		return false;
	}

	// make X and Y mats
	int numObjects = (int)queueTracks_.size();
	std::vector<std::deque<std::pair<int, cv::Point3d>>> vecQueueLocationsOnTime(SET_.numFrames());
	for (int tIdx = 0; tIdx < queueTracks_.size(); tIdx++)
	{
		int fIdx = queueTracks_[tIdx].timeStart_;
		for (int pIdx = 0; pIdx < queueTracks_[tIdx].locations_.size(); pIdx++, fIdx++)
		{
			vecQueueLocationsOnTime[fIdx].push_back(std::make_pair(queueTracks_[tIdx].id_, queueTracks_[tIdx].locations_[pIdx]));
		}
	}

	// read ground truth
	FILE *fp;		
	try
	{
		fopen_s(&fp, strFilePath.c_str(), "w");		
		fprintf_s(fp, "numObj=%d,numTime=%d\n", (int)queueTracks_.size(), SET_.numFrames());

		// write X
		fprintf_s(fp, "X={\n");
		for (int fIdx = 0; fIdx < SET_.numFrames(); fIdx++)
		{
			std::vector<float> curRow(numObjects, 0.0);
			for (int pIdx = 0; pIdx < vecQueueLocationsOnTime[fIdx].size(); pIdx++)
			{
				curRow[vecQueueLocationsOnTime[fIdx][pIdx].first] = (float)vecQueueLocationsOnTime[fIdx][pIdx].second.x;
			}
			for (int colIdx = 0; colIdx < numObjects; colIdx++)
			{
				fprintf_s(fp, "%f,", curRow[colIdx]);				
			}
			fprintf_s(fp, "\n");
		}		

		// write Y
		fprintf_s(fp, "}\nY={\n");
		for (int fIdx = 0; fIdx < SET_.numFrames(); fIdx++)
		{
			std::vector<float> curRow(numObjects, 0.0);
			for (int pIdx = 0; pIdx < vecQueueLocationsOnTime[fIdx].size(); pIdx++)
			{
				curRow[vecQueueLocationsOnTime[fIdx][pIdx].first] = (float)vecQueueLocationsOnTime[fIdx][pIdx].second.y;
			}
			for (int colIdx = 0; colIdx < numObjects; colIdx++)
			{
				fprintf_s(fp, "%f,", curRow[colIdx]);				
			}
			fprintf_s(fp, "\n");
		}
		fprintf_s(fp, "}\n");
		fclose(fp);
	}
	catch (int dwError)
	{
		printf("[ERROR] cannot open data. error code %d\n", dwError);
		return false;
	}
	return true;
}

/************************************************************************
 Method Name: Run
 Description: 
	- 
 Input Arguments:
	- none
 Return Values:
	- 
************************************************************************/
void CHyperGraphTracker::Visualization(int viewIdx)
{
	// for visualization, find rectangle on the target view
	std::vector<std::deque<std::pair<int, cv::Rect>>> vecQueueRectsOnTime(SET_.numFrames());	
	for (int tIdx = 0; tIdx < queueTracks_.size(); tIdx++)
	{
		int fIdx = queueTracks_[tIdx].timeStart_;
		for (int pIdx = 0; pIdx < queueTracks_[tIdx].locations_.size(); pIdx++, fIdx++)
		{
			cv::Rect rectOnView;
			cv::Point2d imagePoint;

			// bottom point
			vecCamModels_[viewIdx].worldToImage(queueTracks_[tIdx].locations_[pIdx].x, 
						                        queueTracks_[tIdx].locations_[pIdx].y, 
											    queueTracks_[tIdx].locations_[pIdx].z, 
											    imagePoint.x, imagePoint.y);
			rectOnView.x = (int)imagePoint.x;
			rectOnView.y = (int)imagePoint.y;
			if (NULL == queueTracks_[tIdx].reconstructions_[pIdx]
			|| NULL == queueTracks_[tIdx].reconstructions_[pIdx]->detections_[viewIdx])
			{
				rectOnView.x = (int)imagePoint.x;
				rectOnView.y = (int)imagePoint.y;

				// head point
				vecCamModels_[viewIdx].worldToImage(queueTracks_[tIdx].locations_[pIdx].x, 
						                            queueTracks_[tIdx].locations_[pIdx].y, 
												    DEFAULT_HEIGHT, 
												    imagePoint.x, imagePoint.y);
				rectOnView.height = (int)((double)rectOnView.y - imagePoint.y + 1.0);
				rectOnView.width = (int)(0.3 * (double)rectOnView.height);
				rectOnView.x -= (int)(0.5 * (double)rectOnView.width);
				rectOnView.y = (int)imagePoint.y;
			}
			else
			{
				rectOnView.width = (int)queueTracks_[tIdx].reconstructions_[pIdx]->detections_[viewIdx]->rect_.width;
				rectOnView.height = (int)queueTracks_[tIdx].reconstructions_[pIdx]->detections_[viewIdx]->rect_.height;
				rectOnView.x -= (int)(0.5 * (double)rectOnView.width);
				rectOnView.y -= rectOnView.height;
			}
			vecQueueRectsOnTime[fIdx].push_back(std::make_pair(queueTracks_[tIdx].id_, rectOnView));
		}
	}

	int imageFrameIdx = SET_.startFrameIdx();
	std::vector<cv::Scalar> COLORS = hj::GenerateColors((int)queueTracks_.size());
	cv::Mat inputFrame;
	for (int fIdx = 0; fIdx < SET_.numFrames(); fIdx++, imageFrameIdx++)
	{
		inputFrame = cv::imread(hj::fullfile(SET_.GetViewPath(viewIdx), hj::sprintf("frame_%04d.jpg", imageFrameIdx)));
		
		// writing frame info
		char strFrameInfo[100];
		sprintf_s(strFrameInfo, "Frame: %04d", imageFrameIdx);
		cv::rectangle(inputFrame, cv::Rect(5, 2, 145, 22), cv::Scalar(0, 0, 0), CV_FILLED);
		cv::putText(inputFrame, strFrameInfo, cv::Point(6, 20), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255));
		
		// draw rectangles
		for (int rectIdx = 0; rectIdx < vecQueueRectsOnTime[fIdx].size(); rectIdx++)
		{
			int trackId = vecQueueRectsOnTime[fIdx][rectIdx].first;
			cv::Rect *curRect = &vecQueueRectsOnTime[fIdx][rectIdx].second;
			cv::rectangle(inputFrame, *curRect, COLORS[trackId], 2);
			cv::putText(inputFrame, std::to_string(trackId), cv::Point(curRect->x, curRect->y+40), cv::FONT_HERSHEY_SIMPLEX, 1.0, COLORS[trackId]);

			// draw trajectory
			int curIdx = fIdx - queueTracks_[trackId].timeStart_;			
			cv::Point2d curPoint, prePoint;
			vecCamModels_[0].worldToImage(queueTracks_[trackId].locations_[curIdx].x, 
				                          queueTracks_[trackId].locations_[curIdx].y, 
										  queueTracks_[trackId].locations_[curIdx].z,
										  curPoint.x, curPoint.y);
			for (int preLocIdx = curIdx-1; preLocIdx >= std::max(0, curIdx - SET_.dispTrajectoryLength()+1); preLocIdx--)
			{
				vecCamModels_[0].worldToImage(queueTracks_[trackId].locations_[preLocIdx].x, 
				                              queueTracks_[trackId].locations_[preLocIdx].y,
											  queueTracks_[trackId].locations_[preLocIdx].z,
											  prePoint.x, prePoint.y);
				cv::line(inputFrame, curPoint, prePoint, COLORS[trackId], 2);
				curPoint = prePoint;
			}
		}

		cv::imshow("result", inputFrame);
		cv::waitKey(0);
	}

	cv::destroyAllWindows();
}

/************************************************************************
 Method Name: ConstructGraphAndSolving
 Description: 
	- construct hypergraph for 3D tracking and solving
 Input Arguments:
	- none
 Return Values:
	- bool variable indicating the proper construction of the graph
************************************************************************/
bool CHyperGraphTracker::ConstructGraphAndSolving(void)
{
	printf("Constructing graph ("); std::cout << hj::currentDateTime() << ") ..."<< std::endl;
	time_t timeStart = time(0);

	///////////////////////////////////////////////////////////
	// GENERATE RECONSTRUCTION
	///////////////////////////////////////////////////////////	
	GenerateReconstructions();

	if (NULL == pGRBEnv_) { return false; }
	try
	{
		///////////////////////////////////////////////////////////
		// GRAPH CONSTRUCTION
		///////////////////////////////////////////////////////////	

		// generate model
		GRBModel model = GRBModel(*pGRBEnv_);			
		GRBLinExpr linExprObjective = 0;			
		std::deque<GRBVar> grbVarReconstruction;
		std::deque<GRBVar> grbVarStarting;
		std::deque<GRBVar> grbVarEnding;
		std::deque<GRBVar> grbVarLinking;
		std::vector<GRBLinExpr> vecGRBConstraints;
		std::vector<std::string> vecGRBContsNames;		
		queueTracks_.clear();
		char strName[100];
		int numIncompatibleConsts = 0;
		
		//---------------------------------------------------------
		// RECONSTRUCTION EDGES / INCOMPATIBILITY CONDITIONS
		//---------------------------------------------------------
		for (int fIdx = 0; fIdx < vecvecPtReconstructions_.size(); fIdx++)
		{
			printf("\r Generate reconstruction edges at frame %04d/%04d ... ", fIdx+1, SET_.numFrames());
			for (int rIdx = 0; rIdx < vecvecPtReconstructions_[fIdx].size(); rIdx++)
			{
				CReconstruction *curReconstruction = vecvecPtReconstructions_[fIdx][rIdx];
				
				sprintf_s(strName, "reconstruction_%d", curReconstruction->id_);
				grbVarReconstruction.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY, strName));

				sprintf_s(strName, "starting_%d", curReconstruction->id_);
				grbVarStarting.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY, strName));
				double costEnter = 0 == fIdx? -std::log(P_EN_MAX) : curReconstruction->costEnter_;
				
				sprintf_s(strName, "ending_%d", curReconstruction->id_);
				grbVarEnding.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY, strName));
				double costExit = vecvecPtReconstructions_[fIdx].size() - 1 == fIdx? -std::log(P_EX_MAX) : curReconstruction->costExit_;

				// objective
				linExprObjective += curReconstruction->costReconstruction_ * grbVarReconstruction.back()
					              + costEnter * grbVarStarting.back()
					              + costExit * grbVarEnding.back();

				// generate incompatibility constraint
				for (int compRIdx = 0; compRIdx < rIdx; compRIdx++)
				{
					if (CReconstruction::IsCompatible(*curReconstruction, *vecvecPtReconstructions_[fIdx][compRIdx]))
					{ 
						continue;
					}					
					sprintf_s(strName, "incompatibility_%d", numIncompatibleConsts++);
					vecGRBContsNames.push_back(strName);
					vecGRBConstraints.push_back(grbVarReconstruction.back() 
						                       + grbVarReconstruction[vecvecPtReconstructions_[fIdx][compRIdx]->id_] 
					                           - 1.0);
				}
			}			
		}
		printf("done\n");

		//---------------------------------------------------------
		// LINKING EDGES
		//---------------------------------------------------------		
		std::vector<std::deque<CLinkage>> vecQueueLinkingEdges(vecPtReconstructions_.size());
		std::vector<std::deque<int>> vecQueueFluxOut(vecPtReconstructions_.size());
		std::vector<std::deque<int>> vecQueueFluxIn(vecPtReconstructions_.size());
		int linkId = 0;
		for (int fIdx = 0; fIdx < vecvecPtReconstructions_.size()-1; fIdx++)
		{
			printf("\r Generate linking edges at frame %04d/%04d ... ", fIdx+1, SET_.numFrames()-1);
			for (int jumpFIdx = fIdx + 1; jumpFIdx < std::min(fIdx + DELTA_T_MAX + 1, (int)vecvecPtReconstructions_.size()); jumpFIdx++)
			{
				for (int prevRIdx = 0; prevRIdx < vecvecPtReconstructions_[fIdx].size(); prevRIdx++)
				{
					for (int nextRIdx = 0; nextRIdx < vecvecPtReconstructions_[jumpFIdx].size(); nextRIdx++)
					{
						CLinkage newLink;						
						newLink.cost_ = CReconstruction::GetTransitionCost(
							*vecvecPtReconstructions_[fIdx][prevRIdx],
							*vecvecPtReconstructions_[jumpFIdx][nextRIdx],
							DELTA_T_MAX, P_FN[SET_.GetScenarioNumber()]);

						if (DBL_MAX == newLink.cost_) { continue; }
						newLink.id_   = linkId++;
						newLink.from_ = vecvecPtReconstructions_[fIdx][prevRIdx]->id_;
						newLink.to_   = vecvecPtReconstructions_[jumpFIdx][nextRIdx]->id_;

						// add variable
						sprintf_s(strName, "linking_%d", newLink.id_);
						grbVarLinking.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY, strName));

						// objective
						linExprObjective += newLink.cost_ * grbVarLinking.back();						

						// for constraints
						vecQueueFluxOut[newLink.from_].push_back(newLink.id_);
						vecQueueFluxIn[newLink.to_].push_back(newLink.id_);

						// for track construction
						vecQueueLinkingEdges[newLink.from_].push_back(newLink);
					}
				}
			}
		}
		printf("done\n");

		// flux conservation
		for (int rIdx = 0; rIdx < vecPtReconstructions_.size(); rIdx++)
		{
			printf("\r Generate flux conservation constraints with reconstruction %04d/%04d ... ", rIdx+1, vecPtReconstructions_.size());
			sprintf_s(strName, "fluxConservation_%d", rIdx);
			vecGRBContsNames.push_back(strName);
			sprintf_s(strName, "fluxInitiation_%d", rIdx);
			vecGRBContsNames.push_back(strName);
			GRBLinExpr curIncomingFlux = -grbVarReconstruction[rIdx] + grbVarStarting[rIdx];
			GRBLinExpr curOutgoingFlux = -grbVarReconstruction[rIdx] + grbVarEnding[rIdx];
			for (int linkIdx = 0; linkIdx < vecQueueFluxIn[rIdx].size(); linkIdx++)
			{
				curIncomingFlux += grbVarLinking[vecQueueFluxIn[rIdx][linkIdx]];				
			}
			for (int linkIdx = 0; linkIdx < vecQueueFluxOut[rIdx].size(); linkIdx++)
			{
				curOutgoingFlux += grbVarLinking[vecQueueFluxOut[rIdx][linkIdx]];
			}			
			vecGRBConstraints.push_back(curIncomingFlux);
			vecGRBConstraints.push_back(curOutgoingFlux);
		}	
		printf("done\n");		

		// set variables and objective (lazy update)
		model.update();
		model.setObjective(linExprObjective, GRB_MINIMIZE);	// min cost

		// set constraints (with sense symbols)
		GRBConstr *addedConstr = NULL;
		if (vecGRBContsNames.size() > 0)
		{
			char *arrGRBSenses = new char[vecGRBContsNames.size()];
			double *arrGRBRHSVals = new double[vecGRBContsNames.size()];
			memset(arrGRBRHSVals, 0, sizeof(double) * (int)vecGRBContsNames.size());

			// incompatibility
			memset(arrGRBSenses, GRB_LESS_EQUAL, sizeof(char) * (int)vecGRBContsNames.size());
			// flux
			for (int senseIdx = numIncompatibleConsts; senseIdx < vecGRBContsNames.size(); senseIdx++)
			{
				arrGRBSenses[senseIdx] = GRB_EQUAL;
			}

			addedConstr = model.addConstrs(&vecGRBConstraints[0], arrGRBSenses, arrGRBRHSVals, &vecGRBContsNames[0], (int)vecGRBContsNames.size());				
			delete arrGRBSenses;
			delete arrGRBRHSVals;
		}
		vecGRBConstraints.clear();
		vecGRBContsNames.clear();

		// elapsed time
		timeProblemConstruction_ = difftime(time(0), timeStart);
		printf(" construction time: "); hj::printTime(timeProblemConstruction_); printf("\n");
		
		///////////////////////////////////////////////////////////
		// SOLVING
		///////////////////////////////////////////////////////////			
		int optimStatus;
		unsigned int numSolution = 0;		

		//---------------------------------------------------------
		// OPTIMIZATION
		//---------------------------------------------------------
		printf("Solving graph ("); std::cout << hj::currentDateTime() << ") ... ";		
		timeStart = time(0);
		model.optimize();
		optimStatus = model.get(GRB_IntAttr_Status);
		numSolution = model.get(GRB_IntAttr_SolCount);
		if (GRB_OPTIMAL != optimStatus || 0 == numSolution)
		{
			// cannot find sufficient solutions
			printf("[ERROR](graph solving) optimization stopped!\n");
			return false;
		}
		timeProblemSolving_ = difftime(time(0), timeStart);
		printf("done\n");
		printf(" solving time: "); hj::printTime(timeProblemSolving_); printf("\n");
		
		//---------------------------------------------------------
		// SOLUTION PARSING AND TRACK GENERATION
		//---------------------------------------------------------		
		std::vector<ReconstructionSet> vecSelectedReconstructions(vecvecPtReconstructions_.size());

		// find starting reconstructions		
		ReconstructionSet vecStartingReconstructions;
		for (int varIdx = 0; varIdx < grbVarStarting.size(); varIdx++)
		{
			if (0 == grbVarStarting[varIdx].get(GRB_DoubleAttr_Xn)) { continue; }
			vecStartingReconstructions.push_back(vecPtReconstructions_[varIdx]);
		}

				// construction track		
		for (int trackIdx = 0; trackIdx < vecStartingReconstructions.size(); trackIdx++)
		{
			int prevFrameIdx = vecStartingReconstructions[trackIdx]->frameIdx_;
			cv::Point3d prevPoint(0.0, 0.0, 0.0);
			CTrack newTrack(trackIdx, prevFrameIdx);	
			CReconstruction* nextReconstruction = vecStartingReconstructions[trackIdx];
			newTrack.cost_ = nextReconstruction->costEnter_;
			bool bNextFound = false;
			do
			{
				// fill the gap with interpolation
				int timeGap = nextReconstruction->frameIdx_ - prevFrameIdx;
				for (int interpolationIdx = 1; interpolationIdx < timeGap; interpolationIdx++)
				{
					cv::Point3d curLocation = prevPoint + (nextReconstruction->location3D_ - prevPoint) / timeGap * interpolationIdx;
					newTrack.reconstructions_.push_back(NULL);
					newTrack.locations_.push_back(curLocation);
				}
				prevPoint = nextReconstruction->location3D_;
				prevFrameIdx = nextReconstruction->frameIdx_;

				newTrack.reconstructions_.push_back(nextReconstruction);
				newTrack.locations_.push_back(nextReconstruction->location3D_);
				newTrack.cost_ += nextReconstruction->costReconstruction_;		
				
				// check termination
				if (0 != grbVarEnding[nextReconstruction->id_].get(GRB_DoubleAttr_Xn)) { break; }

				// find next reconstruction				
				bNextFound = false;
				for (int linkIdx = 0; linkIdx < vecQueueLinkingEdges[nextReconstruction->id_].size(); linkIdx++)
				{
					CLinkage *curLink = &vecQueueLinkingEdges[nextReconstruction->id_][linkIdx];
					if(0 == grbVarLinking[curLink->id_].get(GRB_DoubleAttr_Xn)) { continue; }
					bNextFound = true;
					nextReconstruction = vecPtReconstructions_[curLink->to_];
					break;
				}
			}
			while (bNextFound);
			newTrack.timeEnd_ = newTrack.reconstructions_.back()->frameIdx_;
			newTrack.cost_ += newTrack.reconstructions_.back()->costExit_;

			queueTracks_.push_back(newTrack);
		}
		
		//---------------------------------------------------------
		// WRAP-UP
		//---------------------------------------------------------
		if (NULL != addedConstr) { delete [] addedConstr; }		
		for (int varIdx = 0; varIdx < grbVarReconstruction.size(); varIdx++) { model.remove(grbVarReconstruction[varIdx]); }
		for (int varIdx = 0; varIdx < grbVarStarting.size(); varIdx++)       { model.remove(grbVarStarting[varIdx]); }
		for (int varIdx = 0; varIdx < grbVarEnding.size(); varIdx++)         { model.remove(grbVarEnding[varIdx]); }
		for (int varIdx = 0; varIdx < grbVarLinking.size(); varIdx++)        { model.remove(grbVarLinking[varIdx]); }
		model.update();
		model.reset();
	}
	catch (GRBException e)
	{
		std::cout << "[ERROR](Sovling) Error code = " << e.getErrorCode() << std::endl;
		std::cout << e.getMessage() << std::endl;
	} 
	catch (...) 
	{
		std::cout << "[ERROR](Sovling) Exception during optimization" << std::endl;
	}

	// refinement of tracking result
	SmoothingTrackingResult();

	return true;
}

/************************************************************************
 Method Name: SmoothingTrackingResult
 Description: 
	- 
 Input Arguments:
	- 
 Return Values:
	- 
************************************************************************/
void CHyperGraphTracker::SmoothingTrackingResult(void)
{
	// for smoother
	std::vector<Qset> precomputedQsets;
	for (int windowSize = 1; windowSize <= SGS_DEFAULT_SPAN; windowSize++)
	{
		precomputedQsets.push_back(CSGSmooth::CalculateQ(windowSize));
	}

	for (int tIdx = 0; tIdx < queueTracks_.size(); tIdx++)
	{
		CSGSmooth smootherX, smootherY;
		smootherX.SetPrecomputedQsets(&precomputedQsets);
		smootherY.SetPrecomputedQsets(&precomputedQsets);
		for (int pIdx = 0; pIdx < queueTracks_[tIdx].locations_.size(); pIdx++)
		{
			smootherX.Insert(queueTracks_[tIdx].locations_[pIdx].x);			
			smootherY.Insert(queueTracks_[tIdx].locations_[pIdx].y);
		}

		for (int pIdx = 0; pIdx < queueTracks_[tIdx].locations_.size(); pIdx++)
		{
			queueTracks_[tIdx].locations_[pIdx].x = smootherX.GetResult(pIdx);
			queueTracks_[tIdx].locations_[pIdx].y = smootherY.GetResult(pIdx);
		}
	}
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
					newDetection.frameIdx_     = fIdx - SET_.startFrameIdx();
					newDetection.rect_         = cv::Rect2d((double)x-1.0, (double)y-1.0, (double)w, (double)h);
					newDetection.bottomCenter_ = cv::Point2d((double)x-1.0 + 0.5*(double)w, (double)y - 1.0 + (double)h);
					vecCamModels_[cIdx].imageToWorld(newDetection.bottomCenter_.x, newDetection.bottomCenter_.y, 
						0.0, newDetection.location3D_.x, newDetection.location3D_.y);

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
void CHyperGraphTracker::GenerateReconstructions(void)
{
	vecPtReconstructions_.clear();
	vecvecPtReconstructions_.clear();
	vecvecPtReconstructions_.resize(vecvecPtDetectionSets_.size());
	for (int fIdx = 0; fIdx < vecvecPtDetectionSets_.size(); fIdx++)
	{
		printf("\rGenerate reconstructions at frame %04d/%04d ... ", fIdx+1, SET_.numFrames());

		// generate combinations
		DetectionSet nullSet(SET_.numCams(), NULL);		
		std::deque<DetectionSet> detectionCombinations;
		GenerateDetectionCombinations(nullSet, 0, vecvecPtDetectionSets_[fIdx], detectionCombinations);

		vecvecPtReconstructions_[fIdx].reserve(detectionCombinations.size());
		for (int combIdx = 0; combIdx < detectionCombinations.size(); combIdx++)
		{
			CReconstruction newReconstruction(detectionCombinations[combIdx], 
				                              vecCamModels_, 
											  vecMatProjectionSensitivity_, 
											  vecMatDistanceFromBoundary_, 
											  P_FP, P_FN[SET_.GetScenarioNumber()], 
											  SET_.GetParamHGT()->P_EN_TAU, SET_.GetParamHGT()->P_EX_TAU, 
											  SET_.GetParamHGT()->DETECTION_MIN_HEIGHT);
			if (!newReconstruction.bValid_) { continue; }
			newReconstruction.id_ = numReconstructions_++;
			listReconstructions_.push_back(newReconstruction);
			vecPtReconstructions_.push_back(&listReconstructions_.back());
			vecvecPtReconstructions_[fIdx].push_back(&listReconstructions_.back());
		}
	}
	printf("done\n");
}

/************************************************************************
 Method Name: GenerateTrackletCombinations
 Description: 
	- Generate feasible 2D tracklet combinations
 Input Arguments:
	- combination: current combination
	- entireDetectionsAtEachCame: detection pools of each camera for combination
	- combinationQueue: queue for save combinations
	- camIdx: current camera index
 Return Values:
	- none
************************************************************************/
void CHyperGraphTracker::GenerateDetectionCombinations(DetectionSet curCombination,
													   const int curCamIdx,
													   const std::vector<DetectionSet> &entireDetectionsAtEachCame,
													   std::deque<DetectionSet> &outputCombinationQueue)
{
	if (curCamIdx >= SET_.numCams())
	{
		outputCombinationQueue.push_back(curCombination);
		return;
	}

	// for missing detection
	curCombination[curCamIdx] = NULL;
	GenerateDetectionCombinations(curCombination, curCamIdx+1, entireDetectionsAtEachCame, outputCombinationQueue);

	// iteratively generate combinations
	for (int dIdx = 0; dIdx < entireDetectionsAtEachCame[curCamIdx].size(); dIdx++)
	{
		curCombination[curCamIdx] = entireDetectionsAtEachCame[curCamIdx][dIdx];
		GenerateDetectionCombinations(curCombination, curCamIdx+1, entireDetectionsAtEachCame, outputCombinationQueue);
	}
}


//()()
//('')HAANJU.YOO

