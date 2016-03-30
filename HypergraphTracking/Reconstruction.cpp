#include "Reconstruction.h"
#include "hjlib.h"
#include <limits>

inline double functionF(double d, double dmin, double dmax) { return 0.5 * hj::erfc(4.0 * (d - dmin) / (dmax - dmin) - 2.0); }

CReconstruction::CReconstruction(const DetectionSet &detections,
								 const std::vector<Etiseo::CameraModel> &vecCamModels,
								 const std::vector<cv::Mat> &vecMatProjectionSensitivity,
								 const std::vector<cv::Mat> &vecMatDistanceFromBoundary,
								 const double fpRatio, const double fnRatio,
								 const double tauEnter, const double tauExit,
								 const double minDetectionHeight)
{
	assert(fpRatio > 0.0 && fpRatio < 1.0 && fnRatio > 0.0 && fnRatio < 1.0);

	bValid_             = false;
	id_                 = 0;	
	numDetections_      = 0;
	detections_         = detections;
	location3D_         = cv::Point3d(0.0, 0.0, 0.0);
	costEnter_          = 0.0;
	costExit_           = 0.0;
	costReconstruction_ = 0.0;

	///////////////////////////////////////////////////////////
	// RECONSTRUCTION
	///////////////////////////////////////////////////////////	
	double x = 0.0, y = 0.0;
	int numCameras = (int)detections_.size();
	for (int cIdx = 0; cIdx < numCameras; cIdx++)
	{
		if (NULL == detections_[cIdx]) { continue; }
		frameIdx_    = detections_[cIdx]->frameIdx_;
		location3D_ += detections_[cIdx]->location3D_;
		numDetections_++;
	}	
	if (0 == numDetections_) { return; } // check nullity of detections
	location3D_ /= (double)numDetections_;

	// get reconstruction probability
	double P_rec = 0.5;
	if (1 < numDetections_)
	{
		// reconstruction error
		double reconstructionError = 0.0;
		for (int cIdx = 0; cIdx < numCameras; cIdx++)
		{
			if (NULL == detections_[cIdx]) { continue; }
			reconstructionError += (detections_[cIdx]->location3D_ - location3D_).dot(detections_[cIdx]->location3D_ - location3D_);
		}
		reconstructionError = std::sqrt(reconstructionError / (double)numDetections_);

		// maximum allowable error
		double eps_max = 0.0;
		for (int cIdx = 0; cIdx < numCameras; cIdx++)
		{
			if (NULL == detections_[cIdx]) { continue; }
			eps_max += vecMatProjectionSensitivity[cIdx].at<float>((int)detections_[cIdx]->bottomCenter_.y, 
																   (int)detections_[cIdx]->bottomCenter_.x);
		}
		eps_max = EPS_DET * eps_max + EPS_CAL;

		// non-allowable reconstruction error -> invalidate the current reconstruction
		if (reconstructionError > eps_max) { return; }

		P_rec = functionF(reconstructionError, 0.0, eps_max);
		if (0.0 == P_rec) { return; }
	}

	// get the numver of visible cameras
	double detectionHalfWidth = 0.0;
	std::vector<cv::Point2d> pointOnImage(numCameras, cv::Point2d(0.0 ,0.0));
	int numVisibleCameras = numDetections_;	
	std::vector<bool> vecVisibility(numCameras, true);
	for (int cIdx = 0; cIdx < numCameras; cIdx++)
	{
		if (NULL != detections_[cIdx])
		{
			pointOnImage[cIdx] = detections_[cIdx]->bottomCenter_;
			continue;
		}
		vecCamModels[cIdx].worldToImage(location3D_.x, location3D_.y, 0.0, pointOnImage[cIdx].x, pointOnImage[cIdx].y);

		// pad in for detection probability
		cv::Point2d reprojectedTopPoint, pointDiff;
		vecCamModels[cIdx].worldToImage(location3D_.x, location3D_.y, DEFAULT_HEIGHT, reprojectedTopPoint.x, reprojectedTopPoint.y);
		pointDiff = reprojectedTopPoint - pointOnImage[cIdx];
		detectionHalfWidth = pointDiff.dot(pointDiff) / 6.0;

		if (pointOnImage[cIdx].x < detectionHalfWidth || pointOnImage[cIdx].y < detectionHalfWidth
			|| pointOnImage[cIdx].x >= (double)vecCamModels[cIdx].width() - detectionHalfWidth
			|| pointOnImage[cIdx].y >= (double)vecCamModels[cIdx].height() - detectionHalfWidth)
		{			
			vecVisibility[cIdx] = false;
			continue;
		}
		numVisibleCameras++;
	}

	///////////////////////////////////////////////////////////
	// RECONSTRUCTION COST
	///////////////////////////////////////////////////////////
	// C_k = |R_k| log (beta / (1 - beta)) 
	//     + (n(R_k) - |R_k| lob ((1 - gamma) / gamma) 
	//     + log ((1 - P_rec(R_k) / P_rec(R_k))

	costReconstruction_ = numDetections_ * std::log(fpRatio / (1.0 - fpRatio))
		                + (numVisibleCameras - numDetections_) * std::log((1.0 - fnRatio)/ fnRatio)
                        + std::log((1.0 - P_rec) / P_rec);

	///////////////////////////////////////////////////////////
	// ENTER and EXIT COST
	///////////////////////////////////////////////////////////	
	double maxDistance = -100.0, curDistance = 0.0;
	double maxDetectionHeight = 0.0;
	for (int cIdx = 0; cIdx < numCameras; cIdx++)
	{
		if (!vecVisibility[cIdx]) { continue; }
		curDistance = vecMatDistanceFromBoundary[cIdx].at<float>((int)pointOnImage[cIdx].y, (int)pointOnImage[cIdx].x);
		if (maxDistance < curDistance) { maxDistance = curDistance; }
		if (maxDetectionHeight < detections_[cIdx]->rect_.height) { maxDetectionHeight = detections_[cIdx]->rect_.height; }
	}

	double P_enter = P_EN_MAX;
	double P_exit  = P_EX_MAX;
	bool bInBoundaryArea = true;
	if (D_B_MAX < maxDistance && maxDetectionHeight > ALPHA_H * minDetectionHeight)
	{		
		// not in the boundary area
		P_enter *= std::exp(-(maxDistance - D_B_MAX) / tauEnter);
		P_exit  *= std::exp(-(maxDistance - D_B_MAX) / tauExit);
	}
	costEnter_ = -std::log(P_enter);
	costExit_  = -std::log(P_exit);

	bValid_ = true;
}

CReconstruction::~CReconstruction(void)
{
	detections_.clear();
}

double CReconstruction::GetTransitionCost(const CReconstruction &currR, const CReconstruction &nextR, const int deltaMax, const double fnRatio)
{
	// C_k,l = - log ( P_link (R_l | R_k) )
	// P_link (R_l | R_k) = P_cond (X_l | X_k, delta_time) * P_delta (delta_time)

	int timeGap = nextR.frameIdx_ - currR.frameIdx_;
	if (timeGap > DELTA_T_MAX) { return DBL_MAX; }
	double P_delta = std::pow(fnRatio, timeGap - 1);
	double P_cond  = functionF((nextR.location3D_ - currR.location3D_).dot(nextR.location3D_ - currR.location3D_), 
		                       0.0,
							   V_MAX / FRAME_RATE * timeGap);	
	if (std::numeric_limits<double>::epsilon() > P_cond) { return DBL_MAX; }
	return -std::log(P_cond * P_delta);
}

bool CReconstruction::IsCompatible(const CReconstruction &recons1, const CReconstruction &recons2)
{
	DetectionSet::iterator findIter;
	for (int d1Idx = 0; d1Idx < recons1.detections_.size(); d1Idx++)
	{
		for (int d2Idx = 0; d2Idx < recons2.detections_.size(); d2Idx++)
		{
			if (recons1.detections_[d1Idx] == recons2.detections_[d2Idx])
			{
				return false;
			}
		}
	}
	return true;
}


//()()
//('')HAANJU.YOO


