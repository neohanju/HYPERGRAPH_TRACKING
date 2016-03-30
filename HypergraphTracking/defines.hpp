#pragma once

#include <string>

/////////////////////////////////////////////////////////////////////////
// PATH
/////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////
// INPUT RELATED
/////////////////////////////////////////////////////////////////////////
#define NUM_DPM_PARTS (8)
const char* const DETCTION_PART_NAME[NUM_DPM_PARTS] = {"HEAD", "F1", "S1", "GR", "S2", "A1", "A2", "F2"};

typedef enum { PETS_S2L1 = 0, PETS_S2L2, PETS_S2L3, NUM_PETS_SENARIO } PETS_SCENARIO;
const int START_FRAME_INDICES  [NUM_PETS_SENARIO] = {0, 0, 0};
const int END_FRAME_INDICES    [NUM_PETS_SENARIO] = {794, 432, 239};
const std::string SCENARIO_PATH[NUM_PETS_SENARIO] = {"S2/L1/Time_12-34", "S2/L2/Time_14-55", "S2/L3/Time_14-41" };

/////////////////////////////////////////////////////////////////////////
// EVALUATION SETTING
/////////////////////////////////////////////////////////////////////////
#define CROP_ZONE_X_MIN (-14069.6)
#define CROP_ZONE_X_MAX (4981.3)
#define CROP_ZONE_Y_MIN (-14274.0)
#define CROP_ZONE_Y_MAX (1733.5)
#define CROP_ZONE_MARGIN (1000.0)

/////////////////////////////////////////////////////////////////////////
// PARAMETERS
/////////////////////////////////////////////////////////////////////////

const double DEFAULT_HEIGHT = 1700.0;

// default setting written in the paper
const double FRAME_RATE = 7.5; // video frame rate
const double V_MAX    = 5000.0; // the maximal walking speed of a person (5 m/s)
const double D_B_MAX  = 1000.0; // "close to the boundary" condition (1 m)
const double ALPHA_H  = 2.0; // ratio of the boundary detection's height to the minimum detectable height of detections
const double P_EN_MAX = 0.1; // entering probability
const double P_EX_MAX = 0.1; // exiting probability
const double EPS_DET  = 4.0; // expected detection error in the image coordinate
const double EPS_CAL  = 500.0; // expected calibration error in the 3D space
const double P_FP     = 0.05; // beta, false positive ratio
const double P_FN[NUM_PETS_SENARIO] = {0.1, 0.3, 0.4}; // gamma, false positive ratios
const int    DELTA_T_MAX = 9; // the maximum frame gap

struct stHypergraphTrackingParams
{
	// not in the paper, but from the author (by e-mail)
	double P_EN_TAU; // lifetime for decaying function in the entering probability
	double P_EX_TAU; // lifetime for decaying function in the exiting probability
	double DETECTION_MIN_HEIGHT;
};


//()()
//('')HAANJU.YOO


