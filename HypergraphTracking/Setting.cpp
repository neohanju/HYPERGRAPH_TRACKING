#include "Setting.h"
#include "hjlib.h"

#include <iostream>
#include <sstream>
#include <fstream>

typedef std::pair<std::string, std::string> PARAM_PAIR;
typedef std::vector<PARAM_PAIR> PARAM_SET;

const char STR_COMMENT_START = '%';
const char DELIM = '=';
const char DELIM_ARRAY = ',';
const int  MAX_CHARS_PER_LINE = 10240;

typedef enum { PETS_S2L1 = 0, PETS_S2L2, PETS_S2L3, NUM_PETS_SENARIO } PETS_SCENARIO;
const int START_FRAME_INDICES  [NUM_PETS_SENARIO] = {0, 0, 0};
const int END_FRAME_INDICES    [NUM_PETS_SENARIO] = {794, 432, 239};
const std::string SCENARIO_PATH[NUM_PETS_SENARIO] = { "S2/L1/Time_12-34", "S2/L2/Time_14-55", "S2/L3/Time_14-41" };

CSetting::CSetting(void)
	: bInit_(false), numCams_(0), startFrameIdx_(0), endFrameIdx_(0), numFrames_(0)
{
}

CSetting::CSetting(const char *strSettingPath)
	: bInit_(false), numCams_(0), startFrameIdx_(0), endFrameIdx_(0), numFrames_(0)
{
	LoadSetting(strSettingPath);
}

CSetting::~CSetting(void)
{
}


bool CSetting::LoadSetting(const char *strSettingPath)
{
	numCams_ = 0;
	strDatasetPath_.clear();
	strDatasetScenario_.clear();

	/////////////////////////////////////////////////////////////////////////////
	// READ FILE & PARSING
	/////////////////////////////////////////////////////////////////////////////
	PARAM_SET params;	

	// file reading object
	std::ifstream fstrIn;
	fstrIn.open(strSettingPath);
	if (!fstrIn.good()) 
	{
		std::cerr << "Cannot find file <" << strSettingPath << ">";
		return false; // exit if file not found
	}

	std::vector<std::string> lines;
	while (!fstrIn.eof())
	{
		char buf[MAX_CHARS_PER_LINE];
		fstrIn.getline(buf, MAX_CHARS_PER_LINE);
		if (STR_COMMENT_START == *buf || 0 == strlen(buf)) { continue; }
		
		std::stringstream ss(buf);
		std::string item;
		std::vector<std::string> tokens;
		
		while (std::getline(ss, item, DELIM))
		{
			tokens.push_back(item);
		}
		params.push_back(std::make_pair(tokens[0], tokens[1]));
	}
	fstrIn.close();

	/////////////////////////////////////////////////////////////////////////////
	// SET
	/////////////////////////////////////////////////////////////////////////////
	std::string strDatasetBasePath;
	std::string strDetectionPath;
	int startFrameIdx = -1;
	int endFrameIdx   = -1;
	for (int paramIdx = 0; paramIdx < params.size(); paramIdx++)
	{		
		if (0 == params[paramIdx].first.compare("DATASET_BASE_PATH"))
		{
			strDatasetBasePath = params[paramIdx].second;
		}
		else if (0 == params[paramIdx].first.compare("DATASET_SCENARIO"))
		{
			strDatasetScenario_ = params[paramIdx].second;
		}
		else if (0 == params[paramIdx].first.compare("CAMERA_INDICES"))
		{
			ParseArray(params[paramIdx].second, cameraIdxs_);
			numCams_ = (int)cameraIdxs_.size();
			vecStrViewPaths_.resize(numCams_);
			vecStrDetectionPaths_.resize(numCams_);
		}
		else if (0 == params[paramIdx].first.compare("START_FRAME_IDX"))
		{
			startFrameIdx = std::stoi(params[paramIdx].second);
		}
		else if (0 == params[paramIdx].first.compare("END_FRAME_IDX"))
		{
			endFrameIdx = std::stoi(params[paramIdx].second);
		}
		else if (0 == params[paramIdx].first.compare("CALIBRATION_PATH"))
		{
			strCalibrationPath_ = params[paramIdx].second;
		}
		else if (0 == params[paramIdx].first.compare("DETECTION_PATH"))
		{
			strDetectionPath = params[paramIdx].second;
		}
		else if (0 == params[paramIdx].first.compare("GROUND_TRUTH_PATH"))
		{
			strGroundTruthPath_ = params[paramIdx].second;
		}
		else if (0 == params[paramIdx].first.compare("RESULT_SAVE_PATH"))
		{
			strResultPath_ = params[paramIdx].second;
		}
	}

	if (0 == strDatasetBasePath.size() || 0 == numCams_)
	{
		std::cout << "[WARNING] wrong setting file. Check dataset path!!" << std::endl;
		return false;
	}

	
	// scenario and frame indices
	PETS_SCENARIO curScenario;
	if      (0 == strDatasetScenario_.compare("L1")) { curScenario = PETS_S2L1; }
	else if (0 == strDatasetScenario_.compare("L2")) { curScenario = PETS_S2L2; }	
	else if (0 == strDatasetScenario_.compare("L3")) { curScenario = PETS_S2L3; }
		
	strDatasetPath_ = hj::fullfile(strDatasetBasePath, SCENARIO_PATH[curScenario]);
	startFrameIdx_  = startFrameIdx < 0 ? START_FRAME_INDICES[curScenario] : startFrameIdx;
	endFrameIdx_    = endFrameIdx   < 0 ? END_FRAME_INDICES[curScenario]   : endFrameIdx;
	numFrames_      = endFrameIdx_ - startFrameIdx_ + 1;

	// calibration path
	strCalibrationPath_ = hj::fullfile(strDatasetPath_, strCalibrationPath_);

	// view and detection paths
	for (int cIdx = 0; cIdx < numCams_; cIdx++)
	{
		std::string strViewFolder = hj::sprintf("View_%03d", cameraIdxs_[cIdx]);
		vecStrViewPaths_[cIdx] = hj::fullfile(strDatasetPath_, strViewFolder);
		vecStrDetectionPaths_[cIdx] = hj::fullfile(vecStrViewPaths_[cIdx], strDetectionPath);
	}

	bInit_ = true;

	return true;
}

void CSetting::ParseArray(const std::string strInput, std::vector<int> &output)
{
	output.clear();
	output.reserve(100);
	std::stringstream ss(strInput);
	std::string item;
	std::vector<std::string> tokens;
		
	while (std::getline(ss, item, DELIM_ARRAY))
	{
		output.push_back(std::stoi(item));		
	}
}


//()()
//('')HAANJU.YOO

