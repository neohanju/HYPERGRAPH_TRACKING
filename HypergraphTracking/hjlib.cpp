#include "hjlib.h"
#include <stdarg.h>  // For va_start, etc.
#include <memory>    // For std::unique_ptr

namespace hj {

///************************************************************************
// Method Name: CreateDirectoryForWindows
// Description: 
//	- Check wheter the directory exists or not
// Input Arguments:
//	- dirName: directory path
// Return Values:
//	- true: exists / false: non-exists
//************************************************************************/
//bool CreateDirectoryForWindows(const std::string &dirName)
//{
//	std::wstring wideStrDirName = L"";
//	wideStrDirName.assign(dirName.begin(), dirName.end());
//	if (CreateDirectory(wideStrDirName.c_str(), NULL) || ERROR_ALREADY_EXISTS == GetLastError()) { return true; }
//	return false;
//}

cv::Mat ReadMatrix(const std::string strFilePath)
{
	FILE *fp;
	int numRows = 0, numCols = 0;
	float curSensitivity = 0.0;
	cv::Mat matRead;

	try
	{
		fopen_s(&fp, strFilePath.c_str(), "r");
		if (NULL == fp) { return matRead; }

		fscanf_s(fp, "row:%d,col:%d\n", &numRows, &numCols);
		matRead = cv::Mat::zeros(numRows, numCols, CV_32FC1);

		for(unsigned int rowIdx = 0; rowIdx < (unsigned int)numRows; rowIdx++)
		{
			for(unsigned int colIdx = 0; colIdx < (unsigned int)numCols; colIdx++)
			{
				fscanf_s(fp, "%f,", &curSensitivity);
				matRead.at<float>(rowIdx, colIdx) = curSensitivity;
			}
			fscanf_s(fp, "\n");
		}
		fclose(fp);		
	}
	catch(int nError)
	{
		printf("[ERROR] cannot open matrix from %s with error %d\n", strFilePath, nError);
	}

	return matRead;
}

/************************************************************************
 Method Name: GetIntersection
 Description: 
	- Returns 'true' if the lines intersect, otherwise 'false'. In addition, 
	if the lines intersect the intersection point may be stored in the 'intersection'.
 Input Arguments:
	- 
 Return Values:
	- 
************************************************************************/
// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
// intersect the intersection point may be stored in the floats i_x and i_y.
bool GetIntersection(LINE_SEGMENT segment1, LINE_SEGMENT segment2, cv::Point2d *intersection)
{
	double s1_x, s1_y, s2_x, s2_y;
	s1_x = segment1.second.x - segment1.first.x;  s1_y = segment1.second.y - segment1.first.y;
    s2_x = segment2.second.x - segment2.first.x;  s2_y = segment2.second.y - segment2.first.y;

	double invDenominator = 1.0 / (-s2_x * s1_y + s1_x * s2_y);
	double s, t;
	s = (-s1_y * (segment1.first.x - segment2.first.x) + s1_x * (segment1.first.y - segment2.first.y)) * invDenominator;
    t = ( s2_x * (segment1.first.y - segment2.first.y) - s2_y * (segment1.first.x - segment2.first.x)) * invDenominator;

	if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
	{
		// collision detected
		if (NULL != intersection)
		{
			intersection->x = segment1.first.x + (t * s1_x);
			intersection->y = segment1.first.y + (t * s1_y);
		}
		return true;
	}
	return false;
}

//bool IsSegmentsIntersect(float p0_x, float p0_y, float p1_x, float p1_y, 
//    float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y)
//{
//	// p0 -> segment1.first
//	// p1 -> segment1.second
//	// p2 -> segment2.first
//	// p3 -> segment2.second
//    float s1_x, s1_y, s2_x, s2_y;
//    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
//    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;
//
//    float s, t;
//    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
//    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);
//
//    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
//    {
//        // Collision detected
//        if (i_x != NULL)
//            *i_x = p0_x + (t * s1_x);
//        if (i_y != NULL)
//            *i_y = p0_y + (t * s1_y);
//        return 1;
//    }
//
//    return 0; // No collision
//}

/************************************************************************
 Method Name: string_format
 Description: 
	- 'sprintf' with std::string
 Input Arguments:
	- 
 Return Values:
	- 
************************************************************************/
std::string sprintf(const std::string fmt_str, ...) 
{
	// reference: http://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf (modified with *_s things)
    int final_n, n = ((int)fmt_str.size()) * 2; /* Reserve two times as much as the length of the fmt_str */
    std::string str;
    std::unique_ptr<char []> formatted;
    va_list ap;
    while (true)
	{
        formatted.reset(new char [n]); /* Wrap the plain char array into the unique_ptr */
        strcpy_s(&formatted[0], n, fmt_str.c_str());
        va_start(ap, fmt_str);		
		final_n = vsnprintf_s(&formatted[0], n, _TRUNCATE, fmt_str.c_str(), ap);
        //final_n = vsnprintf(&formatted[0], n, fmt_str.c_str(), ap);
        va_end(ap);
        if (final_n < 0 || final_n >= n)
            n += abs(final_n - n + 1);
        else
            break;
    }
    return std::string(formatted.get());
}

std::string fullfile(const std::string strBasePath, const std::string strFileName)
{
	if (0 == strBasePath.size()) { return strFileName; }
	if ('/' == strBasePath.back() || '\\' == strBasePath.back()) { return strBasePath + strFileName; }
	return strBasePath + "/" + strFileName;
}

/************************************************************************
 Method Name: GenerateColors
 Description: 
	- generate distinguishable color set
 Input Arguments:
	- numColor: the number of colors needed
 Return Values:
	- vector of RGB color coordinates
************************************************************************/
std::vector<cv::Scalar> GenerateColors(int numColor)
{
	// refer: http://martin.ankerl.com/2009/12/09/how-to-create-random-colors-programmatically/
	//# use golden ratio
	//golden_ratio_conjugate = 0.618033988749895
	//h = rand # use random start value
	//gen_html {
	//  h += golden_ratio_conjugate
	//  h %= 1
	//  hsv_to_rgb(h, 0.5, 0.95)
	//}

	double golden_ratio_conjugate = 0.618033988749895;
	//double hVal = (double)std::rand()/(INT_MAX);
	double hVal = 0.0;
	std::vector<cv::Scalar> resultColors(numColor);
	for (int colorIdx = 0; colorIdx < numColor; colorIdx++)
	{
		hVal += golden_ratio_conjugate;
		hVal = std::fmod(hVal, 1.0);
		resultColors[colorIdx] = hsv2bgr(hVal, 0.5, 0.95);
	}
	return resultColors;
}

/************************************************************************
 Method Name: hsv2bgr
 Description: 
	- HSV -> BGR (OpenCV color order)
 Input Arguments:
	- h: hue
	- s: saturation
	- v: value
 Return Values:
	- RGB color coordinates
************************************************************************/
cv::Scalar hsv2bgr(double h, double s, double v)
{
	// refer: http://martin.ankerl.com/2009/12/09/how-to-create-random-colors-programmatically/
	//# HSV values in [0..1[
	//# returns [r, g, b] values from 0 to 255
	//def hsv_to_rgb(h, s, v)
	//  h_i = (h*6).to_i
	//  f = h*6 - h_i
	//  p = v * (1 - s)
	//  q = v * (1 - f*s)
	//  t = v * (1 - (1 - f) * s)
	//  r, g, b = v, t, p if h_i==0
	//  r, g, b = q, v, p if h_i==1
	//  r, g, b = p, v, t if h_i==2
	//  r, g, b = p, q, v if h_i==3
	//  r, g, b = t, p, v if h_i==4
	//  r, g, b = v, p, q if h_i==5
	//  [(r*256).to_i, (g*256).to_i, (b*256).to_i]
	//end

	int h_i = (int)(h * 6);
	double f = h * 6 - (double)h_i;
	double p = v * (1 - s);
	double q = v * (1 - f * s);
	double t = v * (1 - (1 - f) * s);
	double r, g, b;
	switch (h_i)
	{
	case 0: r = v; g = t; b = p; break;
	case 1: r = q; g = v; b = p; break;
	case 2: r = p; g = v; b = t; break;
	case 3: r = p; g = q; b = v; break;
	case 4: r = t; g = p; b = v; break;
	case 5: r = v; g = p; b = q; break;
	default: 
		break;
	}
	
	return cv::Scalar((int)(b * 255), (int)(g * 255), (int)(r * 255));
}

/************************************************************************
 Method Name: alphaRectangle
 Description: 
	- 
 Input Arguments:
	- 
	- 
	- 
 Return Values:
	- 
************************************************************************/
void alphaRectangle(cv::Mat &image, const cv::Rect rect, const cv::Scalar color, const double alpha)
{
	int x = std::max(0, rect.x);
	int y = std::max(0, rect.y);
	int w = std::min(image.cols-x, rect.width);
	int h = std::min(image.rows-y, rect.height);
	cv::Mat roi = image(cv::Rect(x, y, w, h));
    cv::Mat mask(roi.size(), CV_8UC3, color);     
    cv::addWeighted(mask, alpha, roi, 1.0 - alpha , 0.0, roi); 
}

/************************************************************************
 Method Name: MakeMatTile
 Description: 
	- Make one result image for multi-camera input for visualization
 Input Arguments:
	- imageArray: 
	- numRows: number of rows in tile image
	- numCols: number of colums in tile image
 Return Values:
	- cv::Mat: result image
************************************************************************/
cv::Mat MakeMatTile(std::vector<cv::Mat> *imageArray, unsigned int numRows, unsigned int numCols)
{
	// column first arrangement
	unsigned int numImage = (unsigned int)imageArray->size();	
	unsigned int acturalNumCols = numImage < numCols ? numImage : numCols;
	unsigned int acturalNumRows = (0 == numImage % numCols) ? numImage / numCols : numImage / numCols + 1;

	// find maximum size image
	cv::Size maxSize = (*imageArray)[0].size();
	for (unsigned int imageIdx = 0; imageIdx < numImage; imageIdx++)
	{
		if((*imageArray)[imageIdx].rows > maxSize.height){ maxSize.height = (*imageArray)[imageIdx].rows; }
		if((*imageArray)[imageIdx].cols > maxSize.width){ maxSize.width = (*imageArray)[imageIdx].cols; }
	}

	// make augmenting matrix
	std::vector<cv::Mat> augMats;
	cv::Mat baseMat;
	baseMat = 1 == (*imageArray)[0].channels()? cv::Mat::zeros(maxSize, CV_8UC1): cv::Mat::zeros(maxSize, CV_8UC3); 	
	unsigned int numAugMats = acturalNumRows * acturalNumCols;
	for (unsigned int imageIdx = 0; imageIdx < numAugMats; imageIdx++)
	{
		cv::Mat augMat = baseMat.clone();
		if (imageIdx < numImage)
		{
			(*imageArray)[imageIdx].copyTo(augMat(cv::Rect(0, 0, (*imageArray)[imageIdx].cols, (*imageArray)[imageIdx].rows)));
		}
		augMats.push_back(augMat);
	}

	// matrix concatenation
	cv::Mat hConcatMat;
	cv::Mat resultMat;
	for (unsigned int rowIdx = 0; rowIdx < acturalNumRows; rowIdx++)
	{
		unsigned int startIdx = rowIdx * acturalNumCols;		
		for (unsigned int colIdx = 0; colIdx < acturalNumCols; colIdx++)
		{
			if (0 == colIdx)
			{
				hConcatMat = augMats[startIdx].clone();
				continue;
			}
			cv::hconcat(hConcatMat, augMats[startIdx + colIdx], hConcatMat);
		}
		if (0 == rowIdx)
		{
			resultMat = hConcatMat.clone();
			continue;
		}
		cv::vconcat(resultMat, hConcatMat, resultMat);
	}

	return resultMat;
}

}

//()()
//('')HAANJU.YOO

