/******************************************************************************
 *
 *                        WELCOME TO HJ LIBRARY
 *
 ******************************************************************************
 *               .__                           __.
 *                \ `\~~---..---~~~~~~--.---~~| /   
 *                 `~-.   `                   .~         _____ 
 *                     ~.                .--~~    .---~~~    /
 *                      / .-.      .-.      |  <~~        __/
 *                     |  |_|      |_|       \  \     .--'
 *                    /-.      -       .-.    |  \_   \_
 *                    \-'   -..-..-    `-'    |    \__  \_ 
 *                     `.                     |     _/  _/
 *                     ~-                .,-\   _/  _/
 *                      /                 -~~~~\ /_  /_
 *                     |               /   |    \  \_  \_ 
 *                     |   /          /   /      | _/  _/
 *                     |  |          |   /    .,-|/  _/ 
 *                     )__/           \_/    -~~~| _/
 *                       \                      /  \
 *                        |           |        /_---` 
 *                        \    .______|      ./
 *                        (   /        \    /
 *                        `--'          /__/
 *
 ******************************************************************************/
#pragma once
#include <string>
#include <cv.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace hj {

typedef std::pair<cv::Point2d, cv::Point2d> LINE_SEGMENT;
typedef std::pair<cv::Point3d, cv::Point3d> LINE_SEGMENT_3D;

// file interface related
//bool CreateDirectoryForWindows(const std::string &dirName);
cv::Mat ReadMatrix(const std::string strFilePath);

// math related
std::vector<std::vector<int>> nchoosek(int n, int k);
double erf(double x);
double erfc(double x);
cv::Mat histogram(const cv::Mat singleChannelImage, int numBin);
bool GetIntersection(const LINE_SEGMENT segment1, const LINE_SEGMENT segment2, cv::Point2d *intersection = NULL);
double Triangulation(const LINE_SEGMENT_3D &line1, const LINE_SEGMENT_3D &line2, cv::Point3d &outputMidPoint3D);

// string related
std::string sprintf(const std::string fmt_str, ...);
std::string fullfile(const std::string strBasePath, const std::string strFileName);

// display related
std::vector<cv::Scalar> GenerateColors(int numColor);
cv::Scalar hsv2bgr(double h, double s, double v);
void alphaRectangle(cv::Mat &image, const cv::Rect rect, const cv::Scalar color, const double alpha = 0.2);
cv::Mat MakeMatTile(std::vector<cv::Mat> *imageArray, unsigned int numRows, unsigned int numCols);

// time related
void printTime(double secs_);
const std::string currentDateTime();

}



//()()
//('')HAANJU.YOO

