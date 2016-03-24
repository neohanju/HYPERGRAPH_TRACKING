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

// file interface related
//bool CreateDirectoryForWindows(const std::string &dirName);

// math related
bool GetIntersection(LINE_SEGMENT segment1, LINE_SEGMENT segment2, cv::Point2d *intersection = NULL);

// string related
std::string sprintf(const std::string fmt_str, ...);
std::string fullfile(const std::string strBasePath, const std::string strFileName);

// display related
std::vector<cv::Scalar> GenerateColors(int numColor);
cv::Scalar hsv2bgr(double h, double s, double v);
void alphaRectangle(cv::Mat &image, const cv::Rect rect, const cv::Scalar color, const double alpha = 0.2);
cv::Mat MakeMatTile(std::vector<cv::Mat> *imageArray, unsigned int numRows, unsigned int numCols);
}

//()()
//('')HAANJU.YOO

