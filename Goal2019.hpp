#ifndef GOAL2019_HPP
#define GOAL2019_HPP

#include "vision.hpp"

#include <string>
#include "CameraInfo.hpp"
#include "TargetTracker2019.hpp"

class Goal2019
{
  public:
	std::vector<cv::Point> contour;
	cv::Rect rect;
	cv::RotatedRect rRect;
	cv::Point2f pts[4];
	cv::Point2f center;

	float angle;
	float offset;

	double rHeight;
	double rWidth;
	double rArea;
	double rRatio;

	double cArea;
	double cRatio;
	double areaRatio;

	bool isLeft;
	bool isRight;
	int partner;

	Goal2019(std::vector<cv::Point> Contour);
};

#endif