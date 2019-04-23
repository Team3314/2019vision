#ifndef TARGET_TRACKER_HPP
#define TARGET_TRACKER_HPP

#include "vision.hpp"

#include <string>
#include "CameraInfo.hpp"
#include "Goal2019.hpp"

class TargetTracker
{
	cv::VideoCapture input;

public:
	long frame;
	bool verbose;
	CameraInfo *camInfo;
	cv::Mat source;

	TargetTracker(CameraInfo *camInfo, bool Verbose);
	virtual void capture();
	virtual void analyze();

	// utility functions
	double angleFromPixels(double ctx);
	double angleFromRawPixels(double tx);

	// static utility functions
	static cv::Point2f findLongEdge(cv::Point2f pts[]);
	static cv::Point2f findShortEdge(cv::Point2f pts[]);
	static float rrectAngleEdge(cv::Point2f usedEdge);
	static float rrectAngleOffset(float angle);
	static bool isBetween(double min, double max, double min2, double max2);
};

#endif