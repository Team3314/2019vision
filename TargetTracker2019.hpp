#ifndef TARGET_TRACKER2019_HPP
#define TARGET_TRACKER2019_HPP

#include "vision.hpp"
#include <string>
#include "CameraInfo.hpp"
#include "TargetTracker.hpp"
#include "Goal2019.hpp"


class TargetTracker2019 : public TargetTracker
{
  public:
    cv::Mat hsv;
    cv::Mat output;

    //double multiplier = 0;
    cv::Scalar minHSV{100, 100, 100};
    cv::Scalar maxHSV{140, 255, 255};

    double targetX = 0;
    double targetY = 0;
    double centeredTargetX = 0;
    double centeredTargetY = 0;

    double targetAngle = 0;
    double leftTargetAngle = 0;
    double rightTargetAngle = 0;
    double bottomTargetAngle = 0;

    int targetsFound = 0;
    bool hasLeft = false;
    bool hasRight = false;

    TargetTracker2019(CameraInfo *camInfo, bool Verbose, cv::Scalar MinHSV, cv::Scalar MaxHSV)
        : TargetTracker(camInfo, Verbose)

    {
        minHSV = MinHSV;
        maxHSV = MaxHSV;
    }
};
#endif