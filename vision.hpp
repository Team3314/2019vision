#ifndef VISION_HPP
#define VISION_HPP

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <cstdlib>

#include <networktables/NetworkTable.h>
#include <stdlib.h>
#include <string.h>

#include "cap_gstreamer.hpp"

// Camera Parameters
const double HORZ_DEGREES_PER_PIXEL = 1;                        //left, right (70.42 / 640)
const double LEFT_MULTIPLIER = 0.091, RIGHT_MULTIPLIER = 0.076; //.764, RIGHT_MULTIPLIER = .875; .0835

//Unused because back to single camera setup
/*
// Camera Configuration Parameters
const double CALIBRATION_DISTANCE = 27.64; //string len. in inches div. 2 (hypotenuse)
const double LEFT_BASE_ANGLE = 0, RIGHT_BASE_ANGLE = 0; // degrees

// Camera Calibration Parameters
// using T bracket
const double LEFT_SEPARATION = 22 - 9.75; //inches
const double RIGHT_SEPARATION = 9.75;     //inches
const double TARGET_DISTANCE = 20.25;     //inches perpendicular
const bool USE_T_CALIBRATION = true;
const double CAMERA_SEPARATION = LEFT_SEPARATION + RIGHT_SEPARATION; // inches
*/

//OpenCV camera calc parameters
const int OPENCV_WIDTH = 640, OPENCV_HEIGHT = 480;
const double FOV_RADIANS = (LEFT_MULTIPLIER + RIGHT_MULTIPLIER) / 2 * OPENCV_WIDTH * (CV_PI / 180);

// Output stream parameters
const int STREAM_WIDTH = 320, STREAM_HEIGHT = 360, FRAMERATE = 15, BITRATE = 100000, PORT = 5801;

const double MIN_ASPECT_RATIO = 0.2, MAX_ASPECT_RATIO = 0.6, MIN_AREA_RATIO = 0.5, MIN_AREA = 20, MIN_OFFSET = 3, MAX_OFFSET = 30;

const cv::Scalar WHITE(255, 255, 255);
const cv::Scalar RED(0, 0, 255);
const cv::Scalar PINK(255, 0, 255);
const cv::Scalar YELLOW(0, 255, 255);
const cv::Scalar GREEN(0, 255, 0);

#endif