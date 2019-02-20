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

//const int LEFT_CAMERA_ID = 1, RIGHT_CAMERA_ID = 2;

// Camera Parameters
const double HORZ_DEGREES_PER_PIXEL = 1;                        //left, right (70.42 / 640)
const double LEFT_MULTIPLIER = 0.091, RIGHT_MULTIPLIER = 0.076; //.764, RIGHT_MULTIPLIER = .875; .0835

// Camera Configuration Parameters
const double CALIBRATION_DISTANCE = 27.64; //string len. in inches div. 2 (hypotenuse)
//const double LEFT_BASE_ANGLE = 0, RIGHT_BASE_ANGLE = 0; // degrees

// Camera Calibration Parameters
// using T bracket
const double LEFT_SEPARATION = 10.5;  //inches
const double RIGHT_SEPARATION = 11.5; //inches
const double TARGET_DISTANCE = 29;    //inches perpendicular
const bool USE_T_CALIBRATION = true;
//const double CAMERA_SEPARATION = LEFT_SEPARATION + RIGHT_SEPARATION; // inches

//OpenCV camera calc parameters
const int OPENCV_WIDTH = 640, OPENCV_HEIGHT = 480;

const double FOV_RADIANS = (LEFT_MULTIPLIER + RIGHT_MULTIPLIER)/2 * OPENCV_WIDTH * (CV_PI/180);

// Output stream parameters
const int STREAM_WIDTH = 640 * 2, STREAM_HEIGHT = 480, FRAMERATE = 15, BITRATE = 600000, PORT = 5001;

const cv::Scalar MIN_HSV(55, 80, 90);
const cv::Scalar MAX_HSV(255, 255, 255);

const double MIN_ASPECT_RATIO = 0.2, MAX_ASPECT_RATIO = 0.6, MIN_AREA_RATIO = 0.5, MIN_AREA = 25, MIN_OFFSET = 3, MAX_OFFSET = 20;

const cv::Scalar BLACK(0, 0, 0);
const cv::Scalar RED(0, 0, 255);
const cv::Scalar PINK(255, 0, 255);
const cv::Scalar BLUE(255, 0, 0);

#endif
