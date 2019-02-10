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

const int DEVICE = 1, WIDTH = 640, HEIGHT = 480, FRAMERATE = 15, BITRATE = 600000, PORT = 5001;
const std::string IP = "192.168.1.3";

const cv::Scalar MIN_HSV(55, 80, 90);
const cv::Scalar MAX_HSV(255, 255, 255);

const double MIN_ASPECT_RATIO = 0.2, MAX_ASPECT_RATIO = 0.6, MIN_AREA_RATIO = 0.5, MIN_AREA = 25, MIN_OFFSET = 3, MAX_OFFSET = 20;

const cv::Scalar BLACK(0, 0, 0);
const cv::Scalar RED(0, 0, 255);
const cv::Scalar PINK(255, 0, 255);
const cv::Scalar BLUE(255, 0, 0);

#endif