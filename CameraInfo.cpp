#include "CameraInfo.hpp"

CameraInfo::CameraInfo() 
{
    RawWidth = 640;
    RawHeight = 480;
    ImageWidth = RawWidth;
    ImageHeight = RawHeight;
    Transpose = false;
    FlipMode = 0;
    HorizViewAngle = 0;
    VertViewAngle = 0;
    HorizMountAngle = 0;
    VertMountAngle = 0;
    MinGain = 0;
    MaxGain = 0;
    MinIspGain = 0;
    MaxIspGain = 0;
    MinExposure = 0;
    MaxExposure = 0;
    Framerate = 30;
    WarmupDelay = 0;
}    
