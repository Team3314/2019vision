
#ifndef CAMERA_INFO_HPP
#define CAMERA_INFO_HPP

#include <string.h>

class CameraInfo
{
    public:
        int RawWidth;
        int RawHeight;
        int ImageWidth;
        int ImageHeight;
        bool Transpose;
        int  FlipMode;
        double HorizViewAngle;
        double VertViewAngle;
        double HorizMountAngle;
        double VertMountAngle;
        int MaxGain;
        int MinGain;
        int MinIspGain;
        int MaxIspGain;
        long MinExposure;
        long MaxExposure;
        int Framerate;
        int WarmupDelay;

    public: 
    CameraInfo();
    virtual std::string GetPipeline();

};

#endif
