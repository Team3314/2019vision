#ifndef RPV2_CAMERA_INFO_HPP
#define RPV2_CAMERA_INFO_HPP

#include <string.h>

class RPV2CameraInfo : Public CameraInfo
{
    public: 
    RPV2CameraInfo(width, height, framerate,
        horizView, vertView, horizAngle, vertAngle,
        minGain, maxGain, minIspGain, maxIspGain,
    	minExposure, maxExposure)
        : CameraInfo(), RawWidth(width), RawHeight(height), Framerate(framerate),
          Transpose(true), Flipmode(1), ImageWidth(height), ImageHeight(width),
          HorizViewAngle(horizView), VertViewAngle(vertView), HorizMountAngle(horizAngle), VertMountAngle(vertAngle),
          MinGain(minGain), MaxGain(maxGain), MinIspGain(minIspGain), MaxIspGain(maxIspGain), MinExposure(minExposure), MaxExposure(maxExposure),
		  WarmupDelay(0)
          ;

    std::string GetPipeline()
    {
    	char buff[500];
    	sprintf(buff,
    			"nvarguscamerasrc wbmode=0 awblock=true "
    				"gainrange=\"%d %d\" "
    				"ispdigitalgainrange=\"%d %d\" "
    				"exposuretimerange=\"%ld %ld\" "
    				"aelock=true "
    			"! video/x-raw(memory:NVMM), format=(string)NV12, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 "
    			"! nvvidconv "
    			"! video/x-raw,format=(string)BGRx "
    			"! videoconvert "
    			"! video/x-raw,format=(string)BGR "
    			"! appsink",
    			MinGain, MaxGain, MinIspGain, MaxIspGain,
    			MinExposure, MaxExposure,
    			RawWidth, RawHeight, Framerate);

    	std::string pipestring = buff;
    	//printf("write string: %s\n", pipstring.c_str());
    	return pipestring;
    };
};

#endif