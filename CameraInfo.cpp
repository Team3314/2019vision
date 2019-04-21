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

    public CameraInfo() 
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
    }    
}