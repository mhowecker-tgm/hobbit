#ifndef OPENNI2ACQUISITION_H_INCLUDED
#define OPENNI2ACQUISITION_H_INCLUDED


#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief A short hand  enumerator to make it easy to access the double intrinsic[9]; field of a struct calibration
 *
 * This enumerator holds all the possible values passed when and where moduleID is requested
 * These have an 1:1 relation to the modules that are supported and are used to reference them
 *
 */
enum calibIntrinsics
{
  CALIB_INTR_FX = 0 ,
  CALIB_INTR_FY = 4 ,
  CALIB_INTR_CX = 2 ,
  CALIB_INTR_CY = 5
};

/**
 * @brief The structure that holds the calibration data
 *
 *  The calibration structure covers everything from intrinsics / distortion  / dimensions to
 *  the extrinsic calibration of the input source , this gets set by each of the modules and is one of the
 *  core structures used in most things in RGBDAcquisition
 */
struct calibration
{
  /* CAMERA INTRINSIC PARAMETERS */
  char intrinsicParametersSet;
  double intrinsic[9];
  double k1,k2,p1,p2,k3;

  /* CAMERA EXTRINSIC PARAMETERS */
  char extrinsicParametersSet;
  double extrinsicRotationRodriguez[3];
  double extrinsicTranslation[3];
  double extrinsic[16];

  /*CAMERA DIMENSIONS ( WHEN RENDERING )*/
  double nearPlane,farPlane;
  unsigned int width;
  unsigned int height;

  double depthUnit;

  /*CONFIGURATION*/
  int imagesUsed;
  int boardWidth;
  int boardHeight;
  double squareSize;
};


    #define USE_CALIBRATION 1

   //Initialization of OpenNI2
   int startOpenNI2Module(unsigned int max_devs,char * settings);

   int getOpenNI2NumberOfDevices(); // This has to be called AFTER startOpenNI2
   int stopOpenNI2Module();

   int mapOpenNI2DepthToRGB(int devID);
   int mapOpenNI2RGBToDepth(int devID);

   //Basic Per Device Operations
   int createOpenNI2Device(int devID,char * devName,unsigned int width,unsigned int height,unsigned int framerate);
   int destroyOpenNI2Device(int devID);
   int snapOpenNI2Frames(int devID);

   int getTotalOpenNI2FrameNumber(int devID);
   int getCurrentOpenNI2FrameNumber(int devID);

   //Color Frame getters
   int getOpenNI2ColorWidth(int devID);
   int getOpenNI2ColorHeight(int devID);
   int getOpenNI2ColorDataSize(int devID);
   int getOpenNI2ColorChannels(int devID);
   int getOpenNI2ColorBitsPerPixel(int devID);
   unsigned char * getOpenNI2ColorPixels(int devID);

   double getOpenNI2ColorFocalLength(int devID);
   double getOpenNI2ColorPixelSize(int devID);

   //Depth Frame getters
   int getOpenNI2DepthWidth(int devID);
   int getOpenNI2DepthHeight(int devID);
   int getOpenNI2DepthDataSize(int devID);
   int getOpenNI2DepthChannels(int devID);
   int getOpenNI2DepthBitsPerPixel(int devID);
   unsigned short * getOpenNI2DepthPixels(int devID);


   double getOpenNI2DepthFocalLength(int devID);
   double getOpenNI2DepthPixelSize(int devID);

    int getOpenNI2ColorCalibration(int devID,struct calibration * calib);
    int getOpenNI2DepthCalibration(int devID,struct calibration * calib);

    int setOpenNI2ColorCalibration(int devID,struct calibration * calib);
    int setOpenNI2DepthCalibration(int devID,struct calibration * calib);


    extern unsigned int pauseFaceDetection;
    extern unsigned int faceDetectionFramesBetweenScans;
    extern unsigned int pauseSkeletonDetection;
    extern unsigned int skeletonDetectionFramesBetweenScans;


#ifdef __cplusplus
}
#endif

#endif // OPENNI2ACQUISITION_H_INCLUDED
