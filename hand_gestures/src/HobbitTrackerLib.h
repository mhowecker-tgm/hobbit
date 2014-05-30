#ifndef HOBBITTRACKERLIB_H_INCLUDED
#define HOBBITTRACKERLIB_H_INCLUDED


#ifdef __cplusplus
extern "C"
{
#endif

enum recognizedGestures
{
    GESTURE_NONE=0,
    GESTURE_CANCEL ,
    GESTURE_HELP ,
    GESTURE_YES  ,
    GESTURE_NO   ,
    GESTURE_REWARD   ,
    GESTURE_POINT ,
    //----------------
    POSSIBLE_GESTURES
};

extern const char * gestureNames[];

struct handGesture
{
  unsigned int gestureID;

  float x,y,z,theta;
  float toX,toY,toZ;

  unsigned int x2D,y2D;

  unsigned int frameTimestamp;
};


enum calibIntrinsicsHT
{
  HT_CALIB_INTR_FX = 0 ,
  HT_CALIB_INTR_FY = 4 ,
  HT_CALIB_INTR_CX = 2 ,
  HT_CALIB_INTR_CY = 5
};

struct calibrationHT
{
  // CAMERA INTRINSIC PARAMETERS
  char intrinsicParametersSet;
  double intrinsic[9];
  double k1,k2,p1,p2,k3;

  // CAMERA EXTRINSIC PARAMETERS
  char extrinsicParametersSet;
  double extrinsicRotationRodriguez[3];
  double extrinsicTranslation[3];
  double extrinsic[16];

  //CAMERA DIMENSIONS ( WHEN RENDERING )
  double nearPlane,farPlane;
  unsigned int width;
  unsigned int height;

  double depthUnit;

  //CONFIGURATION
  int imagesUsed;
  int boardWidth;
  int boardHeight;
  double squareSize;
};

extern unsigned char doCVOutput;
extern unsigned char doCalibrationOutput;

int hobbitGestures_Initialize(unsigned int targetWidth , unsigned int targetHeight);
int hobbitGestures_Close();

int hobbitGestures_NewFrame(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                            unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                            struct calibrationHT * frameCalibration ,
                            unsigned int frameTimestamp );


int hobbitGestures_RegisterGestureDetectedEvent(void * callback);


#ifdef __cplusplus
}
#endif

#endif // HOBBITTRACKERLIB_H_INCLUDED
