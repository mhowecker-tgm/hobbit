#ifndef HOBBIT_UPPERBODY_TRACKER_LIB_H_INCLUDED
#define HOBBIT_UPPERBODY_TRACKER_LIB_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif


enum calibIntrinsicsHUBT
{
  HUBT_CALIB_INTR_FX = 0 ,
  HUBT_CALIB_INTR_FY = 4 ,
  HUBT_CALIB_INTR_CX = 2 ,
  HUBT_CALIB_INTR_CY = 5
};

struct calibrationHUBT
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





static const char * jointNames[] =
{
 "head",
 "neck" ,
 "torso",
 "left_shoulder",
 "right_shoulder",
 "left_elbow",
 "right_elbow",
 "left_hand",
 "right_hand",
 "left_hip",
 "right_hip",
 "left_knee",
 "right_knee",
 "left_foot",
 "right_foot" ,
 "hip" ,
 //=================
 "End of Joint Names"
};


static const char * tgbtNames[] =
{
 "head",
 "neck",
 "bodyCenter",
 "rightShoulder",
 "leftShoulder",
 "rightElbow",
 "leftElbow",
 "rightWrist",
 "leftWrist",
 "rightLegRoot",
 "leftLegRoot",
 "rightKnee",
 "leftKnee",
 "rightAnkle",
 "leftAnkle",
 "hip",
 //--------------------
 "End Of TGBT Names"
};

static const char * const humanSkeletonJointNames[] =
    {
       "HUMAN_SKELETON_HEAD",
       "HUMAN_SKELETON_NECK",
       "HUMAN_SKELETON_TORSO",
       "HUMAN_SKELETON_RIGHT_SHOULDER",
       "HUMAN_SKELETON_LEFT_SHOULDER",
       "HUMAN_SKELETON_RIGHT_ELBOW",
       "HUMAN_SKELETON_LEFT_ELBOW",
       "HUMAN_SKELETON_RIGHT_HAND",
       "HUMAN_SKELETON_LEFT_HAND",
       "HUMAN_SKELETON_RIGHT_HIP",
       "HUMAN_SKELETON_LEFT_HIP",
       "HUMAN_SKELETON_RIGHT_KNEE",
       "HUMAN_SKELETON_LEFT_KNEE",
       "HUMAN_SKELETON_RIGHT_FOOT",
       "HUMAN_SKELETON_LEFT_FOOT",
       "HUMAN_SKELETON_HIP"
    };

enum humanSkeletonJoints
{
   HUMAN_SKELETON_HEAD = 0,
   HUMAN_SKELETON_NECK,
   HUMAN_SKELETON_TORSO,
   HUMAN_SKELETON_RIGHT_SHOULDER,
   HUMAN_SKELETON_LEFT_SHOULDER,
   HUMAN_SKELETON_RIGHT_ELBOW,
   HUMAN_SKELETON_LEFT_ELBOW,
   HUMAN_SKELETON_RIGHT_HAND,
   HUMAN_SKELETON_LEFT_HAND,
   HUMAN_SKELETON_RIGHT_HIP,
   HUMAN_SKELETON_LEFT_HIP,
   HUMAN_SKELETON_RIGHT_KNEE,
   HUMAN_SKELETON_LEFT_KNEE,
   HUMAN_SKELETON_RIGHT_FOOT,
   HUMAN_SKELETON_LEFT_FOOT,
   HUMAN_SKELETON_HIP,
   //---------------------
   HUMAN_SKELETON_PARTS
};

static const char * const humanSkeletonMirroredJointNames[] =
    {
      "HUMAN_SKELETON_MIRRORED_HEAD",
      "HUMAN_SKELETON_MIRRORED_NECK",
      "HUMAN_SKELETON_MIRRORED_TORSO",
      "HUMAN_SKELETON_MIRRORED_LEFT_SHOULDER",
      "HUMAN_SKELETON_MIRRORED_RIGHT_SHOULDER",
      "HUMAN_SKELETON_MIRRORED_LEFT_ELBOW",
      "HUMAN_SKELETON_MIRRORED_RIGHT_ELBOW",
      "HUMAN_SKELETON_MIRRORED_LEFT_HAND",
      "HUMAN_SKELETON_MIRRORED_RIGHT_HAND",
      "HUMAN_SKELETON_MIRRORED_LEFT_HIP",
      "HUMAN_SKELETON_MIRRORED_RIGHT_HIP",
      "HUMAN_SKELETON_MIRRORED_LEFT_KNEE",
      "HUMAN_SKELETON_MIRRORED_RIGHT_KNEE",
      "HUMAN_SKELETON_MIRRORED_LEFT_FOOT",
      "HUMAN_SKELETON_MIRRORED_RIGHT_FOOT",
      "HUMAN_SKELETON_MIRRORED_HIP"
    };

enum humanMirroredSkeletonJoints
{
   HUMAN_SKELETON_MIRRORED_HEAD = 0,
   HUMAN_SKELETON_MIRRORED_NECK,
   HUMAN_SKELETON_MIRRORED_TORSO,
   HUMAN_SKELETON_MIRRORED_LEFT_SHOULDER,
   HUMAN_SKELETON_MIRRORED_RIGHT_SHOULDER,
   HUMAN_SKELETON_MIRRORED_LEFT_ELBOW,
   HUMAN_SKELETON_MIRRORED_RIGHT_ELBOW,
   HUMAN_SKELETON_MIRRORED_LEFT_HAND,
   HUMAN_SKELETON_MIRRORED_RIGHT_HAND,
   HUMAN_SKELETON_MIRRORED_LEFT_HIP,
   HUMAN_SKELETON_MIRRORED_RIGHT_HIP,
   HUMAN_SKELETON_MIRRORED_LEFT_KNEE,
   HUMAN_SKELETON_MIRRORED_RIGHT_KNEE,
   HUMAN_SKELETON_MIRRORED_LEFT_FOOT,
   HUMAN_SKELETON_MIRRORED_RIGHT_FOOT,
   HUMAN_SKELETON_MIRRORED_HIP,
   //---------------------
   HUMAN_SKELETON_MIRRORED_PARTS
};



enum processingModeENUM
{
   PROCESSING_MODE_UPPER_GESTURE_BODY_TRACKER = 0,
   PROCESSING_MODE_SIMPLE_PERSON_DETECTOR ,
   PROCESSING_MODE_TREE_GRID_BODY_TRACKER
};

extern const char * jointNames[];

struct point2D
{
    float x,y;
};

struct point3D
{
    float x,y,z;
};

struct skeletonHuman
{
  unsigned int observationNumber , observationTotal;
  unsigned int userID;

  unsigned char isNew,isVisible,isOutOfScene,isLost;
  unsigned char statusCalibrating,statusStoppedTracking, statusTracking,statusFailed;

  struct point3D bbox[8];
  struct point3D centerOfMass;
  struct point3D joint[HUMAN_SKELETON_PARTS];
  struct point2D joint2D[HUMAN_SKELETON_PARTS];
  float jointAccuracy[HUMAN_SKELETON_PARTS];
};


struct skeletonPointing
{
  struct point3D pointStart;
  struct point3D pointEnd;
  struct point3D pointingVector;
  unsigned char isLeftHand;
  unsigned char isRightHand;
};



int hobbitUpperBodyTracker_setVisualization(int doVis);
int hobbitUpperBodyTracker_setDumpToFiles(int doDump);

int hobbitUpperBodyTracker_Initialize(unsigned int targetWidth,unsigned int targetHeight);

int hobbitUpperBodyTracker_Close();
void hobbitUpperBodyTracker_NewSkeletonDetectedEvent(unsigned int frameNumber,void * skeleton );
int hobbitUpperBodyTracker_RegisterSkeletonDetectedEvent(void * callback);

int hobbitUpperBodyTracker_NewFrame(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                    unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                    struct calibrationHUBT * frameCalibration ,
                                    unsigned int processingMode ,
                                    unsigned int frameTimestamp );

void hobbitUpperBodyTracker_Clear();

#ifdef __cplusplus
}
#endif


#endif
