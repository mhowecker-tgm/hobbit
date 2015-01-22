
#ifndef FALL_DETECTION_H_INCLUDED
#define FALL_DETECTION_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif


enum HEAD_LOOK_DIRECTION_ENUM
{
   HEAD_UNKNOWN_DIRECTION = 0,
   HEAD_LOOKING_DOWN     ,
   HEAD_LOOKING_CENTER  ,
   HEAD_LOOKING_UP
};


#define ABSDIFF(num1,num2) ( (num1-num2) >=0 ? (num1-num2) : (num2 - num1) )

#define MAX_NUMBER_OF_2D_JOINTS 30

enum FALL_DETECTION_STATE_ENUM
{
    FALL_DETECTION_LOST_SKELETON = 0 ,
    FALL_DETECTION_SKELETON_STANDING ,
    FALL_DETECTION_SKELETON_FALLING ,
    FALL_DETECTION_SKELETON_DOWN ,
    //------------------------------------
    FALL_DETECTION_NUMBER_OF_STATES
};

struct floatTriplet
{
  float x,y,z;
};


struct fallState
{
   unsigned int headLookingDirection;

   unsigned int state;
   struct floatTriplet bboxLast,bboxDelta,bboxCurrent,bboxAverage;
   struct floatTriplet posLast,posDelta,posCurrent,posAverage;

   unsigned int skeletonVelocity;
   unsigned int jointsTimestamp;
   unsigned int lastJointsTimestamp;
   unsigned int numberOfJoints;
   struct floatTriplet currentJoint2D[MAX_NUMBER_OF_2D_JOINTS];
   struct floatTriplet lastJoint2D[MAX_NUMBER_OF_2D_JOINTS];


   struct floatTriplet currentJoint3D[MAX_NUMBER_OF_2D_JOINTS];
   struct floatTriplet lastJoint3D[MAX_NUMBER_OF_2D_JOINTS];
};

int logSkeletonState(struct fallState * fs,unsigned int mode3D);

extern struct fallState fallDetectionContext;

int isSkeletonWrong(struct fallState * fs);

int userIsRecent(struct fallState * fs,unsigned int timeStamp);
int userIsStanding(struct fallState * fs,unsigned int timeStamp);
int userHasFallen(struct fallState * fs,unsigned int timeStamp);

#ifdef __cplusplus
}
#endif


#endif
