
#ifndef FALL_DETECTION_H_INCLUDED
#define FALL_DETECTION_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif


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
   unsigned int state;
   struct floatTriplet bboxLast,bboxDelta,bboxCurrent,bboxAverage;
   struct floatTriplet posLast,posDelta,posCurrent,posAverage;


   unsigned int jointsTimestamp;
   unsigned int numberOfJoints;
   struct floatTriplet lastJoint2D[MAX_NUMBER_OF_2D_JOINTS];

};


extern struct fallState fallDetectionContext;

int userIsRecent(struct fallState * fs,unsigned int timeStamp);
int userIsStanding(struct fallState * fs,unsigned int timeStamp);
int userHasFallen(struct fallState * fs,unsigned int timeStamp);

#ifdef __cplusplus
}
#endif


#endif
