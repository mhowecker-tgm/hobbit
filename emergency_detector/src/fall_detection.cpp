#include "fall_detection.h"


struct fallState fallDetectionContext={0};


int isSkeletonStanding(struct failState * fs)
{
  //If we are standing we have a large bbox on the Y axis ..!
  if (bboxCurrent.y>200)
  {
    return 1;
  }
  return 0;
}



int logSkeletonState(struct fallState * fs)
{

  if (isSkeletonStanding(fs)) { fs->state=FALL_DETECTION_SKELETON_STANDING; }



}



int userHasFallen(struct fallState * fs)
{
  return 0;
}

