#include "fall_detection.h"


struct fallState fallDetectionContext={0};


int isSkeletonStanding(struct fallState * fs)
{
  //If we are standing we have a large bbox on the Y axis ..!

  if (fs->bboxCurrent.y>200)
  {
    return 1;
  }
  return 0;
}



int logSkeletonState(struct fallState * fs)
{

  if (isSkeletonStanding(fs)) { fs->state=FALL_DETECTION_SKELETON_STANDING; }



}

int userIsRecent(struct fallState * fs,unsigned int timeStamp)
{
  if (ABSDIFF(fs->jointsTimestamp,timeStamp)>20) { return 0; }
  return 1;
}


int userHasFallen(struct fallState * fs,unsigned int timeStamp)
{
  if (!userIsRecent(fs,timeStamp)) { return 0; }
  return 0;
}


int userIsStanding(struct fallState * fs,unsigned int timeStamp)
{
  if (!userIsRecent(fs,timeStamp)) { return 0; }
  return (!userHasFallen(fs,timeStamp));
}
