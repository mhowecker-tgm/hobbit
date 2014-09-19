#include "fall_detection.h"


struct fallState fallDetectionContext={0};


int isSkeletonStanding(struct fallState * fs)
{
  //If we are standing we have at least 1 particle high on the Y axis of the frame..!
  unsigned int i=0;

  for (i=0; i<fs->numberOfJoints; i++)
  {
    if (fs->lastJoint2D[i].y<150)
    {
      return 1;
    }
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

int userIsStanding(struct fallState * fs,unsigned int timeStamp)
{
  if (!userIsRecent(fs,timeStamp)) { return 0; }
  if (isSkeletonStanding(fs)) { return 1; }
  return 0;
}


int userHasFallen(struct fallState * fs,unsigned int timeStamp)
{
  if (!userIsRecent(fs,timeStamp)) { return 0; }
  return 0;
}

