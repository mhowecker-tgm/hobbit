#include "fall_detection.h"
#include <stdio.h>
#include <math.h>

struct fallState fallDetectionContext={0};


int isSkeletonStanding(struct fallState * fs)
{
  //If we are standing we have at least 1 particle high on the Y axis of the frame..!
  unsigned int i=0;

  for (i=0; i<fs->numberOfJoints; i++)
  {
    if ( (fs->lastJoint2D[i].y!=0)&& (fs->lastJoint2D[i].y<200) )
    {
      //fprintf(stderr,"Joint #%u ( %0.2f,%0.2f ) is high",i,fs->lastJoint2D[i].x ,fs->lastJoint2D[i].y);
      return 1;
    }
  }
  return 0;
}



int logSkeletonState(struct fallState * fs)
{
  fs->skeletonVelocity=0;

  unsigned int timestampDiff = fs->jointsTimestamp-fs->lastJointsTimestamp;
  if ( timestampDiff > 30 )
    {
      fprintf(stderr,"Cannot track skeleton movement , samples have a big delay ( this %u , last %u , diff %u  )" , fs->jointsTimestamp , fs->lastJointsTimestamp , timestampDiff);
      return 0;
    }

  unsigned int i=0;
  for (i=0; i<fs->numberOfJoints; i++)
  {
      if (
           (fs->lastJoint2D[i].x==0.0) ||
           (fs->lastJoint2D[i].y==0.0) ||
           (fs->currentJoint2D[i].y==0.0) ||
           (fs->currentJoint2D[i].y==0.0)
          )
      {

      } else
      {
        float distX = (float) fs->lastJoint2D[i].x-fs->currentJoint2D[i].x;
        float distY = (float) fs->lastJoint2D[i].y-fs->currentJoint2D[i].y;
        unsigned int displacement = (unsigned int ) sqrt( (distX*distX) + (distY*distY) )  ;
        fs->skeletonVelocity+=displacement;
      }
  }

  if (isSkeletonStanding(fs)) { fprintf(stderr,"Skeleton Is Standing");  fs->state=FALL_DETECTION_SKELETON_STANDING; return 1; }


  return 1;
}

int userIsRecent(struct fallState * fs,unsigned int timeStamp)
{
  if (ABSDIFF(fs->jointsTimestamp,timeStamp)>30) { return 0; }
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

