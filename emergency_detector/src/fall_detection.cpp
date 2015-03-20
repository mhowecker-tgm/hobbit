#include "fall_detection.h"
#include "visualization.h"
#include <stdio.h>
#include <math.h>

struct fallState fallDetectionContext={0};


int userIsRecent(struct fallState * fs,unsigned int timeStamp)
{
  if (ABSDIFF(fs->jointsTimestamp,timeStamp)>30) { return 0; }
  return 1;
}

int isSkeletonWrong(struct fallState * fs,unsigned int frameTimestamp)
{
  if (!userIsRecent(fs,frameTimestamp)) { return 0; }

  int i=0;
  float totalDist=0;
  float thisDist=0;
  signed int dX,dY;
   for (i=0; i<fs->numberOfJoints; i++)
   {

    if ( (fs->lastJoint2D[i].y!=0)&& (fs->lastJoint2D[i].x!=0) )
     {
        dX = fs->lastJoint2D[i].x - fs->lastJoint2D[0].x;
        dY = fs->lastJoint2D[i].y - fs->lastJoint2D[0].y;

        thisDist=sqrt((dX*dX) + (dY*dY));
        totalDist+=thisDist;
     }
   }

  if (totalDist>0) { fprintf(stderr,"Skeleton Spread is %0.2f \n",totalDist); }
  return 0;
}



int isSkeletonStanding(struct fallState * fs)
{
 // if (isSkeletonTooOld(fs,frameTimestamp)) { return 0; }

  //If we are standing we have at least 1 particle high on the Y axis of the frame..!
  unsigned int i=0;
  unsigned int height=0;

  if (fs->headLookingDirection==HEAD_LOOKING_CENTER)      { height=200; } else
  if (fs->headLookingDirection==HEAD_LOOKING_LITTLE_DOWN) { height=150;  } else
  if (fs->headLookingDirection==HEAD_LOOKING_DOWN)        { height=100; }

   for (i=0; i<fs->numberOfJoints; i++)
   {
    if ( (fs->lastJoint2D[i].y!=0)&& (fs->lastJoint2D[i].y<height) )
     {
       return 1;
    }
   }
  return 0;
}



int isSkeletonFallenAndFar(struct fallState * fs)
{
  //If we are standing we have at least 1 particle high on the Y axis of the frame..!
  unsigned int i=0;
  unsigned int height=6660;

  if (fs->headLookingDirection==HEAD_LOOKING_CENTER)      { height=330; } else
  if (fs->headLookingDirection==HEAD_LOOKING_LITTLE_DOWN) { height=260; } else
  if (fs->headLookingDirection==HEAD_LOOKING_DOWN)        { height=130; }

  lowBorderY = height; //Update Visualization Coords

  unsigned int lowPoints = 0;
  unsigned int highPoints = 0;
  unsigned int farPoints = 0;
  unsigned int closePoints = 0;


   for (i=0; i<fs->numberOfJoints; i++)
   {
    if (fs->lastJoint3D[i].z>farBorderZ)      { ++farPoints;  }
    if (fs->lastJoint3D[i].z<closeBorderZ)      { ++closePoints;  }
    if ( (fs->lastJoint2D[i].y!=0)&& (fs->lastJoint2D[i].y>=height) )    { ++lowPoints;  } else
    if ( (fs->lastJoint2D[i].y!=0)&& (fs->lastJoint2D[i].y<height) )     { ++highPoints; }
   }

  fprintf(stderr,"Skeleton : Border @ %u , High : %u / Low : %u / Far : %u / Close %u \n",height,highPoints,lowPoints,farPoints,closePoints);

  if ( (highPoints==0) && (lowPoints>0) && (farPoints==0) && (closePoints==0) ) { return 1; }
  return 0;
}


int logSkeletonState(struct fallState * fs,unsigned int mode3D)
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

  //if (isSkeletonStanding(fs)) { fprintf(stderr,"Skeleton Is Standing\n");  fs->state=FALL_DETECTION_SKELETON_STANDING; return 1; }


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
  if (isSkeletonFallenAndFar(fs)) { return 1; }
  return 0;
}

