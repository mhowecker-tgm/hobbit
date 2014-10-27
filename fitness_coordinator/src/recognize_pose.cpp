#include "recognize_pose.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


struct skeletonHuman rememberedSkeletons[MAX_REMEMBERED_SKELETON_CONFIGS]={0};
unsigned int currentRememberedSkeletons=0;

int saveRememberedSkeletons(const char * filename)
{
  FILE * fp = fopen(filename,"wb");
  if (fp!=0)
  {
   fwrite((const void*) & currentRememberedSkeletons,sizeof(unsigned int),1,fp);

   unsigned int i=0;
   for (i=0; i<currentRememberedSkeletons; i++)
    {
     fwrite((const void*) & rememberedSkeletons[i],sizeof(struct skeletonHuman),1,fp);
    }

   fclose(fp);
   return 1;
  }
 return 0;
}

int loadRememberedSkeletons(const char * filename)
{
  currentRememberedSkeletons=0; //Initial state

  FILE * fp = fopen(filename,"rb");
  if (fp!=0)
  {
   fread((void*) & currentRememberedSkeletons,sizeof(unsigned int),1,fp);

   unsigned int i=0;
   for (i=0; i<currentRememberedSkeletons; i++)
    {
     fread((void*) & rememberedSkeletons[i],sizeof(struct skeletonHuman),1,fp);
    }

   fclose(fp);
   return 1;
  }
 return 0;
}



int rememberSkeleton(struct skeletonHuman * poseToRemember)
{
  fprintf(stderr,"\n Remembering a new skeleton ( %u )\n",currentRememberedSkeletons);
  unsigned int i=0;
  for (i=0; i<HUMAN_SKELETON_PARTS; i++)
  {
    rememberedSkeletons[currentRememberedSkeletons].joint[i].x = poseToRemember->joint[i].x;
    rememberedSkeletons[currentRememberedSkeletons].joint[i].y = poseToRemember->joint[i].y;
    rememberedSkeletons[currentRememberedSkeletons].joint[i].z = poseToRemember->joint[i].z;
  }
  ++currentRememberedSkeletons;
  return 1;
}


double getJointDistance(struct skeletonHuman * pose1 , struct skeletonHuman * pose2 , unsigned int jointID)
{
 double pAX = pose1->joint[jointID].x - pose1->joint[HUMAN_SKELETON_TORSO].x;
 double pAY = pose1->joint[jointID].y - pose1->joint[HUMAN_SKELETON_TORSO].y;
 double pAZ = pose1->joint[jointID].z - pose1->joint[HUMAN_SKELETON_TORSO].z;

 double pBX = pose2->joint[jointID].x - pose2->joint[HUMAN_SKELETON_TORSO].x;
 double pBY = pose2->joint[jointID].y - pose2->joint[HUMAN_SKELETON_TORSO].y;
 double pBZ = pose2->joint[jointID].z - pose2->joint[HUMAN_SKELETON_TORSO].z;


 double pX_AMinB = pAX-pBX;
 double pY_AMinB = pAY-pBY;
 double pZ_AMinB = pAZ-pBZ;

 double distance = sqrt( ( (pX_AMinB)*(pX_AMinB) ) +
                         ( (pY_AMinB)*(pY_AMinB) ) +
                         ( (pZ_AMinB)*(pZ_AMinB) ) );

 return distance;
}


double comparePoses(struct skeletonHuman * pose1 , struct skeletonHuman * pose2)
{
  double score=0.0;
  unsigned int i=0;
  for (i=0; i<HUMAN_SKELETON_PARTS; i++)
  {
    score+= getJointDistance(pose1,pose2,i);
  }
  return score;
}


unsigned int getPoseState(unsigned int * leftState , unsigned int * rightState , struct skeletonHuman * skeleton)
{
  *leftState=0;
  *rightState=0;
}




unsigned int haveWeSeenThisPoseBefore(struct skeletonHuman * observedSkeleton , unsigned int * poseThatLooksMostLikeIt , double * resultScore)
{
  if (currentRememberedSkeletons==0)
     { return 0; }


  unsigned int found=1;
  double bestScore=1000000 , currentScore = 0;

  unsigned int i=0;
  for (i=0; i<currentRememberedSkeletons; i++)
  {
    currentScore = comparePoses( observedSkeleton , &rememberedSkeletons[i] );
    //fprintf(stderr,"Observation vs %u is %0.2f \n",i,currentScore);
    if (currentScore<bestScore)
    {
      *poseThatLooksMostLikeIt = i;
      bestScore=currentScore;
    }
  }

  *resultScore = bestScore;
  return found;
}
