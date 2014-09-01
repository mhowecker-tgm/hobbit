#include "recognize_pose.h"




double getJointDistance(struct skeletonHuman * pose1 , struct skeletonHuman * pose2 , unsigned int jointID)
{
 double pAX = pose1->joint[jointID].x - pose1->joint[HUMAN_SKELETON_TORSO].x;
 double pAY = pose1->joint[jointID].y - pose1->joint[HUMAN_SKELETON_TORSO].y;
 double pAZ = pose1->joint[jointID].z - pose1->joint[HUMAN_SKELETON_TORSO].z;

 double pBX = pose2->joint[jointID].x - pose2->joint[HUMAN_SKELETON_TORSO].x;
 double pBY = pose2->joint[jointID].y - pose2->joint[HUMAN_SKELETON_TORSO].y;
 double pBZ = pose2->joint[jointID].z - pose2->joint[HUMAN_SKELETON_TORSO].z;

 double distance = sqrt( ( (pAX-pBX)*(pAX-pBX) ) +
                         ( (pAY-pBY)*(pAY-pBY) ) +
                         ( (pAZ-pBZ)*(pAZ-pBZ) ) );

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

