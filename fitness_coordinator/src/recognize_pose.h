#ifndef RECOGNIZE_POSE_H_INCLUDED
#define RECOGNIZE_POSE_H_INCLUDED

/*                      _____
                      /  o   \
                      \      /
                         O
                     ___ ____
o- - - -  0 - - - -  |       |  - - - -  0 - - - - o
                     |       |
                     |   O   |
                     |_______|
                      o     o
                      |     |
                      |     |
                      |     |
                      o     o
                      |     |
                      |     |
                      |     |
                      o     o
*/


enum humanSkeletonJoints
{
   POSE_HAND_UP = 0,
   POSE_HAND_UP_MIDDLE ,
   POSE_HAND_MIDDLE ,
   POSE_HAND_DOWN_MIDDLE ,
   POSE_HAND_DOWN,
   //---------------------
   NUMBER_OF_DIFFERENT_POSES
};


struct fitnessState
{
    unsigned int started;
    unsigned int repetitions;
    unsigned int lastTransmittedRepetitions;
    unsigned int exercise;

};



static const char * jointNames[] =
{"head",
 "neck",
 "torso",
 "right_shoulder",
 "left_shoulder",
 "right_elbow",
 "left_elbow",
 "right_hand",
 "left_hand",
 "right_hip",
 "left_hip",
 "right_knee",
 "left_knee",
 "right_foot",
 "left_foot"
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
   //---------------------
   HUMAN_SKELETON_PARTS
};

struct point3D
{
    float x,y,z;
};

struct point2D
{
    float x,y;
};



struct skeletonHuman
{
  unsigned int observationNumber , observationTotal;
  unsigned int userID;
  struct point3D joint[HUMAN_SKELETON_PARTS];
};


unsigned int getPoseState(unsigned int * leftState , unsigned int * rightState , struct skeletonHuman * skeleton);




#endif
