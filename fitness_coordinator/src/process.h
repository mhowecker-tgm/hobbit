
#ifndef PROCESS_H_INCLUDED
#define PROCESS_H_INCLUDED

#define MAX_CMD_STR 1024
#define MAX_NUM_STR 128

struct fitnessState
{
    unsigned int started;
    unsigned int repetitions;
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


int broadcastExcerciseGeneric(char * tag , char * name , char * value);
int broadcastExcerciseRepetition(unsigned int exerciseNumber,unsigned int repetitionNumber);
int broadcastExcerciseStarted(char * name , char * value);
int broadcastExcerciseFinished(char * name , char * value);


int signalRepetition(struct fitnessState * state);
int checkSkeletonForRepetition(struct fitnessState * state , struct skeletonHuman * skeleton);
int stopExcercise(struct fitnessState * state);
int startExcercise(struct fitnessState * state);

#endif
