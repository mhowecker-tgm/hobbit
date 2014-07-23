
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


#define HUMAN_SKELETON_PARTS 15
static const char * jointNames[] =
{"head",
 "neck",
 "torso",
 "left_shoulder",
 "right_shoulder",
 "left_elbow",
 "right_elbow",
 "left_hand",
 "right_hand",
 "left_hip",
 "right_hip",
 "left_knee",
 "right_knee",
 "left_foot",
 "right_foot"
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
