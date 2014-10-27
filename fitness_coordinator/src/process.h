
#ifndef PROCESS_H_INCLUDED
#define PROCESS_H_INCLUDED

#define MAX_CMD_STR 1024
#define MAX_NUM_STR 128

#include "recognize_pose.h"


int broadcastExcerciseGeneric(char * tag , char * name , char * value);
int broadcastExcerciseRepetition(unsigned int exerciseNumber,unsigned int repetitionNumber);
int broadcastExcerciseStarted(char * name , char * value);
int broadcastExcerciseFinished(char * name , char * value);


int signalRepetition(struct fitnessState * state);
int checkSkeletonForRepetition(struct fitnessState * state , struct skeletonHuman * skeleton);
int stopExcercise(struct fitnessState * state);
int startExcercise(struct fitnessState * state);

#endif
