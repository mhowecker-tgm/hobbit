#ifndef RECOGNIZE_EXCERCISE_H_INCLUDED
#define RECOGNIZE_EXCERCISE_H_INCLUDED


#include "recognize_pose.h"

#define MAX_FILENAME 256 //Changing this will potentially fuck up the binary file ..!

struct excerciseDeclaration
{
  unsigned int numberOfStates;
  unsigned int totalStates;

  char skeletonSetFilename[MAX_FILENAME];
  unsigned int stateTree[MAX_REMEMBERED_SKELETON_CONFIGS];
  unsigned int stateTreeDuration[MAX_REMEMBERED_SKELETON_CONFIGS];

  unsigned int acceptedStates[MAX_REMEMBERED_SKELETON_CONFIGS];
};

extern int learnExcercise;
extern struct excerciseDeclaration activeExcercise;
extern struct skeletonJointsWeCareAbout usedJointsByExcercise;

int loadExercise(const char * filename);

int addToStateTree(struct fitnessState * state , struct skeletonHuman * skeleton , unsigned int pose , struct excerciseDeclaration * ed);


#endif
