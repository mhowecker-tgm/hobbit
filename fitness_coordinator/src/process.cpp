#include "process.h"
#include <stdio.h>
#include <stdlib.h>


#define MAX_FILENAME 256 //Changing this will potentially fuck up the binary file ..!


int learnExcercise=0;

struct excerciseDeclaration
{
  unsigned int numberOfStates;
  unsigned int totalStates;

  char skeletonSetFilename[MAX_FILENAME];
  unsigned int stateTree[MAX_REMEMBERED_SKELETON_CONFIGS];
  unsigned int stateTreeDuration[MAX_REMEMBERED_SKELETON_CONFIGS];

  unsigned int acceptedStates[MAX_REMEMBERED_SKELETON_CONFIGS];
};


struct excerciseDeclaration activeExcercise={0};


int broadcastExcerciseGeneric(char * tag , char * name , char * value)
{
  char what2execute[MAX_CMD_STR]={0};
  snprintf(what2execute,MAX_CMD_STR,"rostopic pub /fitness hobbit_msgs/Fitness \"{command: '%s' , params: [ {name: '%s' , value: '%s'} ] }\" -1",tag,name,value);

    int i=system(what2execute);
    if (i!=0) { fprintf(stderr,"Command %s failed\n",what2execute); } else
              { fprintf(stderr,"Command %s success\n",what2execute); }

}

int broadcastExcerciseRepetition(unsigned int exerciseNumber,unsigned int repetitionNumber)
{
  char exerciseStr[MAX_NUM_STR]={0};
  snprintf(exerciseStr,MAX_NUM_STR,"%u",exerciseNumber);

  char repetitionStr[MAX_NUM_STR]={0};
  snprintf(repetitionStr,MAX_NUM_STR,"%u",repetitionNumber);

  return broadcastExcerciseGeneric((char*) "C_EXERCISE_REP",exerciseStr,repetitionStr);
}

int broadcastExcerciseStarted(char * name , char * value)
{
  return broadcastExcerciseGeneric((char*) "C_EXERCISE_STARTED",name,value);
}

int broadcastExcerciseFinished(char * name , char * value)
{
  return broadcastExcerciseGeneric((char*) "C_EXERCISE_FINISHED",name,value);
}

/* ============================= ============================= ============================= =============================  */

int signalRepetition(struct fitnessState * state)
{
  //This should be called after doing a ++state->repetitions;
  state->lastTransmittedRepetitions = state->repetitions;
  return broadcastExcerciseRepetition(state->exercise,state->repetitions);
}


int addToStateTree(struct fitnessState * state , struct skeletonHuman * skeleton , unsigned int pose , struct excerciseDeclaration * ed)
{
   if (ed->numberOfStates==0)
   { // Just add
     fprintf(stderr,"Initial State for Excercise \n");
     ed->stateTree[ed->numberOfStates]=pose;
     ++ed->numberOfStates;
   } else
   {
      if (ed->stateTree[ed->numberOfStates-1]==pose)
        {
          fprintf(stderr,"Lingering on Pose %u ( %u time )  \n",pose,ed->stateTreeDuration[ed->numberOfStates-1]);
          ++ed->stateTreeDuration[ed->numberOfStates-1];
         } else
        {
          fprintf(stderr,"Transitioning from Pose %u to %u \n",ed->stateTreeDuration[ed->numberOfStates-1],pose);
          ed->stateTree[ed->numberOfStates]=pose;
          ++ed->numberOfStates;
        }

   }
}


int checkSkeletonForRepetition(struct fitnessState * state , struct skeletonHuman * skeleton)
{
  //Check skeleton here
  unsigned int poseThatLooksMostLikeIt;
  double resultScore;
  if (haveWeSeenThisPoseBefore(skeleton , &poseThatLooksMostLikeIt , &resultScore))
  {
    fprintf(stderr,"Pose Looks like :  %u ( score %0.2f ) \n",poseThatLooksMostLikeIt,resultScore);

    if (learnExcercise)
    {
       addToStateTree(state ,skeleton , poseThatLooksMostLikeIt , &activeExcercise );
    }

  }

  // If skeleton is changed ++state->repetitions

  //------------------------------------------------------------
  // this should then broadcast the change
  if (state->lastTransmittedRepetitions != state->repetitions)
  {
    return signalRepetition(state);
  }

  return 0;
}

int stopExcercise(struct fitnessState * state)
{
  //TODO : This will get called when LUI will decide that we want to stop the excercise
  return 0;
}

int startExcercise(struct fitnessState * state)
{
  //TODO : This will get called when LUI will decide that we want to start an excercise
  return 0;
}
