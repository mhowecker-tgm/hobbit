#include "process.h"
#include <stdio.h>
#include <stdlib.h>

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

  return broadcastExcerciseGeneric("C_EXERCISE_REP",exerciseStr,repetitionStr);
}

int broadcastExcerciseStarted(char * name , char * value)
{
  return broadcastExcerciseGeneric("C_EXERCISE_STARTED",name,value);
}

int broadcastExcerciseFinished(char * name , char * value)
{
  return broadcastExcerciseGeneric("C_EXERCISE_FINISHED",name,value);
}

/* ============================= ============================= ============================= =============================  */

int signalRepetition(struct fitnessState * state)
{
  //This should be called after doing a ++state->repetitions;
  state->lastTransmittedRepetitions = state->repetitions;
  return broadcastExcerciseRepetition(state->exercise,state->repetitions);
}

int checkSkeletonForRepetition(struct fitnessState * state , struct skeletonHuman * skeleton)
{
  //Check skeleton here

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
