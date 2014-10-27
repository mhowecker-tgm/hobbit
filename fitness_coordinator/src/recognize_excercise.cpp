#include "recognize_excercise.h"


#include <stdio.h>
#include <stdlib.h>

int learnExcercise=0;

struct excerciseDeclaration activeExcercise={0};
struct skeletonJointsWeCareAbout usedJointsByExcercise={0};

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
