#ifndef HOBBIT_EXERCISE_DATA_STRUCTURES_H_INCLUDED
#define HOBBIT_EXERCISE_DATA_STRUCTURES_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif


static const char * exerciseNames[] =
{
 "None",
 "-",
 "Left Flap",
 "Left Double Flap",
 "-",
 "-",
 "Right Flap",
 "Right Double Flap",
 "-",
 "-" ,
 "Dual Flap",
 "Dual Elbow Bend",
 "-"
};


enum excerciseEnumerator
{
  NO_EXERCISE_ACTIVE = 0 ,
  //-----------------------
  MORE_IS_LEFT_EXERCISE ,
  LEFT_ARM_PUMP_EXERCISE,
  LEFT_FLAP_EXERCISE,
  LEFT_ELBOW_BEND_EXERCISE,
  LESS_IS_LEFT_EXERCISE ,
  //-----------------------
  MORE_IS_RIGHT_EXERCISE,
  RIGHT_ARM_PUMP_EXERCISE,
  RIGHT_FLAP_EXERCISE,
  RIGHT_ELBOW_BEND_EXERCISE,
  LESS_IS_RIGHT_EXERCISE ,
  //-----------------------
  MORE_IS_DUAL_EXERCISE,
  DUAL_FLAP_EXERCISE ,
  DUAL_ELBOW_BEND_SYNC_EXERCISE,
  DUAL_ELBOW_BEND_ASYNC_EXERCISE,
  LESS_IS_DUAL_EXERCISE ,
  //-------------------------
};



#define EXERCISE_ERROR_STRING_LENGTH 1024

struct exerciseData
{
    float x,y,z;
    float amplitudeMin;
    float amplitudeMax;

    unsigned int iterations;
    unsigned int resetFrameID;
    char errorReason[EXERCISE_ERROR_STRING_LENGTH];

    unsigned int frameNumber;
    unsigned int excerciseNumber;
    unsigned int repetitionNumber;
    unsigned int monotonicRepetitionNumber;
};




#ifdef __cplusplus
}
#endif


#endif
