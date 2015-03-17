#ifndef HOBBIT_GESTURE_DATA_STRUCTURES_H_INCLUDED
#define HOBBIT_GESTURE_DATA_STRUCTURES_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif


static const char * gestureNamesInternal[] =
{"None",
 "Cancel",
 "Help",
 "Yes",
 "No",
 "Circle", //This is not reward
 "Point",
 "Come",
 "Wave",
 "TODO ADD MORE GESTURE NAMES at gestureNamesInternal[] in hobbitGestureDataStructures.h" ,
 "TODO ADD MORE GESTURE NAMES at gestureNamesInternal[] in hobbitGestureDataStructures.h" ,
};



static const char * gestureNames[] =
{"none",
 "cancel",
 "help",
 "yes",
 "no",
 "reward",
 "point",
 "come",
 "wave",
 "TODO ADD MORE GESTURE NAMES at gestureNames[] in hobbitGestureDataStructures.h" ,
 "TODO ADD MORE GESTURE NAMES at gestureNames[] in hobbitGestureDataStructures.h" ,
};

enum recognizedGestures
{
    GESTURE_NONE=0,
    GESTURE_CANCEL ,
    GESTURE_HELP ,
    GESTURE_YES  ,
    GESTURE_NO   ,
    GESTURE_REWARD   ,
    GESTURE_POINT ,
    GESTURE_COME ,
    GESTURE_WAVE ,
    //----------------
    POSSIBLE_GESTURES
};


struct handGesture
{
  unsigned int gestureID;

  float x,y,z,theta;
  float toX,toY,toZ;

  unsigned int x2D,y2D;

  unsigned int frameTimestamp;
};




#ifdef __cplusplus
}
#endif


#endif
