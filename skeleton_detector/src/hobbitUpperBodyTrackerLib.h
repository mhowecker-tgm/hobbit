#ifndef HOBBIT_UPPERBODY_TRACKER_LIB_H_INCLUDED
#define HOBBIT_UPPERBODY_TRACKER_LIB_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif

//All the data structures we need..
#include "hobbitGestureDataStructures.h"
#include "hobbitUpperBodyTrackerDataStructures.h"
#include "hobbitCalibrationDataStructures.h"

int hobbitUpperBodyTracker_useFaceDetector(int useFaceDetector);
int hobbitUpperBodyTracker_useGestures(int useGestures);
int hobbitUpperBodyTracker_setFloor(float floorX,float floorY,float floorZ,float floorNormalX,float floorNormalY,float floorNormalZ);

int hobbitUpperBodyTracker_setSamplingFrameSize(unsigned int width,unsigned int height);
int hobbitUpperBodyTracker_setSamplingFrameRate(float fps,unsigned int block);

int hobbitUpperBodyTracker_setVisualization(int doVis);
int hobbitUpperBodyTracker_setDumpToFiles(int doDump);
int hobbitUpperBodyTracker_setPrintStats(int doPrintStats);

int hobbitUpperBodyTracker_Initialize(unsigned int targetWidth,unsigned int targetHeight);

int hobbitUpperBodyTracker_Close();
void hobbitUpperBodyTracker_NewSkeletonDetectedEvent(unsigned int frameNumber,void * skeleton );

int hobbitUpperBodyTracker_RegisterSkeletonDetectedEvent(void * callback);
int hobbitUpperBodyTracker_RegisterGestureDetectedEvent(void * callback);

int hobbitUpperBodyTracker_NewFrame(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                    unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                    struct calibrationHobbit * frameCalibration ,
                                    unsigned int processingMode ,
                                    unsigned int frameTimestamp );

void hobbitUpperBodyTracker_Clear();

#ifdef __cplusplus
}
#endif


#endif
