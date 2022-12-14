#ifndef SERVICES_H_INCLUDED
#define SERVICES_H_INCLUDED


#include "hobbitUpperBodyTrackerLib.h"

#include <ros/ros.h>

extern unsigned char dontPublishSkeletons;
extern unsigned char dontPublishPointEvents;
extern unsigned int processingMode;
extern volatile int paused;


extern unsigned int frameTimestamp;
extern unsigned int actualTimestamp;

int triggerAttentionInternal();

int logStartExerciseTime();
int hasExerciseTimedOut();

int registerServices(ros::NodeHandle * nh,unsigned int width,unsigned int height);
int runServicesThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                       unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                        struct calibrationHobbit * calib ,
                                          unsigned int frameTimestamp );

int stopServices();

#endif // SERVICES_H_INCLUDED
