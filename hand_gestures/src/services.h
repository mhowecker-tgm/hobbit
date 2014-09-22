#ifndef SERVICES_H_INCLUDED
#define SERVICES_H_INCLUDED


#include "HobbitTrackerLib.h"

#include <ros/ros.h>


enum HEAD_LOOK_DIRECTION_ENUM
{
   HEAD_UNKNOWN_DIRECTION = 0,
   HEAD_LOOKING_DOWN     ,
   HEAD_LOOKING_CENTER  ,
   HEAD_LOOKING_UP
};


extern unsigned char dontPublishGestures;
extern unsigned int headLookingDirection;

void broadcastGesturesStatus(unsigned int frameNubmer,unsigned int gesturesPaused);

int registerServices(ros::NodeHandle * nh,unsigned int width,unsigned int height);
int runServicesThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                       unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                        struct calibrationHT * calib ,
                                          unsigned int frameTimestamp );

int stopServices();

#endif // SERVICES_H_INCLUDED
