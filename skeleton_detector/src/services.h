#ifndef SERVICES_H_INCLUDED
#define SERVICES_H_INCLUDED


#include "hobbitUpperBodyTrackerLib.h"

#include <ros/ros.h>

extern unsigned char dontPublishSkeletons;
extern unsigned int simplePersonDetector;

int registerServices(ros::NodeHandle * nh,unsigned int width,unsigned int height);
int runServicesThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                       unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                        struct calibrationHUBT * calib ,
                                          unsigned int frameTimestamp );

int stopServices();

#endif // SERVICES_H_INCLUDED
