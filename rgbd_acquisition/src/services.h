#ifndef SERVICES_H_INCLUDED
#define SERVICES_H_INCLUDED


#include <ros/ros.h>

extern unsigned char dontPublishPersons;
extern unsigned char dontPublishPointEvents;

extern float actualConfidence;

void broadcastNewPerson();

int registerServices(ros::NodeHandle * nh);


#endif // SERVICES_H_INCLUDED
