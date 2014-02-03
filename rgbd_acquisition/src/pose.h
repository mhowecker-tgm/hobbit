#ifndef POSE_H_INCLUDED
#define POSE_H_INCLUDED


#include <tf/transform_broadcaster.h>

int postPoseTransform(char * name , float x ,float y , float z);
int postPoseRTTransform(char * name , float x ,float y , float z , float roll, float pitch, float yaw) ;
int postPoseRQTTransform(char * name , float x ,float y , float z , float qA, float qB, float qC, float qD) ; 

#endif // POSE_H_INCLUDED
