#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "pose.h"

#define tfRoot "frame"

int postPoseTransform(char * name , float x ,float y , float z)
{
 static tf::TransformBroadcaster br;
 tf::Transform transform;
 transform.setOrigin( tf::Vector3(x, y , z ) );
 transform.setRotation( tf::createQuaternionFromRPY(0, 0, 0) ); //if we have xyz we dont have a rotation


 br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tfRoot , name));

 //printf("postPoseTransform : %s/%s %f %f %f\n",tfRoot,name,x,y,z);
 return 1;
}

int postPoseRTTransform(char * name , float x ,float y , float z , float roll, float pitch, float yaw)
{
 static tf::TransformBroadcaster br;
 tf::Transform transform;
 transform.setOrigin( tf::Vector3(x, y , z ) );
 transform.setRotation( tf::createQuaternionFromRPY(roll, pitch, yaw) );

 br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tfRoot , name));

 //printf("postPoseRTTransform : %s/%s %f %f %f\n",tfRoot,name,x,y,z,roll,pitch,yaw);
 return 1;
}

int postPoseRQTTransform(char * name , float x ,float y , float z , float qA, float qB, float qC, float qD)
{
 static tf::TransformBroadcaster br;
 tf::Transform transform;
 transform.setOrigin( tf::Vector3(x, y , z ) );
 transform.setRotation( tf::Quaternion(qA, qB, qC , qD) );

 br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tfRoot , name));

 //printf("postPoseRQTTransform : %s/%s %f %f %f %f\n",tfRoot,name,x,y,z,qA,qB,qC,qD);
 return 1;
}

