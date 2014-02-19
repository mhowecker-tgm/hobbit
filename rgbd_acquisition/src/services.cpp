#include "services.h"

#include "Nite2.h"

#include <ros/ros.h>
#include <ros/spinner.h>

#include "rgbd_acquisition/Person.h"
#include "rgbd_acquisition/PointEvents.h"
#include "pose.h"

ros::Publisher personBroadcaster;
ros::Publisher pointEventsBroadcaster;

unsigned char dontPublishPersons=0;
unsigned char dontPublishPointEvents=0;

unsigned int actualTimestamp=0;
unsigned int actualInFieldOfView=0;
float actualX=0.0,actualY=0.0,actualZ=0.0,actualTheta=0.0,actualConfidence=0.0;


void broadcastNewPerson()
{
  if (dontPublishPersons) { return ; }

  rgbd_acquisition::Person msg;
  msg.x = actualX;
  msg.y = actualY;
  msg.z = actualZ;
  msg.theta = actualTheta;

  msg.inFieldOfView = actualInFieldOfView;
  msg.confidence = actualConfidence;
  msg.timestamp=actualTimestamp;

  fprintf(stderr,"Publishing a new Person\n");
  personBroadcaster.publish(msg);
  ros::spinOnce();
}





void broadcastPointing(unsigned int frameNumber ,struct skeletonPointing * skeletonPointingFound)
{
  if (dontPublishPointEvents) { return ; }
  fprintf(stderr,"Broadcasting a pointing event \n");


  rgbd_acquisition::PointEvents msg;
  msg.x = skeletonPointingFound->pointStart.x;
  msg.y = skeletonPointingFound->pointStart.y;
  msg.z = skeletonPointingFound->pointStart.z;

  msg.vectorX = skeletonPointingFound->pointingVector.x;
  msg.vectorY = skeletonPointingFound->pointingVector.y;
  msg.vectorZ = skeletonPointingFound->pointingVector.z;

  msg.leftHand = skeletonPointingFound->isLeftHand;
  msg.rightHand = skeletonPointingFound->isRightHand;

  msg.timestamp=frameNumber;

  fprintf(stderr,"Publishing a new Pointing Event\n");
  pointEventsBroadcaster.publish(msg);

}



void broadcastSkeleton(unsigned int frameNumber ,struct skeletonHuman * skeletonFound)
{
    fprintf(stderr,"Broadcasting a skeleton \n");

     actualTimestamp=frameNumber;
     actualInFieldOfView=skeletonFound->isVisible;
     actualConfidence=skeletonFound->jointAccuracy[HUMAN_SKELETON_HEAD];
     actualX=skeletonFound->joint[HUMAN_SKELETON_HEAD].x;
     actualY=skeletonFound->joint[HUMAN_SKELETON_HEAD].y;
     actualZ=skeletonFound->joint[HUMAN_SKELETON_HEAD].z;
     actualTheta=0.0;


     //Do TF Broadcast here
     unsigned int i =0;
     for ( i=0; i<HUMAN_SKELETON_PARTS; i++ )
      {
        postPoseTransform((char*) jointNames[i],/*-1.0**/skeletonFound->joint[i].x/1000,-1.0*skeletonFound->joint[i].y/1000,skeletonFound->joint[i].z/1000);
      }

     char tag[123]={0};
     for ( i=0; i<8; i++ )
      {
       sprintf(tag,"bbox/point%u",i);
       postPoseTransform(tag,/*-1.0**/skeletonFound->bbox[i].x/1000,/*-1.0**/skeletonFound->bbox[i].y/1000,skeletonFound->bbox[i].z/1000);
      }
     broadcastNewPerson();
}




int registerServices(ros::NodeHandle * nh)
{
    personBroadcaster = nh->advertise <rgbd_acquisition::Person> ("persons", 1000);
    pointEventsBroadcaster = nh->advertise <rgbd_acquisition::PointEvents> ("pointEvents", 1000);


    registerSkeletonDetectedEvent(0,(void*) &broadcastSkeleton);
    registerSkeletonPointingDetectedEvent(0,(void *) &broadcastPointing);
}
