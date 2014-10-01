#include "services.h"

#include "Nite2.h"

#include <ros/ros.h>
#include <ros/spinner.h>

#include "rgbd_acquisition/Person.h"
#include "rgbd_acquisition/PointEvents.h"
#include "rgbd_acquisition/Skeleton2D.h"
#include "rgbd_acquisition/Skeleton3D.h"
#include "rgbd_acquisition/SkeletonBBox.h"
#include "pose.h"



#define USE_PERSON_AGGREGATOR 1

#if USE_PERSON_AGGREGATOR
 #define PERSON_TOPIC "/rgbd_acquisition/persons"
#else
 #define PERSON_TOPIC "persons"
#endif // USE_PERSON_AGGREGATOR



ros::Publisher joint2DBroadcaster;
ros::Publisher joint3DBroadcaster;
ros::Publisher jointBBoxBroadcaster;
ros::Publisher personBroadcaster;
ros::Publisher pointEventsBroadcaster;

unsigned char dontPublishPersons=0;
unsigned char dontPublishPointEvents=0;

unsigned int actualTimestamp=0;
unsigned int actualInFieldOfView=0;
float actualConfidence=0.5;
float actualX=0.0,actualY=0.0,actualZ=0.0,actualTheta=0.0;


void broadcastNewPerson()
{
  if (dontPublishPersons) { return ; }

  rgbd_acquisition::Person msg;
  msg.x = actualX;
  msg.y = actualY;
  msg.z = actualZ;
  msg.source = 1;
  msg.theta = actualTheta;

  msg.inFieldOfView = actualInFieldOfView;
  msg.confidence = actualConfidence;
  msg.timestamp=actualTimestamp;

  fprintf(stderr,"Publishing a new Person\n");
  personBroadcaster.publish(msg);
  ros::spinOnce();
}


void broadcast2DJoints(struct skeletonHuman * skeletonFound)
{
  if (dontPublishPersons) { return ; }

  rgbd_acquisition::Skeleton2D msg;
  msg.joints2D.resize(HUMAN_SKELETON_PARTS * 2, 0.0);

  for (unsigned int i=0; i<HUMAN_SKELETON_PARTS; i++)
  {
    msg.joints2D[2*i+0]=skeletonFound->joint2D[i].x;
    msg.joints2D[2*i+1]=skeletonFound->joint2D[i].y;
  }
  msg.numberOfJoints=HUMAN_SKELETON_PARTS;
  msg.timestamp=actualTimestamp;

  joint2DBroadcaster.publish(msg);
  //ros::spinOnce();
}


void broadcast3DJoints(struct skeletonHuman * skeletonFound)
{
  if (dontPublishPersons) { return ; }

  rgbd_acquisition::Skeleton3D msg;
  msg.joints3D.resize(HUMAN_SKELETON_PARTS * 3, 0.0);

  for (unsigned int i=0; i<HUMAN_SKELETON_PARTS; i++)
  {
    msg.joints3D[3*i+0]=skeletonFound->joint[i].x;
    msg.joints3D[3*i+1]=skeletonFound->joint[i].y;
    msg.joints3D[3*i+2]=skeletonFound->joint[i].z;
  }
  msg.numberOfJoints=HUMAN_SKELETON_PARTS;
  msg.timestamp=actualTimestamp;

  joint3DBroadcaster.publish(msg);
}


void broadcast2DBBox(struct skeletonHuman * skeletonFound)
{
  if (dontPublishPersons) { return ; }

  rgbd_acquisition::SkeletonBBox msg;

  msg.width3D=skeletonFound->bboxDimensions.x;
  msg.height3D=skeletonFound->bboxDimensions.y;
  msg.depth3D=skeletonFound->bboxDimensions.z;

  msg.centerX3D = skeletonFound->centerOfMass.x;
  msg.centerY3D = skeletonFound->centerOfMass.y;
  msg.centerZ3D = skeletonFound->centerOfMass.z;

  msg.width2D=skeletonFound->bboxDimensions.x;
  msg.height2D=skeletonFound->bboxDimensions.y;

  msg.centerX2D = skeletonFound->centerOfMass.x;
  msg.centerY2D = skeletonFound->centerOfMass.y;

  msg.timestamp=actualTimestamp;

  fprintf(stderr,"Publishing a new Joint BBox configuration\n");
  jointBBoxBroadcaster.publish(msg);
  ros::spinOnce();
}





void broadcastPointing(unsigned int frameNumber ,struct skeletonPointing * skeletonPointingFound)
{
  if (dontPublishPointEvents) { return ; }
  fprintf(stderr,"Broadcasting a pointing event \n");

  //David Wants to Flip Y
  signed int YFlipper = -1;

  rgbd_acquisition::PointEvents msg;
  msg.x = skeletonPointingFound->pointStart.x;
  msg.y = YFlipper*skeletonPointingFound->pointStart.y;
  msg.z = skeletonPointingFound->pointStart.z;

  msg.vectorX = skeletonPointingFound->pointingVector.x;
  msg.vectorY = YFlipper*skeletonPointingFound->pointingVector.y;
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
       postPoseTransform(tag,/*-1.0**/skeletonFound->bbox[i].x/1000, -1.0* skeletonFound->bbox[i].y/1000,skeletonFound->bbox[i].z/1000);
      }
     broadcastNewPerson();

     broadcast2DJoints(skeletonFound);
     broadcast2DBBox(skeletonFound);
}




int registerServices(ros::NodeHandle * nh)
{
    personBroadcaster = nh->advertise <rgbd_acquisition::Person> (PERSON_TOPIC, 1000);
    pointEventsBroadcaster = nh->advertise <rgbd_acquisition::PointEvents> ("pointEvents", 1000);
    joint2DBroadcaster = nh->advertise <rgbd_acquisition::Skeleton2D> ("joints2D", 1000);
  joint3DBroadcaster = nh->advertise <rgbd_acquisition::Skeleton3D> ("joints3D", 1000);
    jointBBoxBroadcaster = nh->advertise <rgbd_acquisition::SkeletonBBox> ("jointsBBox", 1000);


    registerSkeletonDetectedEvent(0,(void*) &broadcastSkeleton);
    registerSkeletonPointingDetectedEvent(0,(void *) &broadcastPointing);
}
