#include "services.h"

#include <ros/ros.h>
#include <ros/spinner.h>
#include "pose.h"

#define NORMAL "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */


#define BROADCAST_HOBBIT 1
#define MAXIMUM_DISTANCE_FOR_POINTING 400

#include <follow_user/CameraMotion.h>
#include <follow_user/TrackerTarget.h>

#include <follow_user/Person.h>
#define USE_PERSON_AGGREGATOR 1

#if USE_PERSON_AGGREGATOR
 #define PERSON_TOPIC "/follow_user/persons"
#else
 #define PERSON_TOPIC "persons"
#endif // USE_PERSON_AGGREGATOR


#include "PeopleTracker.h"

#include "hobbit_msgs/Event.h"
#include <std_msgs/String.h>

ros::Publisher personBroadcaster;

ros::Publisher trackerTargetBroadcaster;
ros::Publisher cameraMotionBroadcaster;

#define divisor 1000


struct peopleTrackerContext * ptcx;


unsigned int actualTimestamp=0;
unsigned int actualInFieldOfView=0;
float actualX=0.0,actualY=0.0,actualZ=0.0,actualTheta=0.0,actualConfidence=0.51;


float simpPow(float base,unsigned int exp)
{
    if (exp==0) return 1;
    float retres=base;
    unsigned int i=0;
    for (i=0; i<exp-1; i++)
    {
        retres*=base;
    }
    return retres;
}


void broadcastTrackedMotion(unsigned int frameTimestamp , struct peopleTrackerMotion * ptm)
{
  follow_user::CameraMotion msg;
  msg.dX=ptm->dX;
  msg.dY=ptm->dY;
  msg.dZ=ptm->dZ;
  msg.angleX=ptm->angleX;
  msg.angleY=ptm->angleY;
  msg.angleZ=ptm->angleZ;

  msg.timestamp = frameTimestamp;

  cameraMotionBroadcaster.publish(msg);
}


/*
  from Hobbit README.first
  In relation to a body the standard is :

    x forward
    y left
    z up

   In the case of cameras, there is often a second frame defined with a "_optical" suffix. This uses a slightly different convention:

    z forward
    x right
    y down
*/

void broadcastNewPerson(unsigned int frameTimestamp , unsigned int trackerID , struct peopleTrackerTarget * ptt)
{
  follow_user::Person msg;

  msg.x=ptt->x*1000;
  msg.y=0.0*1000;
  msg.z=ptt->y*1000;
  msg.source = 6;
  msg.theta = 0;

  msg.inFieldOfView = 1;
  msg.confidence = 0.8;
  msg.timestamp=frameTimestamp;

  personBroadcaster.publish(msg);

  char targetName[128]={0};
  snprintf(targetName,128,"follow_user_target_%u",trackerID);

  postPoseTransform((char*) targetName,/*-1.0**/msg.x/1000,/*-1.0**/msg.y/1000,msg.z/1000);
}

void broadcastTrackedTarget(unsigned int frameTimestamp , unsigned int trackerID , struct peopleTrackerTarget * ptt)
{
  follow_user::TrackerTarget msg;
  msg.x=ptt->x;
  msg.y=ptt->y;
  msg.vX=ptt->vx;
  msg.vY=ptt->vy;
  msg.radious=ptt->rad;

  msg.id=ptt->id;
  msg.state=ptt->state;
  msg.status=ptt->status;

  msg.timestamp = frameTimestamp;
  trackerTargetBroadcaster.publish(msg);

  //Also trigger a person message
  broadcastNewPerson( frameTimestamp , trackerID , ptt );
}


int internalSetVisualization(int state)
{
    peopleTracker_SetVisualization(ptcx,state);
    return 1;
}

int runServicesThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                       unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                        struct calibrationHobbit * calib ,
                                         unsigned int frameTimestamp )
{
  int retres=0;

  //fprintf(stderr,"Passing new frame .. ");
   retres = peopleTracker_NewFrame(ptcx,
                                   colorFrame , colorWidth , colorHeight ,
                                   depthFrame  , depthWidth , depthHeight ,
                                   frameTimestamp );
  //fprintf(stderr," survived \n");

  return retres;
}

int resumeServices()
{
  peopleTracker_Reset(ptcx);
  return 1;
}


int registerServices(ros::NodeHandle * nh,unsigned int width,unsigned int height)
{
  personBroadcaster = nh->advertise <follow_user::Person> (PERSON_TOPIC, 1000);

  trackerTargetBroadcaster = nh->advertise <follow_user::TrackerTarget> ("trackedTargets", 1000);
  cameraMotionBroadcaster = nh->advertise <follow_user::CameraMotion> ("cameraMotion", 1000);

  ptcx = peopleTracker_Initialize("options.ini");
  peopleTracker_RegisterTargetDetectedEvent(ptcx,(void *) & broadcastTrackedTarget);
  peopleTracker_RegisterPositionUpdateEvent(ptcx,(void *) & broadcastTrackedMotion);
 return 1;
}

int stopServices()
{
  peopleTracker_Close(ptcx);
  return 1;
}
