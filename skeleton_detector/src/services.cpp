#include "services.h"

//#include "HobbitTrackerLib.h"

#include <ros/ros.h>
#include <ros/spinner.h>
#include "pose.h"

#define SKPREFIX "SK_"

#define BROADCAST_HOBBIT 1
#define MAXIMUM_DISTANCE_FOR_POINTING 400

#include "skeleton_detector/Person.h"
#include "skeleton_detector/PointEvents.h"

ros::Publisher personBroadcaster;
ros::Publisher pointEventsBroadcaster;

#define divisor 1000
//ros::Publisher gestureBroadcaster;

unsigned char dontPublishSkeletons=0;
unsigned char dontPublishPointEvents=0;
unsigned char dontPublishPersons=0;
unsigned int simplePersonDetector = 1;

unsigned int actualTimestamp=0;
unsigned int actualInFieldOfView=0;
float actualX=0.0,actualY=0.0,actualZ=0.0,actualTheta=0.0,actualConfidence=0.51;

unsigned char * colorFrameCopy=0;
unsigned short * depthFrameCopy =0;


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


void broadcastNewPerson()
{
  if (dontPublishPersons) { return ; }

  skeleton_detector::Person msg;
  msg.x = actualX;
  msg.y = actualY;
  msg.z = actualZ;
  msg.theta = actualTheta;

  msg.inFieldOfView = actualInFieldOfView;
  msg.confidence = actualConfidence;
  msg.timestamp=actualTimestamp;

  fprintf(stderr,"Publishing a new Person\n");
  personBroadcaster.publish(msg);
  //ros::spinOnce();
}




void broadcastPointing(unsigned int frameNumber ,struct skeletonPointing * skeletonPointingFound)
{
  if (dontPublishPointEvents) { return ; }

  if (
       ( (skeletonPointingFound->pointingVector.x>-0.0005)&&(skeletonPointingFound->pointingVector.x<0.0005) ) &&
       ( (skeletonPointingFound->pointingVector.y>-0.0005)&&(skeletonPointingFound->pointingVector.y<0.0005) ) &&
       ( (skeletonPointingFound->pointingVector.z>-0.0005)&&(skeletonPointingFound->pointingVector.z<0.0005) )
     )
  {
    fprintf(stderr,"Will not broadcast a null pointing event \n");
    return ;
  }


  fprintf(stderr,"Broadcasting a pointing event \n");

  //David Wants to Flip Y
  signed int YFlipper = -1;

  skeleton_detector::PointEvents msg;
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


int considerSkeletonPointing(unsigned int frameNumber,struct skeletonHuman * skeletonFound)
{
  struct skeletonPointing skelPF={0};

  float distanceLeft = sqrt(
                             simpPow(skeletonFound->joint[HUMAN_SKELETON_TORSO].x - skeletonFound->joint[HUMAN_SKELETON_LEFT_HAND].x ,2)  +
                             simpPow(skeletonFound->joint[HUMAN_SKELETON_TORSO].y - skeletonFound->joint[HUMAN_SKELETON_LEFT_HAND].y ,2)  +
                             simpPow(skeletonFound->joint[HUMAN_SKELETON_TORSO].z - skeletonFound->joint[HUMAN_SKELETON_LEFT_HAND].z ,2)
                           );
  float distanceRight = sqrt(
                             simpPow(skeletonFound->joint[HUMAN_SKELETON_TORSO].x - skeletonFound->joint[HUMAN_SKELETON_RIGHT_HAND].x ,2)  +
                             simpPow(skeletonFound->joint[HUMAN_SKELETON_TORSO].y - skeletonFound->joint[HUMAN_SKELETON_RIGHT_HAND].y ,2)  +
                             simpPow(skeletonFound->joint[HUMAN_SKELETON_TORSO].z - skeletonFound->joint[HUMAN_SKELETON_RIGHT_HAND].z ,2)
                             );


  if ( (distanceLeft<MAXIMUM_DISTANCE_FOR_POINTING) && (distanceRight<MAXIMUM_DISTANCE_FOR_POINTING) ) { fprintf(stderr,"Cutting off pointing "); return 0; }






  int doHand=1; //1 = right , 2 =left
  if (distanceLeft<distanceRight) { doHand=2; }

  if (doHand==2)
  {
   skelPF.pointStart.x = skeletonFound->joint[HUMAN_SKELETON_LEFT_ELBOW].x;
   skelPF.pointStart.y = skeletonFound->joint[HUMAN_SKELETON_LEFT_ELBOW].y;
   skelPF.pointStart.z = skeletonFound->joint[HUMAN_SKELETON_LEFT_ELBOW].z;
   skelPF.pointEnd.x = skeletonFound->joint[HUMAN_SKELETON_LEFT_HAND].x;
   skelPF.pointEnd.y = skeletonFound->joint[HUMAN_SKELETON_LEFT_HAND].y;
   skelPF.pointEnd.z = skeletonFound->joint[HUMAN_SKELETON_LEFT_HAND].z;
   skelPF.pointingVector.x = skelPF.pointEnd.x - skelPF.pointStart.x;
   skelPF.pointingVector.y = skelPF.pointEnd.y - skelPF.pointStart.y;
   skelPF.pointingVector.z = skelPF.pointEnd.z - skelPF.pointStart.z;
   skelPF.isLeftHand=1;
   skelPF.isRightHand=0;

   broadcastPointing(frameNumber,&skelPF);
   return 1;
  } else
  if (doHand==1)
  {
   skelPF.pointStart.x = skeletonFound->joint[HUMAN_SKELETON_RIGHT_ELBOW].x;
   skelPF.pointStart.y = skeletonFound->joint[HUMAN_SKELETON_RIGHT_ELBOW].y;
   skelPF.pointStart.z = skeletonFound->joint[HUMAN_SKELETON_RIGHT_ELBOW].z;
   skelPF.pointEnd.x = skeletonFound->joint[HUMAN_SKELETON_RIGHT_HAND].x;
   skelPF.pointEnd.y = skeletonFound->joint[HUMAN_SKELETON_RIGHT_HAND].y;
   skelPF.pointEnd.z = skeletonFound->joint[HUMAN_SKELETON_RIGHT_HAND].z;
   skelPF.pointingVector.x = skelPF.pointEnd.x - skelPF.pointStart.x;
   skelPF.pointingVector.y = skelPF.pointEnd.y - skelPF.pointStart.y;
   skelPF.pointingVector.z = skelPF.pointEnd.z - skelPF.pointStart.z;
   skelPF.isLeftHand=0;
   skelPF.isRightHand=1;

   broadcastPointing(frameNumber,&skelPF);
   return 1;
  }

 return 0;
}


void broadcastNewSkeleton(unsigned int frameNumber,unsigned int skeletonID , struct skeletonHuman * skeletonFound )
{
    if ( (dontPublishSkeletons) || (skeletonFound==0) ) { return ; }

    fprintf(stderr,"Broadcasting a skeleton at TF\n");
  unsigned int i=0;
  for (i=0; i<HUMAN_SKELETON_PARTS; i++)
    {
      //DO flips here ?
      skeletonFound->joint[i].z = -1 * skeletonFound->joint[i].z;
      fprintf(stderr,"%s(%f,%f,%f)\n",humanSkeletonJointNames[i],skeletonFound->joint[i].x,skeletonFound->joint[i].y,skeletonFound->joint[i].z);
    }
   fprintf(stderr,"\n\n");

   fprintf(stderr,"2DPOSE(");
   for (i=0; i<HUMAN_SKELETON_PARTS; i++)
    {
      if (i<HUMAN_SKELETON_PARTS-1) {  fprintf(stderr,"%f, %f,",humanSkeletonJointNames[i],skeletonFound->joint2D[i].x,skeletonFound->joint2D[i].y); } else
                                    {  fprintf(stderr,"%f, %f)\n",humanSkeletonJointNames[i],skeletonFound->joint2D[i].x,skeletonFound->joint2D[i].y); }
    }
   fprintf(stderr,"\n\n");



     //Do TF Broadcast here
     char tag[256]={0};
     for ( i=0; i<HUMAN_SKELETON_PARTS; i++ )
      {
        if (
              (skeletonFound->joint[i].x!=0) ||
              (skeletonFound->joint[i].y!=0) ||
              (skeletonFound->joint[i].z!=0)
           )
            {
             sprintf(tag,"%s%s",SKPREFIX,jointNames[i]);
             postPoseTransform(tag,skeletonFound->joint[i].x/divisor,skeletonFound->joint[i].y/divisor,skeletonFound->joint[i].z/divisor);
            }
      }

     for ( i=0; i<8; i++ )
      {
       sprintf(tag,SKPREFIX "bbox/point%u",i);
       postPoseTransform(tag,skeletonFound->bbox[i].x/divisor,skeletonFound->bbox[i].y/divisor,-1* skeletonFound->bbox[i].z/divisor);
      }

      actualTimestamp=frameNumber;
      actualX=skeletonFound->joint[HUMAN_SKELETON_HEAD].x;
      actualY=-1.0*skeletonFound->joint[HUMAN_SKELETON_HEAD].y;
      actualZ=skeletonFound->joint[HUMAN_SKELETON_HEAD].z;
      broadcastNewPerson();

      considerSkeletonPointing(frameNumber,skeletonFound);

    return ;
}

int runServicesThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                       unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                        struct calibrationHUBT * calib ,
                                         unsigned int frameTimestamp )
{
  if ( (colorFrameCopy==0) ||  (depthFrameCopy==0) ) { fprintf(stderr,"Cannot run handtracker due to not allocated intermediate buffer\n"); return 0; }
  //Unfortunately gestures need its dedicated frame buffer read/write so we copy frames here before passing them
  memcpy(colorFrameCopy,colorFrame,colorWidth*colorHeight*3*sizeof(unsigned char));
  memcpy(depthFrameCopy,depthFrame,depthWidth*depthHeight*1*sizeof(unsigned short));

  int retres = hobbitUpperBodyTracker_NewFrame(colorFrameCopy , colorWidth , colorHeight ,
                                               depthFrameCopy  , depthWidth , depthHeight ,
                                               calib ,
                                               simplePersonDetector ,
                                               frameTimestamp );



  return retres;
}


int registerServices(ros::NodeHandle * nh,unsigned int width,unsigned int height)
{
  colorFrameCopy = (unsigned char * ) malloc(width*height*3*sizeof(unsigned char));
  if (colorFrameCopy==0) { fprintf(stderr,"Cannot make an intermidiate copy of color frame \n");  }
  depthFrameCopy = (unsigned short * ) malloc(width*height*1*sizeof(unsigned short));
  if (depthFrameCopy==0) { fprintf(stderr,"Cannot make an intermidiate copy of depth frame \n"); }

  hobbitUpperBodyTracker_Initialize(width , height);
  hobbitUpperBodyTracker_RegisterSkeletonDetectedEvent((void *) &broadcastNewSkeleton);

  pointEventsBroadcaster = nh->advertise <skeleton_detector::PointEvents> ("pointEvents", 1000);
  personBroadcaster = nh->advertise <skeleton_detector::Person> ("persons", divisor);
}

int stopServices()
{
  hobbitUpperBodyTracker_Close();

  if (colorFrameCopy!=0) { free(colorFrameCopy); colorFrameCopy=0; }
  if (depthFrameCopy!=0) { free(depthFrameCopy); depthFrameCopy=0; }
}
