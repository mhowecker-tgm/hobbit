#include "services.h"

//#include "HobbitTrackerLib.h"

#include <ros/ros.h>
#include <ros/spinner.h>
#include "pose.h"

#define SKPREFIX "SK_"

#define BROADCAST_HOBBIT 1

#include "hand_gestures/HandGesture.h"

#if BROADCAST_HOBBIT
#include "hobbit_msgs/Event.h"
ros::Publisher gestureEventBroadcaster;
#endif


//ros::Publisher gestureBroadcaster;

unsigned char dontPublishSkeletons=0;

unsigned int actualTimestamp=0;
float actualX=0.0,actualY=0.0,actualZ=0.0,actualTheta=0.0,actualConfidence=0.0;

unsigned char * colorFrameCopy=0;
unsigned short * depthFrameCopy =0;


void broadcastNewSkeleton(unsigned int frameNumber,unsigned int skeletonID , struct skeletonHuman * skeletonFound )
{
    if ( (dontPublishSkeletons) || (skeletonFound==0) ) { return ; }

    fprintf(stderr,"Broadcasting a skeleton at TF\n");
     //Do TF Broadcast here
     char tag[256]={0};
     unsigned int i =0;
     for ( i=0; i<HUMAN_SKELETON_PARTS; i++ )
      {
       sprintf(tag,"%s%s",SKPREFIX,jointNames[i]);
        postPoseTransform(tag,/*-1.0**/skeletonFound->joint[i].x/1000,-1.0*skeletonFound->joint[i].y/1000,skeletonFound->joint[i].z/1000);
      }

     for ( i=0; i<8; i++ )
      {
       sprintf(tag,SKPREFIX "bbox/point%u",i);
       postPoseTransform(tag,/*-1.0**/skeletonFound->bbox[i].x/1000,/*-1.0**/skeletonFound->bbox[i].y/1000,skeletonFound->bbox[i].z/1000);
      }

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
                                               calib , frameTimestamp );



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
}

int stopServices()
{
  hobbitUpperBodyTracker_Close();

  if (colorFrameCopy!=0) { free(colorFrameCopy); colorFrameCopy=0; }
  if (depthFrameCopy!=0) { free(depthFrameCopy); depthFrameCopy=0; }
}
