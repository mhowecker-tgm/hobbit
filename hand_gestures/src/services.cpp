#include "services.h"

#include "HobbitTrackerLib.h"

#include <ros/ros.h>
#include <ros/spinner.h>

#include "hand_gestures/HandGesture.h"

ros::Publisher gestureBroadcaster;

unsigned char dontPublishGestures=0;

unsigned int actualTimestamp=0;
float actualX=0.0,actualY=0.0,actualZ=0.0,actualTheta=0.0,actualConfidence=0.0;

unsigned char * colorFrameCopy=0;
unsigned short * depthFrameCopy =0;



void broadcastNewGesture(unsigned int frameNumber,struct handGesture * gesture)
{
    if ( (dontPublishGestures) || (gesture==0) ) { return ; }

    hand_gestures::HandGesture msg;
    msg.x = gesture->x;
    msg.y = gesture->y;
    msg.z = gesture->z;
    msg.theta = 0;

    msg.confidence = 1.0;
    msg.timestamp=frameNumber;

    fprintf(stderr,"Publishing a new gesture\n");
    gestureBroadcaster.publish(msg);
    return ;
}

int runServicesThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                       unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                        struct calibration * calib ,
                                         unsigned int frameTimestamp )
{
  if ( (colorFrameCopy==0) ||  (depthFrameCopy==0) ) { fprintf(stderr,"Cannot run handtracker due to not allocated intermediate buffer\n"); return 0; }
  //Unfortunately gestures need its dedicated frame buffer read/write so we copy frames here before passing them
  memcpy(colorFrameCopy,colorFrame,colorWidth*colorHeight*3*sizeof(unsigned char));
  memcpy(depthFrameCopy,depthFrame,depthWidth*depthHeight*1*sizeof(unsigned short));

  int retres = hobbitGestures_NewFrame(colorFrameCopy , colorWidth , colorHeight ,
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

  gestureBroadcaster = nh->advertise <hand_gestures::HandGesture> ("gestures", 1000);

  hobbitGestures_Initialize(width , height);
  hobbitGestures_RegisterGestureDetectedEvent((void *) &broadcastNewGesture);
}

int stopServices()
{
    hobbitGestures_Close();

  if (colorFrameCopy!=0) { free(colorFrameCopy); colorFrameCopy=0; }
  if (depthFrameCopy!=0) { free(depthFrameCopy); depthFrameCopy=0; }
}
