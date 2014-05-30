#include "services.h"

//#include "HobbitTrackerLib.h"

#include <ros/ros.h>
#include <ros/spinner.h>


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


/*
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



#if BROADCAST_HOBBIT
    hobbit_msgs::Event evt;
    switch (gesture->gestureID)
    {
     case GESTURE_NONE   : evt.event="G_NONE";   break;
     case GESTURE_CANCEL : evt.event="G_CANCEL"; break;
     case GESTURE_HELP   : evt.event="G_HELP";   break;
     case GESTURE_YES    : evt.event="G_YES";    break;
     case GESTURE_NO     : evt.event="G_NO";     break;
     case GESTURE_REWARD : evt.event="G_REWARD"; break;
     case GESTURE_POINT  : evt.event="G_POINT";  break;
     default :             evt.event="G_NOTFOUND"; break;
    };
    evt.header.seq = frameNumber;
    evt.header.frame_id = "hand_gestures";
    evt.header.stamp = ros::Time::now();
    evt.sessionID  = "SessionID";
    evt.confidence = 1.0;
    evt.params.resize(0);
    fprintf(stderr,"Publishing a new Event ( %u ) \n",gesture->gestureID);
    gestureEventBroadcaster.publish(evt);
#endif



    return ;
}*/

int runServicesThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                       unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                        struct calibrationHUBT * calib ,
                                         unsigned int frameTimestamp )
{
  if ( (colorFrameCopy==0) ||  (depthFrameCopy==0) ) { fprintf(stderr,"Cannot run handtracker due to not allocated intermediate buffer\n"); return 0; }
  //Unfortunately gestures need its dedicated frame buffer read/write so we copy frames here before passing them
  memcpy(colorFrameCopy,colorFrame,colorWidth*colorHeight*3*sizeof(unsigned char));
  memcpy(depthFrameCopy,depthFrame,depthWidth*depthHeight*1*sizeof(unsigned short));
  
  int retres=0;
  //int retres = hobbitGestures_NewFrame(colorFrameCopy , colorWidth , colorHeight ,
  //                                      depthFrameCopy  , depthWidth , depthHeight ,
  //                                       calib , frameTimestamp );



  return retres;
}


int registerServices(ros::NodeHandle * nh,unsigned int width,unsigned int height)
{
  colorFrameCopy = (unsigned char * ) malloc(width*height*3*sizeof(unsigned char));
  if (colorFrameCopy==0) { fprintf(stderr,"Cannot make an intermidiate copy of color frame \n");  }
  depthFrameCopy = (unsigned short * ) malloc(width*height*1*sizeof(unsigned short));
  if (depthFrameCopy==0) { fprintf(stderr,"Cannot make an intermidiate copy of depth frame \n"); }
/*
  gestureBroadcaster = nh->advertise <hand_gestures::HandGesture> ("gestures", 1000);


#if BROADCAST_HOBBIT
  gestureEventBroadcaster = nh->advertise <hobbit_msgs::Event> ("Event", 1000);
#endif
*/
  //hobbitGestures_Initialize(width , height);
  //hobbitGestures_RegisterGestureDetectedEvent((void *) &broadcastNewGesture);
}

int stopServices()
{
  //  hobbitGestures_Close();

  if (colorFrameCopy!=0) { free(colorFrameCopy); colorFrameCopy=0; }
  if (depthFrameCopy!=0) { free(depthFrameCopy); depthFrameCopy=0; }
}
