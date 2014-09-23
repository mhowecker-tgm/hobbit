#include "services.h"

#include "HobbitTrackerLib.h"

#include <ros/ros.h>
#include <ros/spinner.h>

#include <iostream>
#include <sstream>
#include <string>

#define BROADCAST_HOBBIT 1

#include "hand_gestures/HandGesture.h"

#if BROADCAST_HOBBIT
#include "hobbit_msgs/Event.h"
#include <std_msgs/String.h>
ros::Publisher gestureEventBroadcaster;
#endif


unsigned int MaximumDelayBetweenPersonAndGesture = 100;
unsigned int headLookingDirection=HEAD_LOOKING_DOWN; //head looking down by default

ros::Publisher gestureBroadcaster;

unsigned int lastPersonTimestamp=0;
unsigned int frameTimestamp =0;

unsigned char dontPublishGestures=0;

unsigned int actualTimestamp=0;
float actualX=0.0,actualY=0.0,actualZ=0.0,actualTheta=0.0,actualConfidence=0.0;

unsigned char * colorFrameCopy=0;
unsigned short * depthFrameCopy =0;


void broadcastGesturesStatus(unsigned int frameNumber,unsigned int gesturesPaused)
{
    hobbit_msgs::Event evt;
    std::stringstream ss;

    if  (gesturesPaused) { ss<<"G_DISABLED";  } else
                         { ss<<"G_ACTIVATED";  }
    evt.event=ss.str();
    evt.header.seq = frameNumber;
    evt.header.frame_id = "hand_gestures";
    evt.header.stamp = ros::Time::now();
    evt.sessionID  = "SessionID";
    evt.confidence = 1.0;
    evt.params.resize(0);
    fprintf(stderr,"Publishing a new Pause/Resume event ( pause=%u ) \n",gesturesPaused);
    gestureEventBroadcaster.publish(evt);
}



void broadcastNewGesture(unsigned int frameNumber,struct handGesture * gesture)
{
    if ( (dontPublishGestures) || (gesture==0) ) { return ; }


    if (frameTimestamp<lastPersonTimestamp) { fprintf(stderr,"wtf , frame before person?");   }
    if (frameTimestamp-lastPersonTimestamp >= MaximumDelayBetweenPersonAndGesture )
    {
       fprintf(stderr,"Will not publish gesture , no person detected , could be false positive\n");
       return ;
    }

    if (headLookingDirection!=HEAD_LOOKING_CENTER)
    {
      if (
           (gesture->gestureID==GESTURE_REWARD)  ||
           (gesture->gestureID==GESTURE_HELP)
         )
      {
        fprintf(stderr,"Gesture will be hidden because we are not looking in the center position");
        return ;
      }
    }

#if BROADCAST_HOBBIT
    hobbit_msgs::Event evt;

    std::stringstream ss;
    //std_msgs::String sROS;
    switch (gesture->gestureID)
    {
     case GESTURE_NONE   : ss<<"G_NONE";   break;
     case GESTURE_CANCEL : ss<<"G_CANCEL"; break;
     case GESTURE_HELP   : ss<<"G_HELP";   break;
     case GESTURE_YES    : ss<<"G_YES";    break;
     case GESTURE_NO     : ss<<"G_NO";     break;
     case GESTURE_REWARD : ss<<"G_REWARD"; break;
     case GESTURE_POINT  : ss<<"G_POINT";  break;
     default :             ss<<"G_NOTFOUND"; break;
    };

    //sROS.data=ss.str();
    evt.event=ss.str();
    evt.header.seq = frameNumber;
    evt.header.frame_id = "hand_gestures";
    evt.header.stamp = ros::Time::now();
    evt.sessionID  = "SessionID";
    evt.confidence = 1.0;
    evt.params.resize(0);
    fprintf(stderr,"Publishing a new Event ( %u ) \n",gesture->gestureID);
    gestureEventBroadcaster.publish(evt);
#endif



    hand_gestures::HandGesture msg;

#if BROADCAST_HOBBIT
    msg.gesture = ss.str();
#endif

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
                                        struct calibrationHT * calib ,
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


#if BROADCAST_HOBBIT
  gestureEventBroadcaster = nh->advertise <hobbit_msgs::Event> ("Event", 1000);
#endif

  hobbitGestures_Initialize(width , height);
  hobbitGestures_RegisterGestureDetectedEvent((void *) &broadcastNewGesture);
}

int stopServices()
{
    hobbitGestures_Close();

  if (colorFrameCopy!=0) { free(colorFrameCopy); colorFrameCopy=0; }
  if (depthFrameCopy!=0) { free(depthFrameCopy); depthFrameCopy=0; }
}
