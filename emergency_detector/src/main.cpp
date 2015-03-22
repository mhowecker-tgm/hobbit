#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/spinner.h>

//#include <stdexcept>
#include <image_transport/image_transport.h>

#include "process.h"
#include "fall_detection.h"
#include "classifier.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
 #include <opencv2/imgproc/imgproc_c.h>
 #include <opencv2/legacy/legacy.hpp>
 #include "opencv2/highgui/highgui.hpp"

#include <std_srvs/Empty.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include "emergency_detector/SkeletonBBox.h"
//#include "emergency_detector/Skeleton2D.h"
//#include "emergency_detector/Skeleton3D.h"
#include "emergency_detector/Skeleton2D3D.h"
#include "emergency_detector/Person.h"

#include <std_msgs/Float32.h>

#define BROADCAST_HOBBIT 1

#if BROADCAST_HOBBIT
#include "hobbit_msgs/Event.h"
#include <std_msgs/String.h>
ros::Publisher gestureEventBroadcaster;
ros::Publisher personBroadcaster;
#endif



#define USE_PERSON_AGGREGATOR 1

#if USE_PERSON_AGGREGATOR
 #define PERSON_TOPIC "/emergency_detector/persons"
#else
 #define PERSON_TOPIC "persons"
#endif // USE_PERSON_AGGREGATOR


#define DEFAULT_FRAME_RATE 5
#define HEAD_POSITION_LOOKUP_DURATION 1.5

int rate=DEFAULT_FRAME_RATE;


float MinMaxHumanTemperatures[]= {
                                    0 , 0  ,       //DEV MACHINE
                                    25.7 , 33.0 , //Hobbit Demo settings *WITH* clothes
                                    //30.8 , 36.0 , //Hobbit A , last tested 23/1/15
                                    29.9 , 36.0 , //Hobbit B , last tested 23/1/15
                                    30.8 , 36.0 , //Hobbit C , todo when I get back to crete
                                    25.6 , 33.0 , //Hobbit D , last checked 17/03/15
                                    29.8 , 36.0 , //Hobbit E , last tested 17/2/15
                                    30.8 , 36.0   //Hobbit F , N/A
                                 };






message_filters::Subscriber<sensor_msgs::Image> *rgb_img_sub_top;
message_filters::Subscriber<sensor_msgs::CameraInfo> *rgb_cam_info_sub_top;
message_filters::Subscriber<sensor_msgs::Image> *depth_img_sub_top;
message_filters::Subscriber<sensor_msgs::CameraInfo> *depth_cam_info_sub_top;


message_filters::Subscriber<sensor_msgs::Image> *rgb_img_sub_bottom;
message_filters::Subscriber<sensor_msgs::CameraInfo> *rgb_cam_info_sub_bottom;
message_filters::Subscriber<sensor_msgs::Image> *depth_img_sub_bottom;
message_filters::Subscriber<sensor_msgs::CameraInfo> *depth_cam_info_sub_bottom;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RgbdSyncPolicy;

int autoRecordEmergencyTriggers=1;
unsigned int lastEmergencyDetectionTimestamp=0;
unsigned int emergencyDetectionCooldown=250; //This should be time , not frames


char emergencyDir[]="../../../web_interface/bin/emergencies/emergencies";
char safeDir[]="../../../web_interface/bin/emergencies/safe";
char triggersDir[]="../../../web_interface/bin/emergencies/triggers";
char curDir[]="../../../web_interface/bin/emergencies/";


double headLastPitch=0.0;
double headLastRoll=0.0;

bool first=false;
int key = 0;
unsigned int frameTimestamp=emergencyDetectionCooldown+1; //not 0 so we can immediately trigger
ros::NodeHandle * nhPtr=0;
unsigned int paused=0;
unsigned int dontPublishPersons=0;
unsigned int printTFPosition=0;
unsigned int useTFTree=1;
unsigned int fakeTemperatureActivated=0;

void broadcastNewPerson()
{
  if (dontPublishPersons) { return ; }
  personDetected=0;

  emergency_detector::Person msg;
  msg.x = lastState.temperatureX;
  msg.y = lastState.temperatureY;
  msg.z = lastState.temperatureZ;;
  msg.source = 5; //5 = temperature / emergency
  msg.theta = 0;
  ros::Time timestampOfCreation = ros::Time::now();
  msg.stamp = timestampOfCreation;

  msg.inFieldOfView = 1;
  msg.confidence = 0.5;
  msg.timestamp=frameTimestamp;

  fprintf(stderr, "Publishing a new Person @ %0.2f %0.2f %0.2f\n" ,lastState.temperatureX,lastState.temperatureY,lastState.temperatureZ);
  personBroadcaster.publish(msg);
  ros::spinOnce();
}



void broadcastEmergency(unsigned int frameNumber)
{
  if ( !emergencyDetected)  { return ; }


  if ( frameNumber <= lastEmergencyDetectionTimestamp  + emergencyDetectionCooldown )
  {
    emergencyDetected=0;
    fprintf(stderr,"Throttling Emergency Event this frame %u , last detection at %u , cooldown %u  \n" , frameNumber , lastEmergencyDetectionTimestamp , emergencyDetectionCooldown);
    return ;
  }


  #if BROADCAST_HOBBIT
    hobbit_msgs::Event evt;
    std::stringstream ss;
     //case GESTURE_NONE   : break;
     if (emergencyDetected) { ss<<"G_FALL";  ROS_INFO("Emergency node , emitting emergency..!");  } else
                             { ss<<"G_NONE"; }

    //sROS.data=ss.str();
    evt.event=ss.str();
    evt.header.seq = frameNumber;
    evt.header.frame_id = "emergency_detector";
    evt.header.stamp = ros::Time::now();
    evt.sessionID  = "SessionID";
    evt.confidence = 1.0;
    evt.params.resize(0);
    fprintf(stderr,"Publishing a new Emergency Event ( %u ) \n",emergencyDetected);
    gestureEventBroadcaster.publish(evt);

    lastEmergencyDetectionTimestamp=frameNumber;

    //No longer at an emergency state
    emergencyDetected=0;
   #endif

 return ;
}


//----------------------------------------------------------
//Advertised Service switches


void getAmbientTemperature(const std_msgs::Float32::ConstPtr& request)
{
    lastState.ambientTemperature =request->data;
    return;
}


void getObjectTemperature(const std_msgs::Float32::ConstPtr& request)
{
    lastState.objectTemperature=request->data;
    lastState.timestampTemperature=frameTimestamp;
    return;
}


bool terminate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Stopping Emergency Detector");
    exit(0);
    return true;
}


bool trigger(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    emergencyDetected=1;
    return true;
}

bool pause(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    paused=1;
    return true;
}

bool resume(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    paused=0;
    return true;
}


bool clearRecorded(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
   char buffer[512]={0};

   snprintf(buffer,512,"rm %s/*.pnm",emergencyDir);
   int i = system(buffer);
   if (i!=0) { fprintf(stderr,"Error cleaning up %s ",emergencyDir); }

   snprintf(buffer,512,"rm %s/*.pnm",safeDir);
   i = system(buffer);
   if (i!=0) { fprintf(stderr,"Error cleaning up %s ",safeDir); }

   snprintf(buffer,512,"rm %s/*.pnm",triggersDir);
   i = system(buffer);
   if (i!=0) { fprintf(stderr,"Error cleaning up %s ",triggersDir); }

}

bool classifyEmergency(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  imageDir=emergencyDir;
  saveNextTopFrame=1;
  saveNextBottomFrame=1;
  return true;
}




bool classifySafe(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  imageDir=safeDir;
  saveNextTopFrame=1;
  saveNextBottomFrame=1;
  return true;
}





bool save(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  imageDir=curDir;
  saveNextTopFrame=1;
  saveNextBottomFrame=1;
  return true;
}


bool visualizeOn(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    doCVOutput=1;
    return true;
}

bool visualizeOff(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    doCVOutput=0;
    cv::destroyAllWindows();
    cv::destroyWindow("emergency_detector visualization");
    cv::destroyWindow("signs");
    cv::destroyWindow("gestures");
    cv::destroyWindow("raw_models");
    cv::destroyWindow("hand_gestures RAW Depth");
    cv::destroyWindow("hand_gestures RAW RGB");
    cv::destroyWindow("rgbGest");
    cv::destroyWindow("depthGest");
    cv::destroyWindow("centers tracking");


    cv::waitKey(1);
    return true;
}


bool autoPlaneSegmentation(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    autoPlaneSegmentationFlag=1;
}


bool fakeTemperature(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if (fakeTemperatureActivated) { ROS_INFO("Will use REAL temperature from now on");  fakeTemperatureActivated=0; } else
                                  { ROS_INFO("Will use FAKE temperature from now on");  fakeTemperatureActivated=1; }
    return true;
}

bool useMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if (consultHobbitMap) {  ROS_INFO("Will NOT use hobbit map from now on"); consultHobbitMap=0; } else
                          {  ROS_INFO("Will use hobbit map from now on"); consultHobbitMap=1; }
    return true;
}


bool checkMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{

    if ( mapSaysThatWhatWeAreLookingAtPossibleFallenUser(frameTimestamp) )
    {
       ROS_INFO("Map Indicates fallen user!");
    } else
    {
       ROS_INFO("Map Indicates free space!");
    }

    return true;
}


int lookingUpInternal()
{
    if (fallDetectionContext.headLookingDirection!=HEAD_LOOKING_UP)    { fprintf(stderr,"\n\nHead seems to be looking up now..!\n\n"); }
    fallDetectionContext.headLookingDirection=HEAD_LOOKING_UP;
}


int lookingCenterInternal()
{
    if (fallDetectionContext.headLookingDirection!=HEAD_LOOKING_CENTER) { fprintf(stderr,"\n\nHead seems to be looking center now..!\n\n"); }
    fallDetectionContext.headLookingDirection=HEAD_LOOKING_CENTER;
}


int lookingDownInternal()
{
    if (fallDetectionContext.headLookingDirection!=HEAD_LOOKING_DOWN) { fprintf(stderr,"\n\nHead seems to be looking down now..!\n\n"); }
    fallDetectionContext.headLookingDirection=HEAD_LOOKING_DOWN;
}

int lookingLittleDownInternal()
{
    if (fallDetectionContext.headLookingDirection!=HEAD_LOOKING_LITTLE_DOWN) { fprintf(stderr,"\n\nHead seems to be looking little down now..!\n\n"); }
    fallDetectionContext.headLookingDirection=HEAD_LOOKING_LITTLE_DOWN;
}




bool toggleTFPrinting(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if (printTFPosition) { printTFPosition=0; } else { printTFPosition=1; }
    return true;
}

bool lookingUp(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    lookingUpInternal();
    return true;
}


bool lookingDown(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    lookingDownInternal();
    return true;
}


bool lookingLittleDown(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    lookingLittleDownInternal();
    return true;
}


bool lookingCenter(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    lookingCenterInternal();
    return true;
}



bool increasePlaneDistance(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
     increasePlane();
    return true;
}

bool decreasePlaneDistance(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
     decreasePlane();
    return true;
}



void bboxReceived(const emergency_detector::SkeletonBBox & msg)
{
   processBoundingBox(msg.centerX2D , msg.centerY2D , msg.centerZ2D  ,
                      msg.width2D , msg.height2D ,msg.depth2D , frameTimestamp );
}

/*
void joints2DReceived(const emergency_detector::Skeleton2D & msg)
{
  if (msg.numberOfJoints/2 < MAX_NUMBER_OF_2D_JOINTS)
  {
    unsigned int i=0;
    fallDetectionContext.lastJointsTimestamp = fallDetectionContext.jointsTimestamp;
    fallDetectionContext.jointsTimestamp = frameTimestamp;
    fallDetectionContext.numberOfJoints = (unsigned int) msg.numberOfJoints/2;
    for (i=0; i<fallDetectionContext.numberOfJoints; i++)
    {
        fallDetectionContext.lastJoint2D[i].x = fallDetectionContext.currentJoint2D[i].x;
        fallDetectionContext.lastJoint2D[i].y = fallDetectionContext.currentJoint2D[i].y;

        fallDetectionContext.currentJoint2D[i].x = (float) msg.joints2D[i*2];
        fallDetectionContext.currentJoint2D[i].y = (float) msg.joints2D[1+(i*2)];
    }

    logSkeletonState(&fallDetectionContext,0);
  }
}



void joints3DReceived(const emergency_detector::Skeleton3D & msg)
{
  if (msg.numberOfJoints/2 < MAX_NUMBER_OF_2D_JOINTS)
  {
    unsigned int i=0;
    fallDetectionContext.lastJointsTimestamp = fallDetectionContext.jointsTimestamp;
    fallDetectionContext.jointsTimestamp = frameTimestamp;
    fallDetectionContext.numberOfJoints = (unsigned int) msg.numberOfJoints/2;
    for (i=0; i<fallDetectionContext.numberOfJoints; i++)
    {
        fallDetectionContext.lastJoint3D[i].x = fallDetectionContext.currentJoint3D[i].x;
        fallDetectionContext.lastJoint3D[i].y = fallDetectionContext.currentJoint3D[i].y;
        fallDetectionContext.lastJoint3D[i].z = fallDetectionContext.currentJoint3D[i].z;

        fallDetectionContext.currentJoint3D[i].x = (float) msg.joints3D[0+i*3];
        fallDetectionContext.currentJoint3D[i].y = (float) msg.joints3D[1+(i*3)];
        fallDetectionContext.currentJoint3D[i].z = (float) msg.joints3D[2+(i*3)];
    }

    logSkeletonState(&fallDetectionContext,1);
  }
}*/


//New Double 2D/3D receiver
void joints2D3DReceived(const emergency_detector::Skeleton2D3D & msg)
{
  if (msg.numberOfJoints/2 < MAX_NUMBER_OF_2D_JOINTS)
  {
    unsigned int i=0;
    fallDetectionContext.lastJointsTimestamp = fallDetectionContext.jointsTimestamp;
    fallDetectionContext.jointsTimestamp = frameTimestamp;
    fallDetectionContext.numberOfJoints = (unsigned int) msg.numberOfJoints/2;
    for (i=0; i<fallDetectionContext.numberOfJoints; i++)
    {
        fallDetectionContext.lastJoint2D[i].x = fallDetectionContext.currentJoint2D[i].x;
        fallDetectionContext.lastJoint2D[i].y = fallDetectionContext.currentJoint2D[i].y;

        fallDetectionContext.currentJoint2D[i].x = (float) msg.joints2D[i*2];
        fallDetectionContext.currentJoint2D[i].y = (float) msg.joints2D[1+(i*2)];


        fallDetectionContext.lastJoint3D[i].x = fallDetectionContext.currentJoint3D[i].x;
        fallDetectionContext.lastJoint3D[i].y = fallDetectionContext.currentJoint3D[i].y;
        fallDetectionContext.lastJoint3D[i].z = fallDetectionContext.currentJoint3D[i].z;

        fallDetectionContext.currentJoint3D[i].x = (float) msg.joints3D[0+i*3];
        fallDetectionContext.currentJoint3D[i].y = (float) msg.joints3D[1+(i*3)];
        fallDetectionContext.currentJoint3D[i].z = (float) msg.joints3D[2+(i*3)];
    }

    logSkeletonState(&fallDetectionContext,1);
  }
}


int updateHeadPosition()
{
 if (!useTFTree) { return 0; }

 tf::TransformListener listener;
 tf::StampedTransform transformS;
 try
     {
      //using /headcam_rgb_optical_frame and /base_link: , thanks David :)
      listener.waitForTransform("/headcam_rgb_optical_frame", "/base_link", ros::Time(0), ros::Duration(HEAD_POSITION_LOOKUP_DURATION) );
      listener.lookupTransform("/headcam_rgb_optical_frame", "/base_link",ros::Time(0), transformS);
      //can check this with : rosrun tf tf_echo "/headcam_rgb_optical_frame" "/base_link"

      double roll, pitch, yaw;
      tf::Matrix3x3(transformS.getRotation()).getRPY(roll, pitch, yaw);

      // Converts degrees to radians.
      #define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
      // Converts radians to degrees.
      #define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)
      double rollDeg=radiansToDegrees(roll);
      double pitchDeg=radiansToDegrees(pitch);
      double yawDeg=radiansToDegrees(yaw);


      if (printTFPosition)
       {
        fprintf(
                stderr,"Head Pos(%0.2f %0.2f %0.2f)/RPY(%0.2f %0.2f %0.2f)/RPYDeg(%0.2f %0.2f %0.2f)\n",
                transformS.getOrigin().x(),transformS.getOrigin().y(),transformS.getOrigin().z(),
                roll,pitch,yaw,
                rollDeg,pitchDeg,yawDeg
               );
       }

       double pitchChange=pitch-headLastPitch; if (pitchChange<0) { pitchChange = -1 * pitchChange; }
       headLastPitch=pitch;

       double rollChange=roll-headLastRoll; if (rollChange<0) { rollChange = -1 * rollChange; }
       headLastRoll=roll;

       if ( (pitchChange>0.15) || (rollChange>0.15) )  { headIsMoving=15; }


       if ( (headIsMoving>0) )
        {
         fprintf(stderr,"Head is moving ( cooldown %u )\n",headIsMoving);
         fallDetectionContext.headLookingDirection=HEAD_MOVING_FAST;
         --headIsMoving;
         return 1;
        }

      float downBorder = -1.02; // -1.10
      float upBorder = -1.35; // -1.10

      //We dont want head to be looking left or right
      if ((-3.00<roll)&&(roll<-1.40)) { fallDetectionContext.headLookingDirection=HEAD_LOOKING_RIGHT; } else
      if ((3.10>roll)&&(roll> 1.40)) { fallDetectionContext.headLookingDirection=HEAD_LOOKING_LEFT; } else
      // <-- LOW ( -1.10 ) ----   ( LOWMAX(-1.15) --- LOWMIN(-1.29) ) ----- HIGH( -1.35 )
      if ( ( pitch <= downBorder  ) && ( pitch >= upBorder  )  ) { lookingLittleDownInternal(); }  else
      if (pitch >= downBorder) /*-1.10*/                      { lookingDownInternal(); }  else
      if (pitch <= upBorder)                                { lookingCenterInternal(); }

     }
 catch (tf::TransformException &ex)
      {
       ROS_ERROR("Cannot understand head position : %s",ex.what());
       //fallDetectionContext.headLookingDirection=HEAD_UNKNOWN_DIRECTION;
      }
 return 1;
}



//RGBd Callback is called every time we get a new pair of frames , it is synchronized to the main thread
void rgbdCallbackNoCalibrationBoth(unsigned int cameraID,
                                   const sensor_msgs::Image::ConstPtr rgb_img_msg,
                                   const sensor_msgs::Image::ConstPtr depth_img_msg )
{
 if (paused) { return; } //If we are paused spend no time with new input
 //Using Intrinsic camera matrix for the raw (distorted) input images.

 if (fakeTemperatureActivated)
   {
      lastState.objectTemperature = (float) (maximums.objectTemperature - minimums.objectTemperature)  / 2;
      lastState.objectTemperature += (float) minimums.objectTemperature;
      fprintf(stderr,"Emulating temperature %0.2f ( min/max %0.2f/%0.2f ) \n",lastState.objectTemperature , minimums.objectTemperature ,maximums.objectTemperature );
      lastState.timestampTemperature=frameTimestamp;
   }

 unsigned int colorWidth = rgb_img_msg->width;   unsigned int colorHeight = rgb_img_msg->height;
 unsigned int depthWidth = depth_img_msg->width; unsigned int depthHeight = depth_img_msg->height;
 int i=0;

 cv_bridge::CvImageConstPtr orig_rgb_img;
 cv_bridge::CvImageConstPtr orig_depth_img;
 orig_rgb_img = cv_bridge::toCvShare(rgb_img_msg, "rgb8");

 if (depth_img_msg->encoding.compare("32FC1")==0)
 {
  orig_depth_img = cv_bridge::toCvShare(depth_img_msg, depth_img_msg->encoding);
  cv::Mat depth32FC1 = cv::Mat(depthHeight,depthWidth,CV_32FC1,orig_depth_img->image.data,4*depthWidth);

  cv::Mat depth16UC1;
  double scalefactor=1000; //Using meters , so go to millimeters
  depth32FC1.convertTo(depth16UC1 , CV_16UC1, scalefactor);
   //----------------------------------
    switch (cameraID)
    {
       case 0 :
          runServicesThatNeedColorAndDepth((unsigned char*) orig_rgb_img->image.data, colorWidth , colorHeight ,
                                           (unsigned short*) depth16UC1.data ,  depthWidth , depthHeight ,
                                           /*&calib*/ 0, frameTimestamp );
       break;
       case 1 :
          runServicesBottomThatNeedColorAndDepth((unsigned char*) orig_rgb_img->image.data, colorWidth , colorHeight ,
                                                 (unsigned short*) depth16UC1.data ,  depthWidth , depthHeight ,
                                                 /*&calib*/ 0, frameTimestamp );
       break;
       default :
        fprintf(stderr,"rgbdCallbackNoCalibrationBoth called with unknown camera ID\n");
       break;
    };
   //----------------------------------
 } else
 {
  //Our grabber uses millimeters , maybe I should change this
  //----------------------------------
  orig_depth_img = cv_bridge::toCvShare(depth_img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  //----------------------------------
    switch (cameraID)
    {
       case 0 :
          runServicesThatNeedColorAndDepth((unsigned char*) orig_rgb_img->image.data, colorWidth , colorHeight ,
                                           (unsigned short*) orig_depth_img->image.data ,  depthWidth , depthHeight ,
                                           /*&calib*/ 0, frameTimestamp );
            break;
       case 1 :
          runServicesBottomThatNeedColorAndDepth((unsigned char*) orig_rgb_img->image.data, colorWidth , colorHeight ,
                                                 (unsigned short*) orig_depth_img->image.data ,  depthWidth , depthHeight ,
                                                /*&calib*/ 0, frameTimestamp);
       break;
       default :
        fprintf(stderr,"rgbdCallbackNoCalibrationBoth called with unknown camera ID\n");
       break;
    };

  //----------------------------------
 }
 ++frameTimestamp;
 //After running (at least) once it is not a first run any more!
 first = false;
 return;
}


int tellWebInterfaceAboutBottomCamera()
{
 if ( (receivedBaseImages==1) || ( (receivedBaseImages>0) && (receivedBaseImages%2000==0) ) )
  {
    char commandToRun[512]={0};
    snprintf(commandToRun,512,"/bin/bash -c \"rosservice call /web_interface/signalBaseCamIsOk\"");
    int i=system(commandToRun);
    fprintf(stderr,"Signaling to web interface that basecam is receiving input..!\n");
  }
}

void rgbdCallbackTop(const sensor_msgs::Image::ConstPtr rgb_img_msg,
                        const sensor_msgs::Image::ConstPtr depth_img_msg )
{
 rgbdCallbackNoCalibrationBoth(0,rgb_img_msg,depth_img_msg);
}

void rgbdCallbackBottom(const sensor_msgs::Image::ConstPtr rgb_img_msg,
                        const sensor_msgs::Image::ConstPtr depth_img_msg )
{
  ++receivedBaseImages;
  tellWebInterfaceAboutBottomCamera();
  rgbdCallbackNoCalibrationBoth(1,rgb_img_msg,depth_img_msg);
}

unsigned int whichHobbitAreWe()
{
  char* hobbitName = getenv ("hobbit_id");
  if (hobbitName==NULL)
     {
      fprintf(stderr,"No hobbit environment variable.. \n");
      fprintf(stderr,"Please export hobbit_id=\"PT2x\" , probably running on a dev machine.. \n");
      return 0;
     }

   if (strlen(hobbitName)<4)
   {
     fprintf(stderr,"Too Short Name For a hobbit (%s).. \n",hobbitName);
     return 0;
   }


   char remember = hobbitName[3];
   if (strncmp(hobbitName,"PT2",3)!=0)
   {
     fprintf(stderr,"Malformed hobbit name , was looking for PT2 , got (%s).. \n",hobbitName);
     return 0;
   }


  unsigned int ourHobbitNumber = (unsigned int) (remember-'a');
  if (ourHobbitNumber>10) { fprintf(stderr,"We got a very large Hobbit Number ( %u )  \n" , ourHobbitNumber); return 0; }
  return 1+ourHobbitNumber;

}


unsigned int pickTemperatureProfile(unsigned int hobbitID)
{
 minimums.objectTemperature = MinMaxHumanTemperatures[hobbitID*2+0];
 maximums.objectTemperature = MinMaxHumanTemperatures[hobbitID*2+1];
}


int main(int argc, char **argv)
{
   ROS_INFO("Starting Up!!");

   try
	{
	 ROS_INFO("Initializing Emergency Detector Node");
  	 ros::init(argc, argv, "emergency_detector");
     ros::start();

     ros::NodeHandle nh;
     nhPtr = &nh;

     ros::NodeHandle private_node_handle_("~");

     std::string name;
     std::string fromTopDepthTopic , fromTopDepthTopicInfo , fromTopRGBTopic , fromTopRGBTopicInfo;
     std::string fromBottomDepthTopic , fromBottomDepthTopicInfo , fromBottomRGBTopic , fromBottomRGBTopicInfo;

     private_node_handle_.param("fromTopDepthTopic", fromTopDepthTopic, std::string("/headcam/depth_registered/image_rect"));
     private_node_handle_.param("fromTopDepthTopicInfo", fromTopDepthTopicInfo, std::string("/headcam/depth_registered/camera_info"));
     private_node_handle_.param("fromTopRGBTopic", fromTopRGBTopic, std::string("headcam/rgb/image_rect_color"));
     private_node_handle_.param("fromTopRGBTopicInfo", fromTopRGBTopicInfo, std::string("/headcam/rgb/camera_info"));

     private_node_handle_.param("fromBottomDepthTopic", fromBottomDepthTopic, std::string("/basecam/depth_registered/image_rect"));
     private_node_handle_.param("fromBottomDepthTopicInfo", fromBottomDepthTopicInfo, std::string("/basecam/depth_registered/camera_info"));
     private_node_handle_.param("fromBottomRGBTopic", fromBottomRGBTopic, std::string("basecam/rgb/image_rect_color"));
     private_node_handle_.param("fromBottomRGBTopicInfo", fromBottomRGBTopicInfo, std::string("/basecam/rgb/camera_info"));


     private_node_handle_.param("autoRecordEmergencyTriggers", autoRecordEmergencyTriggers ,  0 );
     if (autoRecordEmergencyTriggers)
     {
	   ROS_INFO("Emergencies will be auto recorded , this should be switched off in production");
       fprintf(stderr,"\n\n\n\nEmergencies will be auto recorded , this should be switched off in production \n\n\n\n");
     }


     private_node_handle_.param("maximumFrameDifferenceForTemperatureToBeRelevant", maximumFrameDifferenceForTemperatureToBeRelevant ,  10 );


     std::cerr<<"Human Temperature Range was "<<minimums.objectTemperature<<" up to "<<maximums.objectTemperature<<"\n";
     std::cerr<<"  We are hobbit #" << whichHobbitAreWe() << " \n" ;
     pickTemperatureProfile(whichHobbitAreWe());
     std::cerr<<"Human Temperature Range set to "<<minimums.objectTemperature<<" up to "<<maximums.objectTemperature<<" for our hobbit\n";

    // MinMaxHumanTemperatures[]

     private_node_handle_.param("name", name, std::string("emergency_detector"));
     private_node_handle_.param("rate", rate , int(DEFAULT_FRAME_RATE)); //11 should me optimal  less for a little less CPU Usage
     ros::Rate loop_rate(rate); //  hz should be our target performance
     ros::Rate vis_loop_rate(15); //  hz should be our target performance


     //We advertise the services we want accessible using "rosservice call *w/e*"
     ros::ServiceServer visualizeOnService      = nh.advertiseService(name+"/visualize_on" , visualizeOn);
     ros::ServiceServer visualizeOffService     = nh.advertiseService(name+"/visualize_off", visualizeOff);

     ros::ServiceServer classifyEmergencyService    = nh.advertiseService(name+"/classifyEmergency", classifyEmergency);
     ros::ServiceServer classifySafeService    = nh.advertiseService(name+"/classifySafe", classifySafe);

     ros::ServiceServer clearRecordedService    = nh.advertiseService(name+"/clearRecorded", clearRecorded);
     ros::ServiceServer saveGestureRecognitionService    = nh.advertiseService(name+"/save", save);
     ros::ServiceServer pauseGestureRecognitionService    = nh.advertiseService(name+"/pause", pause);
     ros::ServiceServer resumeGestureRecognitionService   = nh.advertiseService(name+"/resume", resume);
     ros::ServiceServer stopGestureRecognitionService     = nh.advertiseService(name+"/terminate", terminate);
     ros::ServiceServer triggerGestureRecognitionService     = nh.advertiseService(name+"/trigger", trigger);
     ros::ServiceServer fakeTemperatureGestureRecognitionService     = nh.advertiseService(name+"/fakeTemperature", fakeTemperature);
     ros::ServiceServer useMapService     = nh.advertiseService(name+"/useMap", useMap);
     ros::ServiceServer checkMapService     = nh.advertiseService(name+"/checkMap", checkMap);


     ros::ServiceServer autoPlaneSegmentationService     = nh.advertiseService(name+"/autoPlaneSegmentation", autoPlaneSegmentation);

     ros::ServiceServer lookUpService          = nh.advertiseService(name+"/looking_up" , lookingUp);
     ros::ServiceServer lookCenterService      = nh.advertiseService(name+"/looking_center" , lookingCenter);
     ros::ServiceServer lookDownService        = nh.advertiseService(name+"/looking_down" , lookingDown);
     ros::ServiceServer lookLittleDownService  = nh.advertiseService(name+"/looking_little_down" , lookingLittleDown);


     ros::ServiceServer tfPrintService        = nh.advertiseService(name+"/toggleTFPrinting" , toggleTFPrinting);


     ros::ServiceServer startHeadMotionService    = nh.advertiseService(name+"/startHeadMotion", pause);
     ros::ServiceServer stopHeadMotionService     = nh.advertiseService(name+"/stopHeadMotion", resume);


     ros::ServiceServer increasePlaneDistanceService = nh.advertiseService(name+"/increasePlaneDistance" , increasePlaneDistance);
     ros::ServiceServer decreasePlaneDistanceService = nh.advertiseService(name+"/decreasePlaneDistance" , decreasePlaneDistance);



     //Make our rostopic camera grabber
     message_filters::Synchronizer<RgbdSyncPolicy> *syncTop;

	 depth_img_sub_top = new message_filters::Subscriber<sensor_msgs::Image>(nh,fromTopDepthTopic,1);
	 depth_cam_info_sub_top = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,fromTopDepthTopicInfo,1);

	 rgb_img_sub_top = new  message_filters::Subscriber<sensor_msgs::Image>(nh,fromTopRGBTopic, 1);
	 rgb_cam_info_sub_top = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,fromTopRGBTopicInfo,1);

     syncTop = new message_filters::Synchronizer<RgbdSyncPolicy>(RgbdSyncPolicy(rate), *rgb_img_sub_top, *depth_img_sub_top); //*rgb_cam_info_sub,
	 syncTop->registerCallback(rgbdCallbackTop);


	 message_filters::Synchronizer<RgbdSyncPolicy> *syncBottom;
	 depth_img_sub_bottom = new message_filters::Subscriber<sensor_msgs::Image>(nh,fromBottomDepthTopic,1);
	 rgb_img_sub_bottom = new  message_filters::Subscriber<sensor_msgs::Image>(nh,fromBottomRGBTopic, 1);
     syncBottom = new message_filters::Synchronizer<RgbdSyncPolicy>(RgbdSyncPolicy(rate), *rgb_img_sub_bottom, *depth_img_sub_bottom); //*rgb_cam_info_sub,
	 syncBottom->registerCallback(rgbdCallbackBottom);



     //ros::Subscriber sub2D = nh.subscribe("joints2D",1000,joints2DReceived);
     //ros::Subscriber sub3D = nh.subscribe("joints3D",1000,joints3DReceived);
     ros::Subscriber sub3D = nh.subscribe("joints2D3D",1000,joints2D3DReceived);
     ros::Subscriber sub = nh.subscribe("jointsBBox",1000,bboxReceived);
     ros::Subscriber subTempAmbient = nh.subscribe("/head/tempAmbient",1000,getAmbientTemperature);
     ros::Subscriber subTempObject = nh.subscribe("/head/tempObject",1000,getObjectTemperature);
     #if BROADCAST_HOBBIT
      gestureEventBroadcaster = nh.advertise <hobbit_msgs::Event> ("Event", 1000);
      personBroadcaster = nh.advertise <emergency_detector::Person> (PERSON_TOPIC, 1000);
     #endif


     fallDetectionContext.headLookingDirection=HEAD_UNKNOWN_DIRECTION; //

    fallDetectionContext.headLookingDirection=HEAD_LOOKING_CENTER; //TODO : for now ..
     fallDetectionContext.lastJointsTimestamp=100000;
     initializeProcess(nhPtr);

      //Create our context
      //---------------------------------------------------------------------------------------------------
	  //////////////////////////////////////////////////////////////////////////
	  while ( ( key!='q' ) && (ros::ok()) )
		{

                 if (!paused)
                  {
                   if ( (emergencyDetected) && (headIsMoving!=0) )
                   {
                     emergencyDetected=0;
                     ROS_INFO("Border emergency , with moving head suppressed..!");
                   }



                   if ( (emergencyDetected) && (headIsMoving==0) )
                     {
                      if (autoRecordEmergencyTriggers)
                      {
	                    ROS_INFO("Recording Emergency Snapshot, this should be disabled on production");
	                    fprintf(stderr, "\n\n\n\nRecording emergency event this should be disabled on production\n\n\n\n" );
                        imageDir=triggersDir;
                        saveNextTopFrame=1;
                        saveNextBottomFrame=1;
                      }
                      appendClassifierData("../../web_interface/bin/emergencies/triggers/triggerlist.txt",frameTimestamp);

                      broadcastEmergency(frameTimestamp);
                     }
                   if (personDetected)   { broadcastNewPerson(); }
                   updateHeadPosition();
                  }


                  ros::spinOnce();//<- this keeps our ros node messages handled up until synergies take control of the main thread

                  if (doCVOutput) { vis_loop_rate.sleep(); } else
                                  { loop_rate.sleep();     }


                  if (frameTimestamp%30) { fprintf(stderr,"."); }

		 }



	   delete depth_img_sub_top;
	   delete depth_cam_info_sub_top;
	   delete rgb_img_sub_top;
	   delete rgb_cam_info_sub_top;

	   delete depth_img_sub_bottom;
	   delete rgb_img_sub_bottom;

	   delete syncTop;
	   delete syncBottom;
	}
	catch(std::exception &e) { ROS_ERROR("Exception: %s", e.what()); return 1; }
	catch(...)               { ROS_ERROR("Unknown Error"); return 1; }
	ROS_ERROR("Shutdown complete");
	return 0;
}
