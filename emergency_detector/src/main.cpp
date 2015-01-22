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
#include "emergency_detector/Skeleton2D.h"
#include "emergency_detector/Skeleton3D.h"
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


#define DEFAULT_FRAME_RATE 4

int rate=DEFAULT_FRAME_RATE;


message_filters::Subscriber<sensor_msgs::Image> *rgb_img_sub;
message_filters::Subscriber<sensor_msgs::CameraInfo> *rgb_cam_info_sub;
message_filters::Subscriber<sensor_msgs::Image> *depth_img_sub;
message_filters::Subscriber<sensor_msgs::CameraInfo> *depth_cam_info_sub;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RgbdSyncPolicy;

unsigned int lastEmergencyDetectionTimestamp=0;
unsigned int emergencyDetectionCooldown=150; //This should be time , not frames


bool first=false;
int key = 0;
unsigned int frameTimestamp=emergencyDetectionCooldown+1; //not 0 so we can immediately trigger
ros::NodeHandle * nhPtr=0;
unsigned int paused=0;
unsigned int dontPublishPersons=0;
unsigned int useTFTree=0;
unsigned int fakeTemperatureActivated=0;


void broadcastNewPerson()
{
  if (dontPublishPersons) { return ; }
  personDetected=0;

  emergency_detector::Person msg;
  msg.x = temperatureX;
  msg.y = temperatureY;
  msg.z = temperatureZ;;
  msg.source = 5; //5 = temperature / emergency
  msg.theta = 0;

  msg.inFieldOfView = 1;
  msg.confidence = 0.5;
  msg.timestamp=frameTimestamp;

  fprintf(stderr, "Publishing a new Person @ %0.2f %0.2f %0.2f\n" ,temperatureX,temperatureY,temperatureZ);
  personBroadcaster.publish(msg);
  ros::spinOnce();
}



void broadcastEmergency(unsigned int frameNumber)
{
  if ( (!emergencyDetected) ) { return ; }


  if ( frameNumber > lastEmergencyDetectionTimestamp  + emergencyDetectionCooldown )
  {
    fprintf(stderr,"Throttling Emergency Event \n");
  }


  //emergencyDetected=0;
  //fprintf(stderr,"emergencyDetection broadcasting disabled , just for a little while more\n");
  //return ;

  #if BROADCAST_HOBBIT
    hobbit_msgs::Event evt;
    std::stringstream ss;
     //case GESTURE_NONE   : break;
     if (emergencyDetected) { ss<<"G_FALL"; } else
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
    temperatureAmbientDetected=request->data;
    return;
}


void getObjectTemperature(const std_msgs::Float32::ConstPtr& request)
{
    temperatureObjectDetected=request->data;
    tempTimestamp=frameTimestamp;
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

bool fakeTemperature(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if (fakeTemperatureActivated) { fakeTemperatureActivated=0; } else
                                  { fakeTemperatureActivated=1; }
    return true;
}


bool lookingUp(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    fallDetectionContext.headLookingDirection=HEAD_LOOKING_UP;
    return true;
}


bool lookingDown(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    fallDetectionContext.headLookingDirection=HEAD_LOOKING_DOWN;
    return true;
}


bool lookingCenter(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    fallDetectionContext.headLookingDirection=HEAD_LOOKING_CENTER;
    return true;
}



bool setHobbitE(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
     setHobbitEMode();
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
        fallDetectionContext.currentJoint3D[i].y = (float) msg.joints3D[2+(i*3)];
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
      listener.lookupTransform("/map", "/frame",ros::Time(0), transformS);
      fprintf(stderr,"Head At %0.2f %0.2f %0.2f \n" ,  transformS.getOrigin().x() ,  transformS.getOrigin().y() ,  transformS.getOrigin().z());
     }
 catch (tf::TransformException &ex)
      {
       ROS_ERROR("%s",ex.what());
       //ros::Duration(1.0).sleep();
      }
 return 1;
}


//RGBd Callback is called every time we get a new pair of frames , it is synchronized to the main thread
void rgbdCallbackNoCalibration(const sensor_msgs::Image::ConstPtr rgb_img_msg,
                                 const sensor_msgs::Image::ConstPtr depth_img_msg  )
{
 if (paused) { return; } //If we are paused spend no time with new input
  //A new pair of frames has arrived , copy and convert them so that they are ready
  unsigned int colorWidth = rgb_img_msg->width;   unsigned int colorHeight = rgb_img_msg->height;
  unsigned int depthWidth = depth_img_msg->width; unsigned int depthHeight = depth_img_msg->height;


 cv_bridge::CvImageConstPtr orig_rgb_img;
 cv_bridge::CvImageConstPtr orig_depth_img;
 orig_rgb_img = cv_bridge::toCvShare(rgb_img_msg, "rgb8");
 orig_depth_img = cv_bridge::toCvShare(depth_img_msg, sensor_msgs::image_encodings::TYPE_16UC1);

  //doDrawOut();

  if (frameTimestamp%2==0)
  { //Preserve resources

   //Emulate Human Temperature , since we don't always have the temperature sensor available
   if (fakeTemperatureActivated)
   {
      temperatureObjectDetected=36;
      tempTimestamp=frameTimestamp;
   }
   runServicesThatNeedColorAndDepth((unsigned char*) orig_rgb_img->image.data, colorWidth , colorHeight ,
                                    (unsigned short*) orig_depth_img->image.data ,  depthWidth , depthHeight ,
                                     0 , frameTimestamp );


  }
 //After running (at least) once it is not a first run any more!
 first = false;
 ++frameTimestamp;
return;
}

int main(int argc, char **argv)
{
   ROS_INFO("Starting Up!!");

   try
	{
	 ROS_INFO("Initializing ROS");
  	 ros::init(argc, argv, "emergency_detector");
     ros::start();

     ros::NodeHandle nh;
     nhPtr = &nh;

     ros::NodeHandle private_node_handle_("~");

     std::string name;
     std::string fromDepthTopic;
     std::string fromDepthTopicInfo;
     std::string fromRGBTopic;
     std::string fromRGBTopicInfo;


     private_node_handle_.param("fromDepthTopic", fromDepthTopic, std::string("/headcam/depth_registered/image_rect"));
     private_node_handle_.param("fromDepthTopicInfo", fromDepthTopicInfo, std::string("/headcam/depth_registered/camera_info"));
     private_node_handle_.param("fromRGBTopic", fromRGBTopic, std::string("headcam/rgb/image_rect_color"));
     private_node_handle_.param("fromRGBTopicInfo", fromRGBTopicInfo, std::string("/headcam/rgb/camera_info"));


     private_node_handle_.param("maximumFrameDifferenceForTemperatureToBeRelevant", maximumFrameDifferenceForTemperatureToBeRelevant ,  10 );
     private_node_handle_.param("minimumAllowedHolePercentage", minimumAllowedHolePercentage ,  15 );
     private_node_handle_.param("maximumAllowedHolePercentage", maximumAllowedHolePercentage ,  75 );
     private_node_handle_.param("minHumanTemperature", minHumanTemperature , double (31.5) );
     private_node_handle_.param("maxHumanTemperature", maxHumanTemperature , double (37.0) );
     private_node_handle_.param("tempZoneWidth", tempZoneWidth ,  300 );
     private_node_handle_.param("tempZoneHeight", tempZoneHeight ,  200 );
     private_node_handle_.param("minScoreTrigger", minScoreTrigger ,  1600 );
     private_node_handle_.param("maxScoreTrigger", maxScoreTrigger ,  2000 );

     std::cerr<<"Human Temperature Range set to "<<minHumanTemperature<<" up to "<<maxHumanTemperature<<"\n";

     private_node_handle_.param("name", name, std::string("emergency_detector"));
     private_node_handle_.param("rate", rate , int(DEFAULT_FRAME_RATE)); //11 should me optimal  less for a little less CPU Usage
     ros::Rate loop_rate(rate); //  hz should be our target performance
     ros::Rate vis_loop_rate(15); //  hz should be our target performance


     //We advertise the services we want accessible using "rosservice call *w/e*"
     ros::ServiceServer visualizeOnService      = nh.advertiseService(name+"/visualize_on" , visualizeOn);
     ros::ServiceServer visualizeOffService     = nh.advertiseService(name+"/visualize_off", visualizeOff);
     ros::ServiceServer pauseGestureRecognitionService    = nh.advertiseService(name+"/pause", pause);
     ros::ServiceServer resumeGestureRecognitionService   = nh.advertiseService(name+"/resume", resume);
     ros::ServiceServer stopGestureRecognitionService     = nh.advertiseService(name+"/terminate", terminate);
     ros::ServiceServer triggerGestureRecognitionService     = nh.advertiseService(name+"/trigger", trigger);
     ros::ServiceServer fakeTemperatureGestureRecognitionService     = nh.advertiseService(name+"/fakeTemperature", fakeTemperature);

     ros::ServiceServer lookUpService          = nh.advertiseService(name+"/looking_up" , lookingUp);
     ros::ServiceServer lookCenterService      = nh.advertiseService(name+"/looking_center" , lookingCenter);
     ros::ServiceServer lookDownService        = nh.advertiseService(name+"/looking_down" , lookingDown);

     ros::ServiceServer setHobbitEService        = nh.advertiseService(name+"/setHobbitE" , setHobbitE);

     ros::ServiceServer increasePlaneDistanceService = nh.advertiseService(name+"/increasePlaneDistance" , increasePlaneDistance);
     ros::ServiceServer decreasePlaneDistanceService = nh.advertiseService(name+"/decreasePlaneDistance" , decreasePlaneDistance);



     //Make our rostopic camera grabber
     message_filters::Synchronizer<RgbdSyncPolicy> *sync;

	 depth_img_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,fromDepthTopic,1);
	 depth_cam_info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,fromDepthTopicInfo,1);

	 rgb_img_sub = new  message_filters::Subscriber<sensor_msgs::Image>(nh,fromRGBTopic, 1);
	 rgb_cam_info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,fromRGBTopicInfo,1);

     sync = new message_filters::Synchronizer<RgbdSyncPolicy>(RgbdSyncPolicy(rate), *rgb_img_sub, *depth_img_sub); //*rgb_cam_info_sub,
	 sync->registerCallback(rgbdCallbackNoCalibration);

     ros::Subscriber sub2D = nh.subscribe("joints2D",1000,joints2DReceived);
     ros::Subscriber sub3D = nh.subscribe("joints3D",1000,joints3DReceived);
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
     initializeProcess();

      //Create our context
      //---------------------------------------------------------------------------------------------------
	  //////////////////////////////////////////////////////////////////////////
	  while ( ( key!='q' ) && (ros::ok()) )
		{
                  ros::spinOnce();//<- this keeps our ros node messages handled up until synergies take control of the main thread

                  if (doCVOutput) { vis_loop_rate.sleep(); } else
                                  { loop_rate.sleep();     }

                  if(emergencyDetected) { broadcastEmergency(frameTimestamp); }
                  if (personDetected)   { broadcastNewPerson(); }

                  if (frameTimestamp%20) { fprintf(stderr,"."); }

                  updateHeadPosition();
		 }



	   delete depth_img_sub;
	   delete depth_cam_info_sub;
	   delete rgb_img_sub;
	   delete rgb_cam_info_sub;
	   delete sync;
	}
	catch(std::exception &e) { ROS_ERROR("Exception: %s", e.what()); return 1; }
	catch(...)               { ROS_ERROR("Unknown Error"); return 1; }
	ROS_ERROR("Shutdown complete");
	return 0;
}
