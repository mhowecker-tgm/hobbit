#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/spinner.h>

//#include <stdexcept>
#include <image_transport/image_transport.h>

#include "process.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>


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

#include <std_msgs/Float32.h>

#define BROADCAST_HOBBIT 1

#if BROADCAST_HOBBIT
#include "hobbit_msgs/Event.h"
#include <std_msgs/String.h>
ros::Publisher gestureEventBroadcaster;
#endif


#define DEFAULT_FRAME_RATE 4

int rate=DEFAULT_FRAME_RATE;


message_filters::Subscriber<sensor_msgs::Image> *rgb_img_sub;
message_filters::Subscriber<sensor_msgs::CameraInfo> *rgb_cam_info_sub;
message_filters::Subscriber<sensor_msgs::Image> *depth_img_sub;
message_filters::Subscriber<sensor_msgs::CameraInfo> *depth_cam_info_sub;
//OpenCV
 cv::Mat rgb,depth;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RgbdSyncPolicy;

enum HEAD_LOOK_DIRECTION_ENUM
{
   HEAD_UNKNOWN_DIRECTION = 0,
   HEAD_LOOKING_DOWN     ,
   HEAD_LOOKING_CENTER  ,
   HEAD_LOOKING_UP
};



bool first=false;
int key = 0;
unsigned int headLookingDirection=HEAD_UNKNOWN_DIRECTION;
unsigned int frameTimestamp=0;
ros::NodeHandle * nhPtr=0;
unsigned int paused=0;
unsigned int fakeTemperatureActivated=0;



void broadcastEmergency(unsigned int frameNumber)
{
  if ( (!emergencyDetected) ) { return ; }

  emergencyDetected=0;
  fprintf(stderr,"emergencyDetection broadcasting disabled , just for a little while more\n");
  return ;

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
    cv::destroyWindow("emergency_detector rgb");
    cv::destroyWindow("emergency_detector depth");
    cv::destroyWindow("emergency_detector segmented depth");

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
    headLookingDirection=HEAD_LOOKING_UP;
    return true;
}


bool lookingDown(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    headLookingDirection=HEAD_LOOKING_DOWN;
    return true;
}


bool lookingCenter(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    headLookingDirection=HEAD_LOOKING_CENTER;
    return true;
}



void bboxReceived(const emergency_detector::SkeletonBBox & msg)
{
   processBoundingBox(msg.centerX2D , msg.centerY2D , msg.centerZ2D  ,
                      msg.width2D , msg.height2D ,msg.depth2D , frameTimestamp );
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
  orig_rgb_img = cv_bridge::toCvCopy(rgb_img_msg, "rgb8");
  orig_rgb_img->image.copyTo(rgb);
  orig_depth_img = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  orig_depth_img->image.copyTo(depth);

  //doDrawOut();

  if (frameTimestamp%2==0)
  { //Preserve resources

   //Emulate Human Temperature , since we don't always have the temperature sensor available
   if (fakeTemperatureActivated)
   {
      temperatureObjectDetected=36;
      tempTimestamp=frameTimestamp;
   }
   runServicesThatNeedColorAndDepth((unsigned char*) rgb.data, colorWidth , colorHeight ,
                                   (unsigned short*) depth.data ,  depthWidth , depthHeight ,
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

     //Make our rostopic camera grabber
     message_filters::Synchronizer<RgbdSyncPolicy> *sync;

	 depth_img_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,fromDepthTopic,1);
	 depth_cam_info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,fromDepthTopicInfo,1);

	 rgb_img_sub = new  message_filters::Subscriber<sensor_msgs::Image>(nh,fromRGBTopic, 1);
	 rgb_cam_info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,fromRGBTopicInfo,1);

     sync = new message_filters::Synchronizer<RgbdSyncPolicy>(RgbdSyncPolicy(rate), *rgb_img_sub, *depth_img_sub); //*rgb_cam_info_sub,
	 sync->registerCallback(rgbdCallbackNoCalibration);

     ros::Subscriber sub = nh.subscribe("jointsBBox",1000,bboxReceived);
     ros::Subscriber subTempAmbient = nh.subscribe("/head/tempAmbient",1000,getAmbientTemperature);
     ros::Subscriber subTempObject = nh.subscribe("/head/tempObject",1000,getObjectTemperature);
     #if BROADCAST_HOBBIT
      gestureEventBroadcaster = nh.advertise <hobbit_msgs::Event> ("Event", 1000);
     #endif

     initializeProcess();

      //Create our context
      //---------------------------------------------------------------------------------------------------
	  //////////////////////////////////////////////////////////////////////////
	  while ( ( key!='q' ) && (ros::ok()) )
		{
                  ros::spinOnce();//<- this keeps our ros node messages handled up until synergies take control of the main thread

                  if (doCVOutput) { vis_loop_rate.sleep(); } else
                                  { loop_rate.sleep();     }

                  if(emergencyDetected)
                      { broadcastEmergency(frameTimestamp); }

                  if (frameTimestamp%20) { fprintf(stderr,"."); }
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
