#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/spinner.h>

#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>

#include <std_srvs/Empty.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include "face_detection/SetQuality.h"
#include "calibration.h"
#include "pose.h"
#include "services.h"



//This will make this node also register to color/depth calibrations and
//pass them to the gesture node instead of the defaults
#define USE_NONDEFAULT_CALIBRATIONS 1

int rate=11;
int first=0;
int key = 0;
volatile int paused = 0;
unsigned int frameTimestamp =0;
unsigned int colorWidth = 640 , colorHeight =480 , depthWidth = 640 , depthHeight = 480;

struct calibration calib={0};

//sensor_msgs::CameraInfo camInfo;

#if USE_NONDEFAULT_CALIBRATIONS
 typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> RgbdSyncPolicy;
#else
 typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RgbdSyncPolicy;
#endif

//RGB/DEPTH Subscribers
message_filters::Subscriber<sensor_msgs::Image> *rgb_img_sub;
message_filters::Subscriber<sensor_msgs::CameraInfo> *rgb_cam_info_sub;
message_filters::Subscriber<sensor_msgs::Image> *depth_img_sub;
message_filters::Subscriber<sensor_msgs::CameraInfo> *depth_cam_info_sub;
//OpenCV
cv::Mat rgb,depth;
//----------------------------------------------------------

bool visualizeOn(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    return true;
}

bool visualizeOff(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    cv::destroyAllWindows();
    cv::waitKey(1);
    return true;
}

bool terminate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Stopping Hand Gestures");
    exit(0);
    return true;
}


bool pause(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Hand Gestures are now paused");
    paused =1;
    return true;
}

bool resume(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Hand Gestures are now resuming");
    paused =0;
    return true;
}



bool setQuality( face_detection::SetQuality::Request  &request,
                 face_detection::SetQuality::Response &response )
{
   //request.quality <- the quality setting requested
   return false;
}

int doDrawOut()
{
     /*Don't add this on output ( reduce cluttering )
     cv::Mat rgbTmp = rgb.clone();
     //Take care of drawing stuff as visual output
	 cv::Mat bgrMat,rgbMat(colorHeight,colorWidth,CV_8UC3,rgbTmp.data,3*colorWidth);
	 cv::cvtColor(rgbMat,bgrMat, CV_RGB2BGR);// opencv expects the image in BGR format
     cv::Mat depthNorm;
	 cv::normalize(depth,depthNorm,0,255,CV_MINMAX,CV_8UC1);

     //After we have our bgr Frame ready and we added the FPS text , lets show it!
	 cv::imshow("hand_gestures  RAW Depth",depthNorm);
	 cv::imshow("hand_gestures RAW RGB",bgrMat);
	 cv::waitKey(1);*/

 return 1;
}


#if USE_NONDEFAULT_CALIBRATIONS
//RGBd Callback is called every time we get a new pair of frames , it is synchronized to the main thread
void rgbdCallback(const sensor_msgs::Image::ConstPtr rgb_img_msg,
                    const sensor_msgs::Image::ConstPtr depth_img_msg,
                     const sensor_msgs::CameraInfo::ConstPtr camera_info_msg )
{
 if (paused) { return; } //If we are paused spend no time with new input
 //Using Intrinsic camera matrix for the raw (distorted) input images.

 colorWidth = rgb_img_msg->width;   colorHeight = rgb_img_msg->height;
 depthWidth = depth_img_msg->width; depthHeight = depth_img_msg->height;
 int i=0;
 calib.width = colorWidth; calib.height = colorHeight;
 for (i=0; i<9; i++) { calib.intrinsic[i]=camera_info_msg->K[i]; }
 //This hangs -> for (i=0; i<5; i++) { calib.intrinsic[i]=camera_info_msg->D[i]; }
 //TODO maybe populate calib.extrinsics here

  //A new pair of frames has arrived , copy and convert them so that they are ready
 cv_bridge::CvImageConstPtr orig_rgb_img;
 cv_bridge::CvImageConstPtr orig_depth_img;
 orig_rgb_img = cv_bridge::toCvCopy(rgb_img_msg, "rgb8");
 orig_rgb_img->image.copyTo(rgb);
 orig_depth_img = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
 orig_depth_img->image.copyTo(depth);

  runServicesThatNeedColorAndDepth((unsigned char*) rgb.data, colorWidth , colorHeight ,
                                   (unsigned short*) depth.data ,  depthWidth , depthHeight ,
                                     &calib , frameTimestamp );

 doDrawOut();
 //After running (at least) once it is not a first run any more!
 first = false;
 return;
}
#else
//RGBd Callback is called every time we get a new pair of frames , it is synchronized to the main thread
void rgbdCallbackNoCalibration(const sensor_msgs::Image::ConstPtr rgb_img_msg,
                                 const sensor_msgs::Image::ConstPtr depth_img_msg  )
{
 if (paused) { return; } //If we are paused spend no time with new input
  //A new pair of frames has arrived , copy and convert them so that they are ready
  colorWidth = rgb_img_msg->width;   colorHeight = rgb_img_msg->height;
  depthWidth = depth_img_msg->width; depthHeight = depth_img_msg->height;

  cv_bridge::CvImageConstPtr orig_rgb_img;
  cv_bridge::CvImageConstPtr orig_depth_img;
  orig_rgb_img = cv_bridge::toCvCopy(rgb_img_msg, "rgb8");
  orig_rgb_img->image.copyTo(rgb);
  orig_depth_img = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  orig_depth_img->image.copyTo(depth);

  doDrawOut();
  runServicesThatNeedColorAndDepth((unsigned char*) rgb.data, colorWidth , colorHeight ,
                                   (unsigned short*) depth.data ,  depthWidth , depthHeight ,
                                     0 , frameTimestamp );
 //After running (at least) once it is not a first run any more!
 first = false;
return;
}
#endif

int main(int argc, char **argv)
{
   ROS_INFO("Starting Up!!");
   try
	{
	 char regName[128]={0};
	 sprintf(regName,"hand_gestures%u",getpid()); //We use pid on node name
  	 ros::init(argc, argv, regName);
     ros::start();



     ros::NodeHandle nh;
     ros::NodeHandle private_node_handle_("~");

     std::string name;
     std::string frame;
     std::string fromDepthTopic;
     std::string fromDepthTopicInfo;
     std::string fromRGBTopic;
     std::string fromRGBTopicInfo;


     private_node_handle_.param("fromDepthTopic", fromDepthTopic, std::string("/headcam/depth_registered/image_rect"));
     private_node_handle_.param("fromDepthTopicInfo", fromDepthTopicInfo, std::string("/headcam/depth_registered/camera_info"));
     private_node_handle_.param("fromRGBTopic", fromRGBTopic, std::string("headcam/rgb/image_rect_color"));
     private_node_handle_.param("fromRGBTopicInfo", fromRGBTopicInfo, std::string("/headcam/rgb/camera_info"));
     private_node_handle_.param("name", name, std::string("hand_gestures"));
     private_node_handle_.param("frame", frame, std::string("frame"));
     private_node_handle_.param("rate", rate, int(11));
     ros::Rate loop_rate(rate); //  hz should be our target performance


     //Pass root frame for TF poses
     strcpy(tfRoot,frame.c_str());


     std::cout<<"face_detection Starting settings ----------------"<<std::endl;
     std::cout<<"Name : "<<name<<std::endl;
     std::cout<<"Frame : "<<frame<<std::endl;
     std::cout<<"Rate : "<<rate<<std::endl;
     std::cout<<"RGB : "<<fromRGBTopic<<" calibs : "<<fromRGBTopicInfo<<std::endl;
     std::cout<<"Depth : "<<fromDepthTopic<<" calibs : "<<fromDepthTopicInfo<<std::endl;
     std::cout<<"--------------------------------------------------"<<std::endl;



     //We advertise the services we want accessible using "rosservice call *w/e*"
     ros::ServiceServer visualizeOnService      = nh.advertiseService(name+"/visualize_on" , visualizeOn);
     ros::ServiceServer visualizeOffService     = nh.advertiseService(name+"/visualize_off", visualizeOff);
     ros::ServiceServer terminateService        = nh.advertiseService(name+"/terminate"    , terminate);
     ros::ServiceServer resumeService           = nh.advertiseService(name+"/pause"        , pause);
     ros::ServiceServer pauseService            = nh.advertiseService(name+"/resume"       , resume);
     ros::ServiceServer setQualityService       = nh.advertiseService(name+"/set_quality"  , setQuality);

     //Make our rostopic cmaera grabber
     message_filters::Synchronizer<RgbdSyncPolicy> *sync;

	 depth_img_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,fromDepthTopic,1);
	 depth_cam_info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,fromDepthTopicInfo,1);

	 rgb_img_sub = new  message_filters::Subscriber<sensor_msgs::Image>(nh,fromRGBTopic, 1);
	 rgb_cam_info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,fromRGBTopicInfo,1);

     #if USE_NONDEFAULT_CALIBRATIONS
	   sync = new message_filters::Synchronizer<RgbdSyncPolicy>(RgbdSyncPolicy(rate), *rgb_img_sub, *depth_img_sub,*depth_cam_info_sub); //*rgb_cam_info_sub,
 	   sync->registerCallback(rgbdCallback);
     #else
       sync = new message_filters::Synchronizer<RgbdSyncPolicy>(RgbdSyncPolicy(rate), *rgb_img_sub, *depth_img_sub); //*rgb_cam_info_sub,
	   sync->registerCallback(rgbdCallbackNoCalibration);
    #endif


   ROS_INFO("Registering services !!");
     registerServices(&nh,640,480);

      ROS_INFO("Entering Spin loop!!");
	  while ( ( key!='q' ) && (ros::ok()) )
		{
          ros::spinOnce();
          loop_rate.sleep();
		 }
      ROS_INFO("Stopping Spin loop!!");

	   stopServices();
	}
	catch(std::exception &e) { ROS_ERROR("Exception: %s", e.what()); return 1; }
	catch(...)               { ROS_ERROR("Unknown Error"); return 1; }
	ROS_ERROR("Shutdown complete");
	return 0;
}
