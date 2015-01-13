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


#include "PeopleTracker.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include "services.h"
#include "pose.h"



//This will make this node also register to color/depth calibrations and
//pass them to the gesture node instead of the defaults
#define MAX_RECORDED_FRAMES 1000


#define ABSDIFF(num1,num2) ( (num1-num2) >=0 ? (num1-num2) : (num2 - num1) )

int rate=11;
int first=0;
int key = 0;

int recording=0;
int recordedFrames=0;
volatile int paused = 1; //Follow User starts paused from now on , after Paloma's request 12/01/15..
unsigned int frameTimestamp =0;
unsigned int runFullSpeed=0;
unsigned int runMaxSpeed=0;
unsigned int colorWidth = 640 , colorHeight =480 , depthWidth = 640 , depthHeight = 480;

//#define USE_BASECAM 1

#if USE_BASECAM
#define depthTopic "/basecam/depth_registered/image_rect"
#define depthTopicInfo "/basecam/depth_registered/camera_info"
#define rgbTopic "/basecam/rgb/image_rect_color"
#define rgbTopicInfo "/basecam/rgb/camera_info"
#else
#define depthTopic "/headcam/depth_registered/image_rect"
#define depthTopicInfo "/headcam/depth_registered/camera_info"
#define rgbTopic "/headcam/rgb/image_rect_color"
#define rgbTopicInfo "/headcam/rgb/camera_info"
#endif // USE_BASECAM

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> RgbdSyncPolicy;

//RGB/DEPTH Subscribers
message_filters::Subscriber<sensor_msgs::Image> *rgb_img_sub;
message_filters::Subscriber<sensor_msgs::CameraInfo> *rgb_cam_info_sub;
message_filters::Subscriber<sensor_msgs::Image> *depth_img_sub;
message_filters::Subscriber<sensor_msgs::CameraInfo> *depth_cam_info_sub;

//----------------------------------------------------------


bool visualizeOn(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    internalSetVisualization(1);
    runFullSpeed=1;
    return true;
}

bool visualizeOff(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    internalSetVisualization(0);
    runFullSpeed=0;
    return true;
}

bool terminate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Stopping Follow User");
    exit(0);
    return true;
}

bool pause(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Follow User is now paused");
    paused =1;
    return true;
}

bool resume(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Follow User is now resuming");
    paused =0;

    resumeServices();
    return true;
}


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

 //DISABLED until i have a calib structure again
 //calib.width = colorWidth; calib.height = colorHeight;
 //for (i=0; i<9; i++) { calib.intrinsic[i]=camera_info_msg->K[i]; }
 //DISABLED until i have a calib structure again

 //This hangs -> for (i=0; i<5; i++) { calib.intrinsic[i]=camera_info_msg->D[i]; }
 //TODO maybe populate calib.extrinsics here

  //A new pair of frames has arrived , copy and convert them so that they are ready
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
    runServicesThatNeedColorAndDepth((unsigned char*) orig_rgb_img->image.data, colorWidth , colorHeight ,
                                     (unsigned short*) depth16UC1.data ,  depthWidth , depthHeight ,
                                      /*&calib*/ 0, frameTimestamp );
   //----------------------------------
 } else
 {
  //Our grabber uses millimeters , maybe I should change this
  //----------------------------------
  orig_depth_img = cv_bridge::toCvShare(depth_img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  //----------------------------------
  runServicesThatNeedColorAndDepth((unsigned char*) orig_rgb_img->image.data, colorWidth , colorHeight ,
                                   (unsigned short*) orig_depth_img->image.data ,  depthWidth , depthHeight ,
                                     /*&calib*/ 0, frameTimestamp );
  //----------------------------------
 }
 ++frameTimestamp;
 //After running (at least) once it is not a first run any more!
 first = false;
 return;
}


int main(int argc, char **argv)
{
   ROS_INFO("Starting Up !!");
   try
    {
     ros::init(argc, argv, "follow_user");
     ros::start();

     ros::NodeHandle nh;
     ros::NodeHandle private_node_handle_("~");

     std::string name;
     std::string fromDepthTopic;
     std::string fromDepthTopicInfo;
     std::string fromRGBTopic;
     std::string fromRGBTopicInfo;
     std::string tfRootStr;


     private_node_handle_.param("fromDepthTopic", fromDepthTopic, std::string( depthTopic ));
     private_node_handle_.param("fromDepthTopicInfo", fromDepthTopicInfo, std::string( depthTopicInfo ));
     private_node_handle_.param("fromRGBTopic", fromRGBTopic, std::string( rgbTopic ));
     private_node_handle_.param("fromRGBTopicInfo", fromRGBTopicInfo, std::string( rgbTopicInfo ));
     private_node_handle_.param("tfRoot", tfRootStr, std::string( tfRoot ));
     strncpy(tfRoot,tfRootStr.c_str(),TF_ROOT_NAME_SIZE); //Respect chosen TFRoot
     private_node_handle_.param("name", name, std::string("follow_user"));
     private_node_handle_.param("rate", rate, int(20));
     ros::Rate loop_rate_ultra_low(1); //  hz should be our target performance
     ros::Rate loop_rate(rate); //  hz should be our target performance
     unsigned int fastRate = rate*2;
     if (fastRate > 30) { fastRate=30; } //Cap fast speed at 30Hz ( i.e. frame rate of Depth camera )
     ros::Rate loop_rate_fast(fastRate); //  hz should be our target performance
     ros::Rate loop_rate_fullSpeed(rate); //  hz should be our target performance

     //We advertise the services we want accessible using "rosservice call *w/e*"
     ros::ServiceServer visualizeOnService      = nh.advertiseService(name+"/visualize_on" , visualizeOn);
     ros::ServiceServer visualizeOffService     = nh.advertiseService(name+"/visualize_off", visualizeOff);
     ros::ServiceServer terminateService        = nh.advertiseService(name+"/terminate"    , terminate);
     ros::ServiceServer resumeService           = nh.advertiseService(name+"/pause"        , pause);
     ros::ServiceServer pauseService            = nh.advertiseService(name+"/resume"       , resume);

     //Make our rostopic cmaera grabber
     message_filters::Synchronizer<RgbdSyncPolicy> *sync;

     std::cerr<<"\n\n\nfollow_user , RGB feed "<<fromRGBTopic<<" \n";
     std::cerr<<"follow_user , RGB Info "<<fromRGBTopicInfo<<" \n";
     std::cerr<<"follow_user , Depth feed "<<fromDepthTopic<<" \n";
     std::cerr<<"follow_user , Depth Info "<<fromDepthTopicInfo<<" \n";
     std::cerr<<"follow_user , TF Root "<<tfRootStr<<" ( "<<tfRoot<<") \n";
     std::cerr<<"frame rate set to : "<<rate<<" \n\n\n\n";

	 depth_img_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,fromDepthTopic,1);
	 depth_cam_info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,fromDepthTopicInfo,1);

	 rgb_img_sub = new  message_filters::Subscriber<sensor_msgs::Image>(nh,fromRGBTopic, 1);
	 rgb_cam_info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,fromRGBTopicInfo,1);

     std::cerr<<"Done\n";

	 sync = new message_filters::Synchronizer<RgbdSyncPolicy>(RgbdSyncPolicy(rate), *rgb_img_sub, *depth_img_sub,*depth_cam_info_sub); //*rgb_cam_info_sub,
     sync->registerCallback(rgbdCallback);


     registerServices(&nh,640,480);

	  while ( ( key!='q' ) && (ros::ok()) )
		{
          ros::spinOnce();

            unsigned int lastDetectedFrame = ABSDIFF(frameTimestamp,actualTimestamp);

           if (runMaxSpeed)
               {
                 //No Sleep at all
               } else
            if (runFullSpeed)
                {
                  loop_rate_fullSpeed.sleep();
                } else
            {  loop_rate.sleep(); }

		 }

	   stopServices();

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
