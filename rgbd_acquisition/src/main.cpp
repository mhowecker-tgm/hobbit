

#define EMMIT_CALIBRATION 1



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdexcept>
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/spinner.h>

#include <opencv2/opencv.hpp>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>


#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>


#include <image_transport/image_transport.h>

#include "OpenNI2Acquisition.h"
#include "pose.h"
#include "services.h"

#include "rgbd_acquisition/SetQuality.h"

#include "extAcquisition.h"


#define NODE_NAME "rgbd_acquisition"


//These are the static declarations of the various parts of this ROS package
int key = 0;
double  virtual_baseline=0.0; //This is 0 and should be zero since we have a registered depth/rgb stream , however it can be changed to allow fake disparity to be generated
volatile int paused = 0;

ros::NodeHandle * nhPtr=0;
image_transport::Publisher pubRGB;
image_transport::Publisher pubDepth;

ros::Publisher pubRGBInfo;
ros::Publisher pubDepthInfo;


//----------------------------------------------------------
//Advertised Service switches
bool visualizeOn(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    switchDrawOutTo(1);
    return true;
}

bool visualizeOff(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    switchDrawOutTo(0);
    return true;
}


bool trigger(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    actualConfidence=0.5;
    broadcastNewPerson();
    return true;
}

bool terminate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Stopping RGBDAcquisition");
    exit(0);
    return true;
}


bool pause(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    pauseFaceDetection = 1;
    pauseSkeletonDetection = 1;
    return true;
}

bool resume(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    pauseFaceDetection = 0;
    pauseSkeletonDetection = 0;
    return true;
}



bool pausePointingMessages(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    dontPublishPointEvents = 1;
    return true;
}

bool resumePointingMessages(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    dontPublishPointEvents = 0;
    return true;
}




bool setQuality( rgbd_acquisition::SetQuality::Request  &request,
                  rgbd_acquisition::SetQuality::Response &response )
{
   //request.quality <- the quality setting requested
   if (request.quality>=1.0) { faceDetectionFramesBetweenScans=5; skeletonDetectionFramesBetweenScans=0; }
   if (request.quality=0.0) {  pauseFaceDetection = 1; pauseSkeletonDetection = 1; } else
   {
    //Ok we have 25 frames per second so for 10 seconds we have 250
    float fdQuality = 5/request.quality;
    float skQuality = 1/request.quality;
    faceDetectionFramesBetweenScans=(unsigned int ) fdQuality;
    skeletonDetectionFramesBetweenScans=(unsigned int ) skQuality;
   }

   return true ;
}

bool publishImagesFrames(unsigned char * color , unsigned int colorWidth , unsigned int colorHeight ,
                          unsigned short * depth , unsigned int depthWidth , unsigned int depthHeight ,
                           struct calibration  * calib
                         )
{
  //convert & publish RGB Stream - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  IplImage *imageRGB = cvCreateImageHeader( cvSize(colorWidth,colorHeight), IPL_DEPTH_8U ,3);
  imageRGB->imageData = (char *) color;

   cv_bridge::CvImage out_RGB_msg;
   //out_RGB_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
   out_RGB_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
   out_RGB_msg.image    = imageRGB; // Your cv::Mat

   ros::Time sampleTime = ros::Time::now();


   out_RGB_msg.header.frame_id= tfRoot;
   out_RGB_msg.header.stamp= sampleTime;
   pubRGB.publish(out_RGB_msg.toImageMsg());


  //convert & publish Depth Stream - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  IplImage *imageDepth = cvCreateImageHeader( cvSize(depthWidth,depthHeight), IPL_DEPTH_16U ,1);
  imageDepth->imageData = (char *) depth;

   cv_bridge::CvImage out_Depth_msg;
   //out_Depth_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
   out_Depth_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1; // Or whatever
   out_Depth_msg.image    = imageDepth; // Your cv::Mat

   out_Depth_msg.header.frame_id= tfRoot;
   out_Depth_msg.header.stamp= sampleTime;
   pubDepth.publish(out_Depth_msg.toImageMsg());


   //---------------------------------------------------------------------------------------------------
   //  CAMERA INFORMATION BROADCAST --------------------------------------------------------------------
   //---------------------------------------------------------------------------------------------------
   #if EMMIT_CALIBRATION
   if (calib!=0)
   {
    int i=0;

    sensor_msgs::CameraInfo cal;
    cal.header.stamp = sampleTime;
    cal.width=colorWidth; cal.height=colorHeight;

    cal.D.resize(5, 0.0);
    cal.distortion_model=sensor_msgs::distortion_models::PLUMB_BOB;
    cal.D[0]=calib->k1; cal.D[1]=calib->k2; cal.D[2]=calib->p1; cal.D[3]=calib->p2; cal.D[4]=calib->k3;

    for (i=0; i<9; i++) { cal.K[i]=calib->intrinsic[i]; }

    cal.R[0]=1.0; cal.R[1]=0.0; cal.R[2]=0.0;
    cal.R[3]=0.0; cal.R[4]=1.0; cal.R[5]=0.0;
    cal.R[6]=0.0; cal.R[7]=0.0; cal.R[8]=1.0;

    for (i=0; i<12; i++) { cal.P[i]=0.0; }

    //Base line for fake disparity !
    //P[3] should be 0.0 since we have a registered feed but we add a baseline for fake disparity here
    //Please note that the virtual baseline is multiplied by -1 and fX to fit the "ROS" way of
    //declaring a baseline as seen here https://github.com/ros-perception/image_pipeline/blob/hydro-devel/depth_image_proc/src/nodelets/disparity.cpp#L136
    //for the record , I added an issue https://github.com/ros-perception/image_pipeline/issues/58 but it seems this is the way to do it..
     cal.P[3]=-1.0 * virtual_baseline * calib->intrinsic[CALIB_INTR_FX];
    // ----------------------------------------------------------------------

    cal.P[0]=calib->intrinsic[CALIB_INTR_FX];
    cal.P[5]=calib->intrinsic[CALIB_INTR_FY];
    cal.P[2]=calib->intrinsic[CALIB_INTR_CX];
    cal.P[6]=calib->intrinsic[CALIB_INTR_CY];
    cal.P[10]=1.0;

    pubRGBInfo.publish(cal);
    cal.width=depthWidth; cal.height=depthHeight;
    pubDepthInfo.publish(cal);
   }
   #endif
   //---------------------------------------------------------------------------------------------------
   //---------------------------------------------------------------------------------------------------
   //---------------------------------------------------------------------------------------------------


   //Spin ROS one time
   ros::spinOnce();

   //Deallocate constructed OpenCV images
   cvReleaseImageHeader( &imageRGB );
   cvReleaseImageHeader( &imageDepth );

   return true;
}


int publishImages()
{
   struct calibration rgbCalibration;
   getOpenNI2ColorCalibration(devID,&rgbCalibration);
   return publishImagesFrames(getOpenNI2ColorPixels(devID),getOpenNI2ColorWidth(devID),getOpenNI2ColorHeight(devID),
                               getOpenNI2DepthPixels(devID),getOpenNI2DepthWidth(devID),getOpenNI2DepthHeight(devID) , &rgbCalibration );
}


//----------------------------------------------------------


void loopEvent()
{
  //ROS_INFO("Loop Event started");
  //We spin from this thread to keep thread synchronization problems at bay
  ros::spinOnce();
  externalAcquisitionCallback(); //<- this also passes the new frames to skeleton / face detection
  publishImages();

   // if we got a depth and rgb frames , lets go
   key=getKeyPressed(); //Post our geometry to ROS
   //If we draw out we have to visualize the hand pose , set up windows , put out text etc.

    doDrawOut();  // <- end of we have depth and rgb code
}


int main(int argc, char **argv)
{
   ROS_INFO("Starting Up!!");

   int i=system("./downloadDependencies.sh");
   if (i!=0 ) { ROS_INFO("Could not check for missing dependencies of rgbd_acquisition"); }

   try
	{
	 ROS_INFO("Initializing ROS");
  	 ros::init(argc, argv, NODE_NAME);
     ros::start();

     ros::NodeHandle nh;
     nhPtr = &nh;

     int useSkeleton=0,devID=0;
     std::string from;
     std::string name;
     std::string camera;
     std::string frame;
     int rate;

     ros::NodeHandle private_node_handle_("~");
     private_node_handle_.param("useSkeleton", useSkeleton, int(0));
     if (useSkeleton) { pauseSkeletonDetection=0; pauseFaceDetection=0; dontPublishPointEvents=0;} else
                      { pauseSkeletonDetection=1; pauseFaceDetection=1; dontPublishPointEvents=0;}
     private_node_handle_.param("name", name, std::string("rgbd_acquisition"));
     private_node_handle_.param("camera", camera, std::string("camera"));
     private_node_handle_.param("frame", frame, std::string("frame"));
     private_node_handle_.param("rate", rate, int(30));
     private_node_handle_.param("deviceID", from, std::string(""));
     private_node_handle_.param("virtual_baseline", virtual_baseline, 0.0);

     //Pass root frame for TF poses
     strcpy(tfRoot,frame.c_str());


     //Decide on devID
/*
     if (from.length()==0) { } else
     if (from.length()<=2) { devID=atoi( from.c_str() ); from.clear(); ROS_INFO("Using OpenNI2 Serializer to get device"); }*/



     std::cout<<"RGBDAcquisition Starting settings ----------------"<<std::endl;
     std::cout<<"Name : "<<name<<std::endl;
     std::cout<<"Camera : "<<camera<<std::endl;
     std::cout<<"Frame : "<<frame<<std::endl;
     std::cout<<"Rate : "<<rate<<std::endl;
     std::cout<<"virtual_baseline : "<<virtual_baseline<<std::endl;
     std::cout<<"Device_id : "<<from<<" length "<<from.length()<<"  devID : "<<devID<<std::endl;
     std::cout<<"useSkeleton : "<<useSkeleton<<std::endl;
     if (!useSkeleton) { std::cout<<"(please note that Nite2 is disabled , to re-enable rosservice call /rgbd_acquisition/resume_peopletracker"<<std::endl; }
     std::cout<<"--------------------------------------------------"<<std::endl;


     ros::Rate loop_rate(rate); //  hz should be our target performance

     //We advertise the services we want accessible using "rosservice call *w/e*"
     ros::ServiceServer visualizeOnService      = nh.advertiseService(name+"/visualize_on", visualizeOn);
     ros::ServiceServer visualizeOffService     = nh.advertiseService(name+"/visualize_off", visualizeOff);
     ros::ServiceServer terminateService        = nh.advertiseService(name+"/terminate", terminate);
     ros::ServiceServer pauseService            = nh.advertiseService(name+"/pause_peopletracker", pause);
     ros::ServiceServer resumeService           = nh.advertiseService(name+"/resume_peopletracker", resume);
     ros::ServiceServer triggerPeopleTracker    = nh.advertiseService(name+"/trigger_peopletracker", trigger);
     ros::ServiceServer pausePointingService    = nh.advertiseService(name+"/pause_pointing_gesture_messages", pausePointingMessages);
     ros::ServiceServer resumePointingService   = nh.advertiseService(name+"/resume_pointing_gesture_messages", resumePointingMessages);
     ros::ServiceServer setQualityService       = nh.advertiseService(name+"/setQuality", setQuality);


     //Output RGB Image
     image_transport::ImageTransport it(nh);

     pubRGB = it.advertise(camera+"/rgb/image_rect_color", 1);
     pubRGBInfo = nh.advertise<sensor_msgs::CameraInfo>(camera+"/rgb/camera_info",1);
     std::cout<<"ROS RGB Topic name : "<<camera<<"/rgb/image_rect_color"<<std::endl;

     pubDepth = it.advertise(camera+"/depth_registered/image_rect", 1);
     pubDepthInfo = nh.advertise<sensor_msgs::CameraInfo>(camera+"/depth_registered/camera_info",1);
     std::cout<<"ROS Depth Topic name : "<<camera<<"/depth_registered/image_rect"<<std::endl;

      //---------------------------------------------------------------------------------------------------
      //This code segment waits for a valid first frame to come and initialize the focal lengths etc..
      //If there is no first frame it times out after a little while displaying a relevant error message
      //---------------------------------------------------------------------------------------------------
      if (! acquistionStartUp((char*) "OPENNI2",devID,(char*) from.c_str(),640,480,30) )
          {
            ROS_ERROR("Stopping RGBDAcquisition service , OpenNI2 USB Issues etc ? :( ..");
            return 1;
          }

      //Create our context
      //---------------------------------------------------------------------------------------------------
	  //////////////////////////////////////////////////////////////////////////
      registerServices(&nh);


	 ROS_INFO("Done Initializing RGBDAcqusition , now entering grab loop..");
	  while ( ( key!='q' ) && (ros::ok()) )
		{
                  loopEvent(); //<- this keeps our ros node messages handled up until synergies take control of the main thread
                  loop_rate.sleep();
		}

      switchDrawOutTo(0);
	}
	catch(std::exception &e) { ROS_ERROR("Exception: %s", e.what()); return 1; }
	catch(...)               { ROS_ERROR("Unknown Error"); return 1; }
	ROS_ERROR("Shutdown complete");
	return 0;
}
