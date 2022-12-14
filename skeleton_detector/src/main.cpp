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
#include "skeleton_detector/setPitch.h"


#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include "services.h"


#include "hobbitUpperBodyTrackerLib.h"


//This will make this node also register to color/depth calibrations and
//pass them to the gesture node instead of the defaults
#define USE_NONDEFAULT_CALIBRATIONS 1
#define USE_COMPRESSED_STREAMS 0
#define MAX_RECORDED_FRAMES 1000


#define ABSDIFF(num1,num2) ( (num1-num2) >=0 ? (num1-num2) : (num2 - num1) )

int rate=11;
int first=0;
int key = 0;

int recording=0;
int recordedFrames=0;



unsigned int framesForHeightenedAttention=30;
unsigned int runFullSpeed=0;
unsigned int runMaxSpeed=0;
unsigned int colorWidth = 640 , colorHeight =480 , depthWidth = 640 , depthHeight = 480;

struct calibrationHobbit calib={0};

#if USE_NONDEFAULT_CALIBRATIONS
 typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> RgbdSyncPolicy;
#else
 typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RgbdSyncPolicy;
#endif


//----------------------------------------------------------



bool setPitch( skeleton_detector::setPitch::Request  &request,
                  skeleton_detector::setPitch::Response &response )
{
   hobbitUpperBodyTracker_setCameraPitch(request.num);
   response.ok=1;

   return true ;
}


bool visualizeOn(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    hobbitUpperBodyTracker_setVisualization(1);
    runFullSpeed=1;
    return true;
}

bool visualizeOff(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    hobbitUpperBodyTracker_setVisualization(0);
    runFullSpeed=0;
    return true;
}

bool terminate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Stopping Skeleton Detector");
    exit(0);
    return true;
}


bool startDump(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Enabling Dump to files");
    recording=1;
    recordedFrames=0;
    hobbitUpperBodyTracker_setDumpToFiles(1);

    return true;
}

int stopDumpInternal()
{
    ROS_INFO("Disabling Dump to files");
    hobbitUpperBodyTracker_setDumpToFiles(0);
    recording=0;
    recordedFrames=0;

    ROS_INFO("Packaging Dataset , this might take a while..");
    int i=system("./packageRecord.sh");
    if (i==0) { ROS_INFO("Success packaging..!\n"); } else
              { ROS_INFO("Error packaging..!\n"); }

  return (i==0);
}


bool clearDump(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    int i=system("./clearRecords.sh");
    if (i==0) { fprintf(stderr,"Success packaging..!\n"); } else
              { fprintf(stderr,"Error packaging..!\n");   }
    return true;
}

bool benchmark(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector : Benchmark Mode on , will consume a LOT of CPU");
    runMaxSpeed=1;
    return true;
}


bool attention(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector : Heightened Attention..!");
    triggerAttentionInternal();
    return true;
}

bool stopDump(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    stopDumpInternal();
    return true;
}



bool simple(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector : Simple Mode on");
    processingMode=PROCESSING_MODE_SIMPLE_PERSON_DETECTOR;
    triggerAttentionInternal(); // On switches between modes , give more CPU time [ switches happen for a reason typically ]
    return true;
}

bool advanced(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector : Advanced Mode , we go to Super Mode instead ");
    //processingMode=PROCESSING_MODE_TREE_GRID_BODY_TRACKER;
    processingMode=PROCESSING_MODE_UPPER_GESTURE_BODY_TRACKER;
    triggerAttentionInternal(); // On switches between modes , give more CPU time [ switches happen for a reason typically ]
    return true;
}

bool super(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector : Super New Mode on");
    processingMode=PROCESSING_MODE_UPPER_GESTURE_BODY_TRACKER;
    triggerAttentionInternal(); // On switches between modes , give more CPU time [ switches happen for a reason typically ]
    return true;
}


bool pause(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector is now paused");
    paused =1;
    return true;
}

bool resume(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector is now resuming");
    paused =0;
    triggerAttentionInternal(); // On switches between modes , give more CPU time [ switches happen for a reason typically ]
    return true;
}


bool pauseBodylessGestures(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector is now filtering bodyless gestures");
    hobbitUpperBodyTracker_useGesturesWithoutABody(0);
    return true;
}

bool resumeBodylessGestures(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector is now publishing bodyless gestures");
    hobbitUpperBodyTracker_useGesturesWithoutABody(1);
    return true;
}


bool startDualElbowBendSyncExercise(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector is now starting startDualElbowBendSyncExercise");
    logStartExerciseTime();
    hobbitFitnessFunction_StartExercise(frameTimestamp,DUAL_ELBOW_BEND_SYNC_EXERCISE,0/*isLeftHand*/,5);
    return true;
}

bool startDualElbowBendAsyncExercise(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector is now starting Exercise Batch");
    logStartExerciseTime();
    hobbitFitnessFunction_StartExercise(frameTimestamp,DUAL_ELBOW_BEND_ASYNC_EXERCISE,0/*isLeftHand*/,5);
    return true;
}


bool startLeftFlapExercise(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector is now starting Exercise Batch");
    logStartExerciseTime();
    hobbitFitnessFunction_StartExercise(frameTimestamp,LEFT_FLAP_EXERCISE,1/*isLeftHand*/,5);
    return true;
}

bool startRightFlapExercise(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector is now starting Exercise Batch");
    logStartExerciseTime();
    hobbitFitnessFunction_StartExercise(frameTimestamp,RIGHT_FLAP_EXERCISE,0/*isLeftHand*/,5);
    return true;
}


bool startLeftArmPumpExercise(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector is now starting Exercise Batch");
    logStartExerciseTime();
    hobbitFitnessFunction_StartExercise(frameTimestamp,LEFT_ARM_PUMP_EXERCISE,1/*isLeftHand*/,5);
    return true;
}

bool startRightArmPumpExercise(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector is now starting Exercise Batch");
    logStartExerciseTime();
    hobbitFitnessFunction_StartExercise(frameTimestamp,RIGHT_ARM_PUMP_EXERCISE,1/*isLeftHand*/,5);
    return true;
}




bool stopExercise(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Skeleton Detector is now stopping Exercise Batch");
    hobbitFitnessFunction_StopExercise(frameTimestamp);
    return true;
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
 orig_depth_img = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::TYPE_16UC1);

          if (recording) { ++recordedFrames; }
          if (recordedFrames>MAX_RECORDED_FRAMES)
          {
            fprintf(stderr,"Automatic Cut Off of recording activated..");
            stopDumpInternal();
          }


  runServicesThatNeedColorAndDepth((unsigned char*) orig_rgb_img->image.data, colorWidth , colorHeight ,
                                   (unsigned short*) orig_depth_img->image.data ,  depthWidth , depthHeight ,
                                     &calib , frameTimestamp );
 ++frameTimestamp;
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
 orig_rgb_img = cv_bridge::toCvShare(rgb_img_msg, "rgb8");
 orig_depth_img = cv_bridge::toCvShare(depth_img_msg, sensor_msgs::image_encodings::TYPE_16UC1);


          if (recording) { ++recordedFrames; }
          if (recordedFrames>MAX_RECORDED_FRAMES)
          {
            fprintf(stderr,"Automatic Cut Off of recording activated..");
            stopDumpInternal();
          }

  runServicesThatNeedColorAndDepth((unsigned char*) orig_rgb_img->image.data, colorWidth , colorHeight ,
                                   (unsigned short*) orig_depth_img->image.data ,  depthWidth , depthHeight ,
                                     0 , frameTimestamp );

 ++frameTimestamp;
 //After running (at least) once it is not a first run any more!
 first = false;
return;
}
#endif

int main(int argc, char **argv)
{
   ROS_INFO("Starting Up !!");
   try
    {
     ros::init(argc, argv, "skeleton_detector");
     ros::start();

     ros::NodeHandle nh;
     ros::NodeHandle private_node_handle_("~");

     std::string name;
     std::string fromDepthTopic;
     std::string fromDepthTopicInfo;
     std::string fromRGBTopic;
     std::string fromRGBTopicInfo;


     private_node_handle_.param("fromDepthTopic", fromDepthTopic, std::string("/headcam/depth_registered/image_rect"));
     private_node_handle_.param("fromDepthTopicInfo", fromDepthTopicInfo, std::string("/headcam/depth_registered/camera_info"));
     private_node_handle_.param("fromRGBTopic", fromRGBTopic, std::string("/headcam/rgb/image_rect_color"));
     private_node_handle_.param("fromRGBTopicInfo", fromRGBTopicInfo, std::string("/headcam/rgb/camera_info"));
     private_node_handle_.param("name", name, std::string("skeleton_detector"));
     private_node_handle_.param("rate", rate, int(5));
     ros::Rate loop_rate_ultra_low(2); //  hz should be our target performance
     ros::Rate loop_rate(rate); //  hz should be our target performance
     unsigned int fastRate = rate*3;
     if (fastRate > 30) { fastRate=30; } //Cap fast speed at 30Hz ( i.e. frame rate of Depth camera )
     ros::Rate loop_rate_fast(fastRate); //  hz should be our target performance
     ros::Rate loop_rate_fullSpeed(30); //  hz should be our target performance

     //We advertise the services we want accessible using "rosservice call *w/e*"
     ros::ServiceServer visualizeOnService      = nh.advertiseService(name+"/visualize_on" , visualizeOn);
     ros::ServiceServer visualizeOffService     = nh.advertiseService(name+"/visualize_off", visualizeOff);
     ros::ServiceServer benchmarkService        = nh.advertiseService(name+"/benchmark" , benchmark);
     ros::ServiceServer attentionService        = nh.advertiseService(name+"/attention" , attention);
     ros::ServiceServer terminateService        = nh.advertiseService(name+"/terminate"    , terminate);
     ros::ServiceServer resumeService           = nh.advertiseService(name+"/pause"        , pause);
     ros::ServiceServer dumpService             = nh.advertiseService(name+"/startDump"    , startDump);
     ros::ServiceServer stopDumpService         = nh.advertiseService(name+"/stopDump"     , stopDump);
     ros::ServiceServer clearDumpService        = nh.advertiseService(name+"/clearDump"    , clearDump);
     ros::ServiceServer pauseService            = nh.advertiseService(name+"/resume"       , resume);
     ros::ServiceServer simpleService           = nh.advertiseService(name+"/simple"       , simple);
     ros::ServiceServer advancedService         = nh.advertiseService(name+"/advanced"     , advanced);
     ros::ServiceServer superService            = nh.advertiseService(name+"/super"        , super);

     ros::ServiceServer nearService         = nh.advertiseService(name+"/near"         , simple);
     ros::ServiceServer farService            = nh.advertiseService(name+"/far"          , super);


     ros::ServiceServer pauseBodylessGesturesService = nh.advertiseService(name+"/pauseBodylessGestures" , pauseBodylessGestures);
     ros::ServiceServer resumeBodylessGesturesService= nh.advertiseService(name+"/resumeBodylessGestures", resumeBodylessGestures);

     ros::ServiceServer  startExercise0LService    = nh.advertiseService(name+"/startLeftArmPumpExercise"        , startLeftArmPumpExercise);
     ros::ServiceServer  startExercise0RService    = nh.advertiseService(name+"/startRightArmPumpExercise"       , startRightArmPumpExercise);
     ros::ServiceServer  startExercise1LService    = nh.advertiseService(name+"/startLeftFlapExercise"        , startLeftFlapExercise);
     ros::ServiceServer  startExercise1RService    = nh.advertiseService(name+"/startRightFlapExercise"       , startRightFlapExercise);

     ros::ServiceServer  startExerciseDuSyncService    = nh.advertiseService(name+"/startDualElbowBendSyncExercise"       , startDualElbowBendSyncExercise);
     ros::ServiceServer  startExerciseDuAsyncService   = nh.advertiseService(name+"/startDualElbowBendAsyncExercise"       , startDualElbowBendAsyncExercise);
     ros::ServiceServer  stopExerciseService       = nh.advertiseService(name+"/stopExercise"         , stopExercise);


     ros::ServiceServer setPitchService         = nh.advertiseService(name+"/setPitch", setPitch);
     //Make our rostopic cmaera grabber
     message_filters::Synchronizer<RgbdSyncPolicy> *sync;

     std::cerr<<"\n\n\nskeleton detector , RGB feed "<<fromRGBTopic<<" \n";
     std::cerr<<"skeleton detector , RGB Info "<<fromRGBTopicInfo<<" \n";
     std::cerr<<"skeleton detector , Depth feed "<<fromDepthTopic<<" \n";
     std::cerr<<"skeleton detector , Depth Info "<<fromDepthTopicInfo<<" \n";



    image_transport::ImageTransport it(nh);

     #if USE_COMPRESSED_STREAMS
      std::string depth_topic = std::string(fromDepthTopic);
      image_transport::TransportHints hintsDepth("compressedDepth");
 	  image_transport::SubscriberFilter *depth_img_sub = new image_transport::SubscriberFilter();
 	  depth_img_sub->subscribe(it,depth_topic,(uint32_t) 1,hintsDepth);
     #else
	  message_filters::Subscriber<sensor_msgs::Image> *depth_img_sub  = new message_filters::Subscriber<sensor_msgs::Image>(nh,fromDepthTopic,1);
     #endif // USE_COMPRESSED_STREAMS


     #if USE_COMPRESSED_STREAMS
      std::string color_topic = std::string(fromRGBTopic);
      image_transport::TransportHints hints("compressed");
      image_transport::SubscriberFilter *rgb_img_sub = new  image_transport::SubscriberFilter();
      rgb_img_sub->subscribe(it,color_topic, (uint32_t) 1 , hints);
     #else
      message_filters::Subscriber<sensor_msgs::Image> *rgb_img_sub = new  message_filters::Subscriber<sensor_msgs::Image>(nh,fromRGBTopic, 1);
     #endif // USE_COMPRESSED_STREAMS

     std::cerr<<"Done\n";


     #if USE_NONDEFAULT_CALIBRATIONS
       std::cerr<<"Also subscribing to the camera info topics\n";
       message_filters::Subscriber<sensor_msgs::CameraInfo> *depth_cam_info_sub;
       message_filters::Subscriber<sensor_msgs::CameraInfo> *rgb_cam_info_sub;

	   depth_cam_info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,fromDepthTopicInfo,1);
	   rgb_cam_info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,fromRGBTopicInfo,1);

 	   sync = new message_filters::Synchronizer<RgbdSyncPolicy>(RgbdSyncPolicy(rate), *rgb_img_sub, *depth_img_sub,*depth_cam_info_sub); //*rgb_cam_info_sub,
 	   sync->registerCallback(rgbdCallback);
     #else
       std::cerr<<"Ignoring camera info topics\n";
       sync = new message_filters::Synchronizer<RgbdSyncPolicy>(RgbdSyncPolicy(rate), *rgb_img_sub, *depth_img_sub); //*rgb_cam_info_sub,
	   sync->registerCallback(rgbdCallbackNoCalibration);
    #endif

     registerServices(&nh,640,480);

	  while ( ( key!='q' ) && (ros::ok()) )
		{
          ros::spinOnce();

            unsigned int lastDetectedFrame = ABSDIFF(frameTimestamp,actualTimestamp);

           if (runMaxSpeed)
               {
                 //No Sleep at all
               } else
            if ( (hobbitFitnessFunction_AreWeDoingAnExercise()) || (runFullSpeed) )
                { //if we want exercise tracking or running at full speed we try to run as fast as possible..
                  loop_rate_fullSpeed.sleep();
                } else
            if (processingMode==PROCESSING_MODE_UPPER_GESTURE_BODY_TRACKER)
            {
              if ( lastDetectedFrame < framesForHeightenedAttention )
                         {  loop_rate_fast.sleep();       /*Face Detector Not using a lot of CPU , so let's go fast*/ } else
                         {  loop_rate_ultra_low.sleep();  /*Face Detector using a lot of CPU , so let's go slow */    }
            } else
            {  loop_rate.sleep(); }



            if ( hobbitFitnessFunction_AreWeDoingAnExercise() )
              {
                  if (hasExerciseTimedOut())
                  {
                    ROS_INFO("Skeleton Detector is now auto-stopping Exercise");
                    hobbitFitnessFunction_StopExercise(frameTimestamp);
                  }
              }

		 }

	   stopServices();

	   delete depth_img_sub;
	   delete rgb_img_sub;


     #if USE_NONDEFAULT_CALIBRATIONS
       delete depth_cam_info_sub;
	   delete rgb_cam_info_sub;
     #endif // USE_NONDEFAULT_CALIBRATIONS

	   delete sync;
	}
	catch(std::exception &e) { ROS_ERROR("Exception: %s", e.what()); return 1; }
	catch(...)               { ROS_ERROR("Unknown Error"); return 1; }
	ROS_ERROR("Shutdown complete");
	return 0;
}
