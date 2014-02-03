#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/spinner.h>
 
#include <stdexcept> 
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
 
int key = 0;


ros::NodeHandle * nhPtr=0;

//----------------------------------------------------------
//Advertised Service switches
bool terminate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Stopping Emergency Detector");
    exit(0);
    return true;
}


bool pause(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    return true;
}

bool resume(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    return true;
}

 
int main(int argc, char **argv)
{
   ROS_INFO("Starting Up!!");

   try
	{
	 ROS_INFO("Initializing ROS");

	 char regName[128]={0};
	 sprintf(regName,"HandGestures%u",getpid());
	 fprintf(stderr,"Node named %s \n",regName);
  	 ros::init(argc, argv, regName);
     ros::start();

     ros::NodeHandle nh;
     nhPtr = &nh;

     //We advertise the services we want accessible using "rosservice call *w/e*"
     ros::ServiceServer pauseGestureRecognitionService    = nh.advertiseService("emergency_detector/pause", pause);
     ros::ServiceServer resumeGestureRecognitionService   = nh.advertiseService("emergency_detector/resume", resume);
     ros::ServiceServer stopGestureRecognitionService     = nh.advertiseService("emergency_detector/terminate", terminate);

      //Create our context
      //---------------------------------------------------------------------------------------------------
	  //////////////////////////////////////////////////////////////////////////
	  while ( ( key!='q' ) && (ros::ok()) )
		{  
                  ros::spinOnce();//<- this keeps our ros node messages handled up until synergies take control of the main thread
                  usleep(1000);
		 }

	}
	catch(std::exception &e) { ROS_ERROR("Exception: %s", e.what()); return 1; }
	catch(...)               { ROS_ERROR("Unknown Error"); return 1; }
	ROS_ERROR("Shutdown complete");
	return 0;
}
