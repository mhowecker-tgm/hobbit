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
 #include <opencv2/imgproc/imgproc_c.h>
 #include <opencv2/legacy/legacy.hpp>
 #include "opencv2/highgui/highgui.hpp"

#include <std_srvs/Empty.h>


#include "person_aggregator/Person.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#define NODE_NAME "person_aggregator"
#define MAX_CMD_STR 1024
#define MAX_NUM_STR 128

#define divisor 1000

#define DEFAULT_FRAME_RATE 30

int rate=DEFAULT_FRAME_RATE;

int key = 0;
unsigned int frameTimestamp=0;
ros::NodeHandle * nhPtr=0;
unsigned int paused=0;

ros::Publisher personBroadcaster;

struct personMessageSt
{
    float actualX , actualY , actualZ , actualTheta , actualConfidence;
    unsigned int actualTimestamp , actualInFieldOfView;
    unsigned int source;
};


void broadcastNewPerson( struct personMessageSt * p)
{
  person_aggregator::Person msg;
  msg.x = p->actualX;
  msg.y = p->actualY;
  msg.z = p->actualZ;
  msg.source = p->source;
  msg.theta = p->actualTheta;

  msg.inFieldOfView = p->actualInFieldOfView;
  msg.confidence = p->actualConfidence;
  msg.timestamp=p->actualTimestamp;

  fprintf(stderr,"Publishing a new Person\n");
  personBroadcaster.publish(msg);
  //ros::spinOnce();
}



void personMessageAggregator(const person_aggregator::Person & msg , unsigned int source)
{
    struct personMessageSt prsn;

    prsn.actualX = msg.x;
    prsn.actualY = msg.y;
    prsn.actualZ = msg.z;

    prsn.source = msg.source;
    prsn.actualTheta = msg.theta;

    prsn.actualInFieldOfView = msg.inFieldOfView;
    prsn.actualInFieldOfView = msg.confidence;
    prsn.actualInFieldOfView = msg.timestamp;
}

void personMessageRGBDAcquisition(const person_aggregator::Person & msg)
{
  personMessageAggregator(msg,1);
}

void personMessageSkeletonDetector(const person_aggregator::Person & msg)
{
  personMessageAggregator(msg,2);
}

void personMessageFaceDetector(const person_aggregator::Person & msg)
{
  personMessageAggregator(msg,3);
}

void personMessageGestures(const person_aggregator::Person & msg)
{
  personMessageAggregator(msg,4);
}



//----------------------------------------------------------
//Advertised Service switches
bool terminate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Stopping Node " NODE_NAME);
    exit(0);
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



int main(int argc, char **argv)
{
   ROS_INFO("Starting Up!!");

   try
	{
	 ROS_INFO("Initializing ROS");

  	 ros::init(argc, argv, NODE_NAME);
     ros::start();

     ros::NodeHandle nh;
     nhPtr = &nh;

     ros::NodeHandle private_node_handle_("~");

     std::string name;

     private_node_handle_.param("name", name, std::string(NODE_NAME));
     private_node_handle_.param("rate", rate , int(DEFAULT_FRAME_RATE));
     ros::Rate loop_rate(rate); //  hz should be our target performance


     //We advertise the services we want accessible using "rosservice call *w/e*"
     ros::ServiceServer pauseService    = nh.advertiseService(name+"/pause", pause);
     ros::ServiceServer resumeService   = nh.advertiseService(name+"/resume", resume);
     ros::ServiceServer stopService     = nh.advertiseService(name+"/terminate", terminate);


     personBroadcaster = nh.advertise <person_aggregator::Person> ("persons", divisor);

     ros::Subscriber sub1 = nh.subscribe("/rgbd_acquisition/persons",divisor,personMessageRGBDAcquisition);
     ros::Subscriber sub2 = nh.subscribe("/skeleton_detector/persons",divisor,personMessageSkeletonDetector);
     ros::Subscriber sub3 = nh.subscribe("/face_detection/persons",divisor,personMessageFaceDetector);
     ros::Subscriber sub4 = nh.subscribe("/hand_gestures/persons",divisor,personMessageGestures);

     //Create our context
     //---------------------------------------------------------------------------------------------------
	 //////////////////////////////////////////////////////////////////////////

	  while ( ( key!='q' ) && (ros::ok()) )
		{
		          fprintf(stderr,".");
                  ros::spinOnce();//<- this keeps our ros node messages handled up until synergies take control of the main thread
	    }

	}
	catch(std::exception &e) { ROS_ERROR("Exception: %s", e.what()); return 1; }
	catch(...)               { ROS_ERROR("Unknown Error"); return 1; }
	ROS_ERROR("Shutdown complete");
	return 0;
}
