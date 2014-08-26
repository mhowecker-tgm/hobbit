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

#include "timer.h"

#define NODE_NAME "person_aggregator"
#define MAX_CMD_STR 1024
#define MAX_NUM_STR 128

#define divisor 1000

#define DEFAULT_FRAME_RATE 30

float maximumDistanceForIntegration = 220;
unsigned int integrationTime = 100000;
#define maxSource 4

int rate=DEFAULT_FRAME_RATE;

int key = 0;
unsigned int frameTimestamp=0;
ros::NodeHandle * nhPtr=0;
unsigned int paused=0;
unsigned int raw=1;
unsigned int localityUsed=1;

ros::Publisher personBroadcaster;

struct position3D
{
  float x,y,z;
};

struct personMessageSt
{
    float actualX , actualY , actualZ , actualTheta , actualConfidence;
    unsigned int actualTimestamp , actualInFieldOfView;
    unsigned int source;
};

struct position3D lastKnownPosition[maxSource]={0};




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
}


void aggregatePersonMessage(struct personMessageSt * p )
{
  fprintf(stderr,"aggregatePersonMessage is not implemented yet\n");
  unsigned int i=0;
  unsigned int currentTime = EndTimer(p->source);

  unsigned int clues = 0;
  for (i=0; i<maxSource; i++)
  {
    if (i!=p->source)
      {
       fprintf(stderr,"Temporal Diff %u with %u is %u \n",p->source,i,EndTimer(i)-currentTime);
       if (EndTimer(i)-currentTime < integrationTime)
        {
          if (localityUsed)
          {
            double distance = sqrt( ((p->actualX-lastKnownPosition[i].x)*(p->actualX-lastKnownPosition[i].x)) +
                                     ((p->actualY-lastKnownPosition[i].y)*(p->actualY-lastKnownPosition[i].y)) +
                                     ((p->actualZ-lastKnownPosition[i].z)*(p->actualZ-lastKnownPosition[i].z))   );

            fprintf(stderr,"Spatial Diff %u with %u is %0.2f \n",p->source,i,distance);
            if (distance<maximumDistanceForIntegration)
            {
              ++clues;
            }
          } else
          {
            //If we don't use locality having two messages at the same period is good enough
            ++clues;
          }
        }
      }
  }

   fprintf(stderr,"Gathered %u clues for a person \n",clues);
   if (clues>=1)
   {
     broadcastNewPerson(p);
   }
}


void personMessageAggregator(const person_aggregator::Person & msg , unsigned int source)
{
    fprintf(stderr,"personMessageAggregator triggered %u , after %u time \n",source , EndTimer(0));

    struct personMessageSt prsn;

    prsn.actualX = msg.x;
    prsn.actualY = msg.y;
    prsn.actualZ = msg.z;

    prsn.source = msg.source;
    prsn.actualTheta = msg.theta;

    prsn.actualInFieldOfView = msg.inFieldOfView;
    prsn.actualInFieldOfView = msg.confidence;
    prsn.actualInFieldOfView = msg.timestamp;

    if (prsn.source<TOTAL_TIMERS)
    {
        StartTimer(prsn.source);
    }

    if (prsn.source<maxSource)
    {
      lastKnownPosition[prsn.source].x = msg.x;
      lastKnownPosition[prsn.source].y = msg.y;
      lastKnownPosition[prsn.source].z = msg.z;
    }

    if (raw) {
               fprintf(stderr,"Raw Mode : Blindly passing around received Person Message\n");
               broadcastNewPerson(&prsn);
              } else
             {
               fprintf(stderr,"Precise Mode : Collecting Person Message \n");
               aggregatePersonMessage(&prsn);
             }
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


bool switchToPrecise(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    raw=0;
    return true;
}

bool switchToRaw(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    raw=1;
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
     ros::ServiceServer preciseService  = nh.advertiseService(name+"/precise", switchToPrecise);
     ros::ServiceServer rawService      = nh.advertiseService(name+"/raw", switchToRaw);


     personBroadcaster = nh.advertise <person_aggregator::Person> ("persons", divisor);

     ros::Subscriber sub1 = nh.subscribe("/rgbd_acquisition/persons",divisor,personMessageRGBDAcquisition);
     ros::Subscriber sub2 = nh.subscribe("/skeleton_detector/persons",divisor,personMessageSkeletonDetector);
     ros::Subscriber sub3 = nh.subscribe("/face_detection/persons",divisor,personMessageFaceDetector);
     ros::Subscriber sub4 = nh.subscribe("/hand_gestures/persons",divisor,personMessageGestures);

     //Create our context
     //---------------------------------------------------------------------------------------------------
	 //////////////////////////////////////////////////////////////////////////
	 unsigned int i=0;
     for (i=0; i<maxSource; i++) { StartTimer(i); }



	  while ( ( key!='q' ) && (ros::ok()) )
		{

                  ros::spinOnce();//<- this keeps our ros node messages handled up until synergies take control of the main thread
	    }

	}
	catch(std::exception &e) { ROS_ERROR("Exception: %s", e.what()); return 1; }
	catch(...)               { ROS_ERROR("Unknown Error"); return 1; }
	ROS_ERROR("Shutdown complete");
	return 0;
}
