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

#include "hobbit_msgs/Event.h"
#include "hobbit_msgs/SwitchVision.h"
//#include "person_aggregator/BoolSwitch.h"
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
#define MAX_CMD 2048

#define DEFAULT_FRAME_RATE 15

float maximumDistanceForIntegration = 230;
unsigned int integrationTimeMicroseconds = 5 * 1000* 1000;
#define maxSource 7

int rate=DEFAULT_FRAME_RATE;

int key = 0;
unsigned int frameTimestamp=0;
ros::NodeHandle * nhPtr=0;
unsigned int paused=0;
unsigned int raw=1;
unsigned int localityUsed=1;

unsigned int lastTriggerTime = 0;

ros::Publisher personBroadcaster;



#define NORMAL "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */


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

  fprintf(stderr,GREEN "Publishing a new Person\n" NORMAL);
  personBroadcaster.publish(msg);
}


void aggregatePersonMessage(struct personMessageSt * p )
{
  unsigned int i=0;
  unsigned int currentTime = EndTimer(p->source);

  unsigned int clues = 0;
  for (i=0; i<maxSource; i++)
  {
    if (i!=p->source)
      {

       unsigned int messageTimeOffsetMicroSeconds =  EndTimer(i)-currentTime;
       float messageTimeOffsetSecs = (float) messageTimeOffsetMicroSeconds/1000000;
       if (messageTimeOffsetMicroSeconds < integrationTimeMicroseconds)
        {
          fprintf(stderr,GREEN "Temporal Diff %u with %u is %0.2f sec\n" NORMAL,p->source,i,messageTimeOffsetSecs);
          if (localityUsed)
          {
            double distance = sqrt( ((p->actualX-lastKnownPosition[i].x)*(p->actualX-lastKnownPosition[i].x)) +
                                     ((p->actualY-lastKnownPosition[i].y)*(p->actualY-lastKnownPosition[i].y)) +
                                     ((p->actualZ-lastKnownPosition[i].z)*(p->actualZ-lastKnownPosition[i].z))   );

            if (distance<=maximumDistanceForIntegration)
            {
              fprintf(stderr,GREEN "Spatial Diff %u with %u is %0.2f mm \n" NORMAL,p->source,i,distance);
              ++clues;
            } else
            {
              fprintf(stderr,RED "Spatial Diff %u with %u is %0.2f mm \n" NORMAL,p->source,i,distance);
            }
          } else
          {
            fprintf(stderr,RED "Distance is too far sources %u,%u are %0.2f mm away\n" NORMAL,p->source,i,messageTimeOffsetSecs);
            //If we don't use locality having two messages at the same period is good enough
            ++clues;
          }
        }
      }
  }

   if (clues>=1)
   {
     fprintf(stderr,GREEN "Gathered enough clues ( %u ) to broadcast a person \n" NORMAL ,clues);
     broadcastNewPerson(p);
   } else
   {
     fprintf(stderr,RED "Gathered only %u clues , not sure if this is really a person\n" NORMAL ,clues);
   }


     fprintf(stderr,YELLOW "----------------------------------------------\n\n\n\n\n\n" NORMAL);

}


void personMessageAggregator(const person_aggregator::Person & msg , unsigned int source)
{
    unsigned int thisTriggerTime = EndTimer(0);
    float messageTriggerTimeDifference = (float) (thisTriggerTime-lastTriggerTime)/1000000;
    fprintf(stderr,"\n\n\n\n\n\n\n\n\n\n");
    fprintf(stderr,"personMessageAggregator triggered by %u after %0.2f secs of inactivity\n",source , messageTriggerTimeDifference );
    lastTriggerTime = thisTriggerTime;

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

void personMessageEmergency(const person_aggregator::Person & msg)
{
   ROS_INFO("Emergency Node is no longer allowed to trigger persons ");
   //personMessageAggregator(msg,5);
}

void personMessageFollowUser(const person_aggregator::Person & msg)
{
  personMessageAggregator(msg,6);
}



//----------------------------------------------------------
//Advertised Service switches
bool terminate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Stopping Node " NODE_NAME);
    exit(0);
    return true;
}

bool trigger(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Triggering Person Message" NODE_NAME);
    struct personMessageSt p={0};
    p.actualConfidence=0.5;
    p.actualInFieldOfView=1;
    p.actualTimestamp=frameTimestamp;
    p.actualX=0;
    p.actualY=0;
    p.actualZ=1000;
    p.actualTheta=0;
    p.source=0;
    //--------------
    broadcastNewPerson(&p);

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



int rosservice_call(const char * serviceName)
{
  char commandToRun[MAX_CMD]={0};
  unsigned int maxCommandSize=MAX_CMD;
  snprintf(commandToRun,maxCommandSize,"/bin/bash -c \"rosservice call %s\"",serviceName);
  int i=system(commandToRun);

  if (i!=0)
    {
      char msg[512]={0};
      snprintf(msg,512,"Could not rosservice call %s \n",serviceName);
      ROS_ERROR(msg);
    }

  return (i==0);
}


int rosparam_set(const char * paramName,const char * paramValue)
{
  char commandToRun[MAX_CMD]={0};
  unsigned int maxCommandSize=MAX_CMD;
  snprintf(commandToRun,maxCommandSize,"/bin/bash -c \"rosparam set %s \"%s\" \"",paramName,paramValue);
  int i=system(commandToRun);

  if (i!=0)
    {
      char msg[512]={0};
      snprintf(msg,512,"Could not rosparam set %s \"%s\" \n",paramName,paramValue);
      ROS_ERROR(msg);
    }

  return (i==0);
}


int rostopic_pub(const char * topicName,const char * topicType,const char * topicValue)
{
  char commandToRun[MAX_CMD]={0};
  unsigned int maxCommandSize=MAX_CMD;
  snprintf(commandToRun,maxCommandSize,"/bin/bash -c \"rostopic pub %s %s %s -1\"",topicName,topicType,topicValue);
  int i=system(commandToRun);

  if (i!=0)
    {
      char msg[512]={0};
      snprintf(msg,512,"Could not rostopic pub %s %s %s \n",topicName,topicType,topicValue);
      ROS_ERROR(msg);
    }

  return (i==0);
}

bool pauseEverything(hobbit_msgs::SwitchVision::Request & request ,  hobbit_msgs::SwitchVision::Response & response  )
{
    ROS_INFO("Pausing All User Sensing");
    int i=0,target=0;
    ++target; i+=rosservice_call("/rgbd_acquisition/pause_peopletracker");
    ++target; i+=rosservice_call("/follow_user/pause");
    ++target; i+=rosservice_call("/emergency_detector/pause");
    //++target; i+=rosservice_call("/face_detection/pause");
    //++target; i+=rosservice_call("/hand_gestures/pause");
    ++target; i+=rosservice_call("/skeleton_detector/pause");
    //++target; i+=rosservice_call("/fitness_coordinator/pause");
    if (target==i) { response.result=true; } else { response.result=false; }
    if (!response.result) { ROS_ERROR("Could not successfully set all relevant nodes to the new mode"); }
    return true;
}


bool resumeBasic(hobbit_msgs::SwitchVision::Request & request ,  hobbit_msgs::SwitchVision::Response & response  )
{
    ROS_INFO("Resuming Basic User Sensing");
     int i=0,target=0;
    //++target; i+=rosservice_call("/follow_user/resume");
    ++target; i+=rosservice_call("/emergency_detector/resume");
    ++target; i+=rosservice_call("/skeleton_detector/resume");

    if (target==i) { response.result=true; } else { response.result=false; }
    if (!response.result) { ROS_ERROR("Could not successfully set all relevant nodes to the new mode"); }
    raw=1;
    return true;
}


bool resumeEverything(hobbit_msgs::SwitchVision::Request & request ,  hobbit_msgs::SwitchVision::Response & response  )
{
    ROS_INFO("Resuming All User Sensing");
     int i=0,target=0;
    //Never Resume People tracker : ++target; i+=rosservice_call("/rgbd_acquisition/resume_peopletracker");
    ++target; i+=rosservice_call("/follow_user/resume");
    ++target; i+=rosservice_call("/emergency_detector/resume");
    ++target; i+=rosservice_call("/skeleton_detector/resume");
    if (target==i) { response.result=true; } else { response.result=false; }
    if (!response.result) { ROS_ERROR("Could not successfully set all relevant nodes to the new mode"); }
    raw=1;
    return true;
}

bool followUser(hobbit_msgs::SwitchVision::Request & request ,  hobbit_msgs::SwitchVision::Response & response  )
{
    ROS_INFO("Setting Vision System to Follow a User");
     int i=0,target=0;
    //Never Resume People tracker : ++target; i+=rosservice_call("/rgbd_acquisition/resume_peopletracker"); //Nite tracker might be useful
    ++target; i+=rosservice_call("/follow_user/resume");
    ++target; i+=rosservice_call("/emergency_detector/resume");
    ++target; i+=rosservice_call("/skeleton_detector/resume");
    ++target; i+=rosservice_call("/skeleton_detector/simple"); //We want the simple skeleton detector , no hands but fast and more robust ( even without a face )
    if (target==i) { response.result=true; } else { response.result=false; }
    if (!response.result) { ROS_ERROR("Could not successfully set all relevant nodes to the new mode"); }
    return true;
}

bool comeCloser(hobbit_msgs::SwitchVision::Request & request ,  hobbit_msgs::SwitchVision::Response & response)
{
    ROS_INFO("Setting Vision System to Come Closer Mode");
    int i=0,target=0;
    ++target; i+=rosservice_call("/follow_user/pause");
    ++target; i+=rosservice_call("/emergency_detector/pause"); // So that we get
    ++target; i+=rosservice_call("/skeleton_detector/resume");
    ++target; i+=rosservice_call("/skeleton_detector/advanced"); //
    if (target==i) { response.result=true; } else { response.result=false; }
    if (!response.result) { ROS_ERROR("Could not successfully set all relevant nodes to the new mode"); }
    return true;
}

bool locateUser(hobbit_msgs::SwitchVision::Request & request ,  hobbit_msgs::SwitchVision::Response & response  )
{
    ROS_INFO("Setting Vision System to Locate a User");
     int i=0,target=0;
    //Never Resume People tracker : ++target; i+=rosservice_call("/rgbd_acquisition/resume_peopletracker"); //Nite tracker might be useful
    ++target; i+=rosservice_call("/follow_user/resume");
    ++target; i+=rosservice_call("/emergency_detector/resume"); // So that we get
    ++target; i+=rosservice_call("/face_detection/resume");
    //++target; i+=rosservice_call("/hand_gestures/resume");
    ++target; i+=rosservice_call("/skeleton_detector/resume");
    ++target; i+=rosservice_call("/skeleton_detector/simple"); //We want the simple skeleton detector , no hands but fast and more robust ( even without a face )
    if (target==i) { response.result=true; } else { response.result=false; }
    if (!response.result) { ROS_ERROR("Could not successfully set all relevant nodes to the new mode"); }
    raw=1; //We want to be precise..
    return true;
}

bool fitnessFunction(hobbit_msgs::SwitchVision::Request & request ,  hobbit_msgs::SwitchVision::Response & response  )
{
    ROS_INFO("Setting Vision System to do FitnessFunction");
     int i=0,target=0;
    ++target; i+=rosservice_call("/rgbd_acquisition/pause_peopletracker"); //Nite tracker is no longer used
    ++target; i+=rosservice_call("/follow_user/pause");
    ++target; i+=rosservice_call("/skeleton_detector/resume");
    ++target; i+=rosservice_call("/skeleton_detector/advanced"); //We want the advanced skeleton detector
    ++target; i+=rosservice_call("/fitness_coordinator/resume");
    if (target==i) { response.result=true; } else { response.result=false; }
    if (!response.result) { ROS_ERROR("Could not successfully set all relevant nodes to the new mode"); }
    return true;
}


bool whereIsUserPointing(hobbit_msgs::SwitchVision::Request & request ,  hobbit_msgs::SwitchVision::Response & response  )
{
    ROS_INFO("Setting Vision System to see where the User is Pointing");
     int i=0,target=0;
    ++target; i+=rosservice_call("/rgbd_acquisition/pause_peopletracker"); //Nite tracker might be useful
    ++target; i+=rosservice_call("/follow_user/pause");
    ++target; i+=rosservice_call("/skeleton_detector/resume");
    ++target; i+=rosservice_call("/skeleton_detector/advanced"); //We want the advanced skeleton detector
    if (target==i) { response.result=true; } else { response.result=false; }
    if (!response.result) { ROS_ERROR("Could not successfully set all relevant nodes to the new mode"); }
    return true;
}


bool navigating(hobbit_msgs::SwitchVision::Request & request ,  hobbit_msgs::SwitchVision::Response & response  )
{
    ROS_INFO("Setting Vision System to Navigating");
     int i=0,target=0;
    ++target; i+=rosservice_call("/follow_user/pause");
    ++target; i+=rosservice_call("/emergency_detector/resume"); // So that we get
    //++target; i+=rosservice_call("/emergency_detector/looking_down"); // So that we get
    if (target==i) { response.result=true; } else { response.result=false; }
    if (!response.result) { ROS_ERROR("Could not successfully set all relevant nodes to the new mode"); }
    return true;
}

bool charging(hobbit_msgs::SwitchVision::Request & request ,  hobbit_msgs::SwitchVision::Response & response  )
{
    ROS_INFO("Setting Vision System to Charging , low processing mode");
    int i=0,target=0;
    ++target; i+=rosservice_call("/follow_user/pause");
    ++target; i+=rosservice_call("/skeleton_detector/resume");
    if (target==i) { response.result=true; } else { response.result=false; }
    if (!response.result) { ROS_ERROR("Could not successfully set all relevant nodes to the new mode"); }
    return true;
}


bool idle(hobbit_msgs::SwitchVision::Request & request ,  hobbit_msgs::SwitchVision::Response & response  )
{
    ROS_INFO("Setting Vision System to default idle mode");
    int i=0,target=0;
    ++target; i+=rosservice_call("/follow_user/pause");
    ++target; i+=rosservice_call("/skeleton_detector/resume");
    //++target; i+=rosservice_call("/emergency_detector/looking_center"); // So that we get
    if (target==i) { response.result=true; } else { response.result=false; }
    if (!response.result) { ROS_ERROR("Could not successfully set all relevant nodes to the new mode"); }
    return true;
}

bool startScanning3DObject(hobbit_msgs::SwitchVision::Request & request ,  hobbit_msgs::SwitchVision::Response & response  )
{
    ROS_INFO("Setting Vision System to 3D Scanning mode");
    return pauseEverything(request,response);
}

bool stopScanning3DObject(hobbit_msgs::SwitchVision::Request & request ,  hobbit_msgs::SwitchVision::Response & response  )
{
    ROS_INFO("Setting Vision System switching off from 3D Scanning mode");
    return resumeBasic(request,response);
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
     ros::ServiceServer triggerService  = nh.advertiseService(name+"/trigger", trigger);
     ros::ServiceServer stopService     = nh.advertiseService(name+"/terminate", terminate);
     ros::ServiceServer preciseService  = nh.advertiseService(name+"/precise", switchToPrecise);
     ros::ServiceServer rawService      = nh.advertiseService(name+"/raw", switchToRaw);


     ros::ServiceServer VSPauseService           = nh.advertiseService("/vision_system/pauseEverything", pauseEverything);
     ros::ServiceServer VSResumeService          = nh.advertiseService("/vision_system/resumeEverything", resumeEverything);
     ros::ServiceServer VSFollowUserService      = nh.advertiseService("/vision_system/followUser", followUser);
     ros::ServiceServer VSLocateUserService      = nh.advertiseService("/vision_system/locateUser", locateUser);
     ros::ServiceServer VSfitnessFunctionService = nh.advertiseService("/vision_system/fitnessFunction", fitnessFunction);
     ros::ServiceServer VSSeePointingService     = nh.advertiseService("/vision_system/seeWhereUserIsPointing", whereIsUserPointing);
     ros::ServiceServer VSNavigatingService      = nh.advertiseService("/vision_system/navigating", navigating);
     ros::ServiceServer VSIdleService            = nh.advertiseService("/vision_system/idle", idle);
     ros::ServiceServer VSComeCloserService      = nh.advertiseService("/vision_system/comeCloser", comeCloser);
     ros::ServiceServer VSChargingService        = nh.advertiseService("/vision_system/charging", charging);
     ros::ServiceServer VSStartScan3Service      = nh.advertiseService("/vision_system/startScanning3DObject", startScanning3DObject);
     ros::ServiceServer VSStopScan3Service       = nh.advertiseService("/vision_system/stopScanning3DObject", stopScanning3DObject);



     personBroadcaster = nh.advertise <person_aggregator::Person> ("persons", divisor);

     ros::Subscriber sub1 = nh.subscribe("/rgbd_acquisition/persons",divisor,personMessageRGBDAcquisition);
     ros::Subscriber sub2 = nh.subscribe("/skeleton_detector/persons",divisor,personMessageSkeletonDetector);
     ros::Subscriber sub3 = nh.subscribe("/face_detection/persons",divisor,personMessageFaceDetector);
     ros::Subscriber sub4 = nh.subscribe("/hand_gestures/persons",divisor,personMessageGestures);
     ros::Subscriber sub5 = nh.subscribe("/emergency_detector/persons",divisor,personMessageEmergency);
     ros::Subscriber sub6 = nh.subscribe("/follow_user/persons",divisor,personMessageFollowUser);

     //Create our context
     //---------------------------------------------------------------------------------------------------
	 //////////////////////////////////////////////////////////////////////////
	 unsigned int i=0;
     for (i=0; i<maxSource; i++) { StartTimer(i); }



	  while ( ( key!='q' ) && (ros::ok()) )
		{

                  ros::spinOnce();//<- this keeps our ros node messages handled up until synergies take control of the main thread
                  loop_rate.sleep();
	    }

	}
	catch(std::exception &e) { ROS_ERROR("Exception: %s", e.what()); return 1; }
	catch(...)               { ROS_ERROR("Unknown Error"); return 1; }
	ROS_ERROR("Shutdown complete");
	return 0;
}
