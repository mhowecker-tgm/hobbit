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

#include "recognize_pose.h"
#include "recognize_excercise.h"
#include "process.h"
#include "hobbit_msgs/Fitness.h"


#define NODE_NAME "fitness_coordinator"
#define SKELETON_PREFIX "SK_"
#define MAX_CMD_STR 1024
#define MAX_NUM_STR 128

#define DEFAULT_FRAME_RATE 30

int rate=DEFAULT_FRAME_RATE;

int key = 0;
unsigned int frameTimestamp=0;
ros::NodeHandle * nhPtr=0;
unsigned int paused=0;
unsigned int autotrigger=0;
unsigned int rememberNext = 0;


struct fitnessState state;
struct skeletonHuman sk;


int collectSkeletonFromTF(struct skeletonHuman * sk)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;

  listener.waitForTransform(jointNames[0],"frame",ros::Time(0),ros::Duration(0.5));

  for (unsigned int i=0; i<HUMAN_SKELETON_PARTS; i++)
                  {
                    try
                    {
                     listener.lookupTransform(jointNames[i],"frame",ros::Time(0),transform);
                     sk->joint[i].x = transform.getOrigin().x();
                     sk->joint[i].y = transform.getOrigin().y();
                     sk->joint[i].z = transform.getOrigin().z();

                     //fprintf(stderr,"%s is %f %f %f \n",jointNames[i],sk->joint[i].x,sk->joint[i].y,sk->joint[i].z);
                    }
                    catch (...)
                    {
                       //fprintf(stderr,"... no tf ... \n");
                       return 0;
                    }
                  }
   return 1;
}


//----------------------------------------------------------
//Advertised Service switches
bool terminate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Stopping Node " NODE_NAME);
    exit(0);
    return true;
}


bool save(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    saveRememberedSkeletons("skeleton.sk");
    return true;
}



bool load(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    loadRememberedSkeletons("skeleton.sk");
    return true;
}



bool learn(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if (learnExcercise) { learnExcercise=0; } else
                        { learnExcercise=1; }
    return true;
}


bool remember(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    rememberNext=1;
    return true;
}

bool trigger(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    autotrigger=1;
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


void fitnessMessage(const hobbit_msgs::Fitness & msg)
{
 //Test Trigger with rostopic pub /fitness hobbit_msgs/Fitness " { command: C_EXERCISE_STARTED , params: [  { name: '1' , value: 'STARTED' } ] } " -1
  if (strcmp("C_EXERCISE_STARTED",msg.command.c_str())==0)
  {
   //if (strcmp("STARTED", msg.params[0].value.c_str() )==0)
   {
    state.started=1;
    state.repetitions=0;
    state.exercise=atoi(msg.params[0].name.c_str());
    fprintf(stderr,"Started Exercise %u \n",state.exercise);
    startExcercise(&state);
   }
  }
  else
 //Test Trigger with rostopic pub /fitness hobbit_msgs/Fitness " { command: C_EXERCISE_STOPPED , params: [  { name: '1' , value: 'STOPPED' } ] } " -1
  if (strcmp("C_EXERCISE_STOPPED",msg.command.c_str())==0)
  {
   //if (strcmp("STOPPED", msg.params[0].value.c_str() )==0)
   {
    state.started=0;
    state.repetitions=0;
    state.exercise=0;
    fprintf(stderr,"Stopped Exercise %u \n",state.exercise);
    stopExcercise(&state);
   }
  }
  else
  {
    fprintf(stderr,"Unknown command arrived %s \n",msg.command.c_str());
  }
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
     ros::ServiceServer saveService    = nh.advertiseService(name+"/save", save);
     ros::ServiceServer loadService    = nh.advertiseService(name+"/load", load);
     ros::ServiceServer pauseService    = nh.advertiseService(name+"/pause", pause);
     ros::ServiceServer resumeService   = nh.advertiseService(name+"/resume", resume);
     ros::ServiceServer stopService     = nh.advertiseService(name+"/terminate", terminate);
     ros::ServiceServer triggerService  = nh.advertiseService(name+"/trigger", trigger);
     ros::ServiceServer rememberService = nh.advertiseService(name+"/remember", remember);
     ros::ServiceServer learnService    = nh.advertiseService(name+"/learn", learn);


     ros::Subscriber sub = nh.subscribe("fitness",1000,fitnessMessage);

     if (!loadExercise("nothing.exercise"))
     {
       ROS_ERROR("Could not load fitness function presets , check file and rosservice call /fitness_coordinator/load");
     }


     if (!loadRememberedSkeletons("skeleton.sk"))
     {
       ROS_ERROR("Could not load fitness function presets , check file and rosservice call /fitness_coordinator/load");
     }
      //Create our context
      //---------------------------------------------------------------------------------------------------
	  //////////////////////////////////////////////////////////////////////////

	  int repetitions=0;
          state.started=1;
	  while ( ( key!='q' ) && (ros::ok()) )
		{
		          fprintf(stderr,".");
                  ros::spinOnce();//<- this keeps our ros node messages handled up until synergies take control of the main thread
                  loop_rate.sleep();

                  collectSkeletonFromTF(&sk);

                  if (rememberNext)
                  {
                     rememberSkeleton(&sk);
                     rememberNext=0;
                  }

                  checkSkeletonForRepetition(&state,&sk);




                  ++repetitions;
                  if ( (autotrigger) && (repetitions%40==0) && ( state.started ) )
                  {
                    ++state.repetitions;
                  }
	    }

	}
	catch(std::exception &e) { ROS_ERROR("Exception: %s", e.what()); return 1; }
	catch(...)               { ROS_ERROR("Unknown Error"); return 1; }
	ROS_ERROR("Shutdown complete");
	return 0;
}
