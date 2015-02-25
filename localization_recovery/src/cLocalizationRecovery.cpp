#include "ros/ros.h"
#include "../include/LocalizationRecovery/cLocalizationRecovery.h"

#include <mira_msgs/ResetMotorStop.h>
#include <mira_msgs/EmergencyStop.h>

#include "hobbit_msgs/GetState.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor. Initialises member attributes and allocates resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cLocalizationRecovery::cLocalizationRecovery(int argc, char **argv) : init_argc(argc), init_argv(argv)
{
	as_ = new actionlib::SimpleActionServer<hobbit_msgs::GeneralHobbitAction>(ros::NodeHandle(), "localization_recovery", boost::bind(&cLocalizationRecovery::executeCb, this, _1), false);

	ros::NodeHandle nh("~");
	nh.param("numb", numb, 1);

	ros::NodeHandle n;
	discrete_motion_cmd_pub = n.advertise<std_msgs::String>("/DiscreteMotionCmd", 20);

	motion_state_sub = n.subscribe<std_msgs::String>("/DiscreteMotionState", 2, &cLocalizationRecovery::motion_state_callback, this);

        as_->start();

	current_motion_state.data = "Idle";

	emergency_stop_client = n.serviceClient<mira_msgs::EmergencyStop>("/emergency_stop");
  	reset_motorstop_client = n.serviceClient<mira_msgs::ResetMotorStop>("/reset_motorstop");

 	check_rotation_client = n.serviceClient<hobbit_msgs::GetState>("/check_rotation");

	discrete_motion_cmd_pub = n.advertise<std_msgs::String>("/DiscreteMotionCmd", 20);
	motion_state_sub = n.subscribe<std_msgs::String>("/DiscreteMotionState", 2, &cLocalizationRecovery::motion_state_callback, this);

	std::cout << "numb " << numb << std::endl;


}

cLocalizationRecovery::~cLocalizationRecovery()
{
  printf("cLocalizationRecovery::~cLocalizationRecovery(): shutting down ROS\n");
  usleep(100000);
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  usleep(100000);
  printf(" - done\n");

  if(as_ != NULL)
  	delete as_;
}


void cLocalizationRecovery::motion_state_callback(const std_msgs::String::ConstPtr& msg)
{
  current_motion_state = (*msg);  

  if (!started_rotation && current_motion_state.data == "Turning")
  	started_rotation = true;
  else if (started_rotation && current_motion_state.data == "Idle")
	finished_rotation = true;
}



void cLocalizationRecovery::executeCb(const hobbit_msgs::GeneralHobbitGoalConstPtr& goal)
{

	std::cout << "localization_recovery callback " << std::endl;

	if (goal->command.data != "start")
	{
		as_->setAborted(hobbit_msgs::GeneralHobbitResult(), "aborted");
		return;
	}

	started_rotation = false;
	finished_rotation = false;

	std::cout << "check rotation " << std::endl;

	//check if rotation is safe
	bool rotation_is_safe = false;

	hobbit_msgs::GetState get_state_srv;
	if (check_rotation_client.call(get_state_srv))
	{
		rotation_is_safe = get_state_srv.response.state;
		std::cout << "rot is safe " << rotation_is_safe << std::endl;
	}
	else
	{
		ROS_DEBUG("Failed to call service check_rotation");
		
	}

	if (rotation_is_safe)
	{
		std::cout << "rotation seems ok " << std::endl;
		// rotate
		std_msgs::String rotate_cmd;
		rotate_cmd.data = "Turn 180";
  		discrete_motion_cmd_pub.publish(rotate_cmd);
	}
	
	else
	{
		as_->setAborted(hobbit_msgs::GeneralHobbitResult(), "aborted, rotation not safe");
		std::cout << "aborted, rotation not safe" << std::endl; 
		return;

	}

	int rot_numb = 0;

	//std::cout << "numb " << numb << std::endl;


	///////////////////////////////////////////////////////////////////////////////////////
	ros::NodeHandle n;
	while (n.ok())
	{

		if (finished_rotation)
		{
			rot_numb++;
			std::cout << "finished rotation " << rot_numb << std::endl;
			//std::cout << "numb " << numb << std::endl;

			if (rot_numb < numb)
			{
				started_rotation = false;
				finished_rotation = false;
				sleep(1);

				std_msgs::String rotate_cmd;
				rotate_cmd.data = "Turn 180";
  				discrete_motion_cmd_pub.publish(rotate_cmd);
				
			}
			else
			{
				//discrete motion finished
				sleep(1);
				as_->setSucceeded(hobbit_msgs::GeneralHobbitResult(), "succeded");
				std::cout << "rotation finished, succeeded " << numb << std::endl; 
				return;

			}
				

		}

		if(as_->isPreemptRequested())
      		{
			std::cout << "preempt requested" << std::endl;
			
			if(current_motion_state.data == "Turning")
			{

				mira_msgs::EmergencyStop srv;
				if (!emergency_stop_client.call(srv))
				{
				  ROS_DEBUG("Failed to call service stop_request");
				}

				sleep(5);

				mira_msgs::ResetMotorStop srv2;
				if (!reset_motorstop_client.call(srv2))
				{
				  ROS_DEBUG("Failed to call service reset_motorstop");
				}
			}

			as_->setPreempted();
			return;

		}

	}

	//if the node is killed then we'll abort and return
        as_->setAborted(hobbit_msgs::GeneralHobbitResult(), "Aborting on the goal because the node has been killed");
        return;

}






