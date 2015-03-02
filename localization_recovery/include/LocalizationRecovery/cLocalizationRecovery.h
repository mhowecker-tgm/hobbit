#ifndef LOCALIZATIONRECOVERY_H
#define LOCALIZATIONRECOVERY_H

#include <ros/ros.h>

#include "std_srvs/Empty.h"
#include "std_msgs/String.h"

#include <actionlib/server/simple_action_server.h>
#include <hobbit_msgs/GeneralHobbitAction.h>

class cLocalizationRecovery
{
public:

  	//Constructor
  	cLocalizationRecovery(int argc, char **argv);
  	~cLocalizationRecovery();

	int init_argc;
  	char **init_argv;

  	actionlib::SimpleActionServer<hobbit_msgs::GeneralHobbitAction>* as_;
	void executeCb(const hobbit_msgs::GeneralHobbitGoalConstPtr& goal);

private:
	int numb;
  	ros::ServiceClient check_rotation_client;
	ros::ServiceClient get_nav_mode_client;

  	ros::Publisher discrete_motion_cmd_pub;
  	bool started_rotation;
  	bool finished_rotation;
  	ros::Subscriber motion_state_sub;
  	void motion_state_callback(const std_msgs::String::ConstPtr& msg);
  	std_msgs::String current_motion_state;

	ros::ServiceClient emergency_stop_client;
	ros::ServiceClient reset_motorstop_client;
	//int numb;


};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


