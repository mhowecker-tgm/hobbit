#ifndef COMECLOSER_H
#define COMECLOSER_H

#include <ros/ros.h>

#include "std_srvs/Empty.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "std_msgs/String.h"
#include <tf/tf.h>

#include <actionlib/server/simple_action_server.h>
#include <hobbit_msgs/GeneralHobbitAction.h>

#include "hobbit_msgs/GetOccupancyGrid.h"

#include <ctime>

#include "hobbit_msgs/Event.h"

class cComeCloser
{
public:

  	//Constructor
  	cComeCloser(int argc, char **argv);
  	~cComeCloser();

	int init_argc;
  	char **init_argv;

  	actionlib::SimpleActionServer<hobbit_msgs::GeneralHobbitAction>* as_;
	void executeCb(const hobbit_msgs::GeneralHobbitGoalConstPtr& goal);

	double margin;
	double front_dis;
	double x_sensor;
	double time_limit_secs;

	double x_offset;

private:

	ros::Publisher discrete_motion_cmd_pub;

	ros::ServiceClient get_local_map_client;

	nav_msgs::OccupancyGrid local_grid;
	
	geometry_msgs::PoseWithCovarianceStamped currentPose;
	void loc_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        ros::Subscriber current_loc_sub;

	ros::Publisher laserPublisher; //FIXME, only for initial testing

	ros::Subscriber motion_state_sub;
	void motion_state_callback(const std_msgs::String::ConstPtr& msg);
	std_msgs::String current_motion_state;

	bool started_rotation;
	bool finished_rotation;
	bool started_movement;
	bool finished_movement;
	
	double ang_margin;

	bool movement_cmd_sent;
 
	ros::Subscriber event_sub;
	void event_callback(const hobbit_msgs::Event::ConstPtr& msg);
	ros::ServiceClient reset_motorstop_client;

	double dis_thres;
	double ang_thres;

	double range_max;

};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


