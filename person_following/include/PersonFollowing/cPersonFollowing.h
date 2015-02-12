#ifndef PERSONFOLLOWING_H
#define PERSONFOLLOWING_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "std_srvs/Empty.h"
#include "follow_user/TrackerTarget.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "std_msgs/String.h"
#include <tf/tf.h>

#include <actionlib/server/simple_action_server.h>
#include <hobbit_msgs/FollowMeAction.h>

#include <ctime>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class cPersonFollowing
{
public:

  	//Constructor
  	cPersonFollowing(int argc, char **argv);
  	~cPersonFollowing();

	int init_argc;
  	char **init_argv;

  	actionlib::SimpleActionServer<hobbit_msgs::FollowMeAction>* as_;

  	double x_sensor, dis2target, dis_thres, v_thres, it_limit, time_limit_secs;
	double time_limit_no_feedback_secs;

  	ros::Subscriber user_pose_sub;
  	ros::Subscriber current_loc_sub;

	follow_user::TrackerTarget current_target;
	geometry_msgs::PoseWithCovarianceStamped current_pose;

	bool following_active;

	ros::ServiceServer start_following_service;
	ros::ServiceServer stop_following_service;

	ros::ServiceClient resume_following_client;
	ros::ServiceClient pause_following_client; 

	ros::ServiceClient cancel_goal_client;


	std_msgs::String goal_status;
	ros::Publisher status_pub;

	void init();
	void tracker_target_callback(const follow_user::TrackerTarget::ConstPtr& msg);
	void loc_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	bool startFollowing();
	bool startFollowing(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
	bool stopFollowing();
	bool stopFollowing(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

	void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);

	void executeCb(const hobbit_msgs::FollowMeGoalConstPtr& goal);

private:
	bool new_pose; //new pose spearated from previous one
	bool new_target; //new target received

	int count_num;
	double v_sum;

	bool starting;

	double previous_target_x;
	double previous_target_y;

	bool response;

	ros::ServiceClient deactivate_recovery_client;
	ros::ServiceClient activate_recovery_client;
 

};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


