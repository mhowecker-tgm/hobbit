//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "std_srvs/Empty.h"
#include "follow_user/TrackerTarget.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "std_msgs/String.h"
#include <tf/tf.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

follow_user::TrackerTarget current_target;
geometry_msgs::PoseWithCovarianceStamped current_pose;

bool following_active;

ros::ServiceServer start_following_service;
ros::ServiceServer stop_following_service;

ros::ServiceClient resume_following_client;
ros::ServiceClient pause_following_client; 

std_msgs::String goal_status;
ros::Publisher status_pub;

bool new_pose;

void tracker_target_callback(const follow_user::TrackerTarget::ConstPtr& msg)
{
	current_target = (*msg);
        
}

void loc_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  current_pose = (*msg);
      
}

bool startFollowing(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{	
	ROS_INFO("person_following request received");
	following_active = true;
	std_srvs::Empty srv;
        if (!resume_following_client.call(srv))
        	ROS_DEBUG("Failed to call service resume following");
	return true;
}

bool stopFollowing(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{	
	ROS_INFO("stop_person_following request received");
	following_active = false;
	std_srvs::Empty srv;
        if (!pause_following_client.call(srv))
        	ROS_DEBUG("Failed to call service pause following");
	return true;
}

void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
	if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) 
	{
		goal_status.data = "reached";
	}
	if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) 
	{
		goal_status.data = "aborted";
	}
	if(state.state_ == actionlib::SimpleClientGoalState::PREEMPTED) 
	{
		goal_status.data = "preempted";
	}
	if(state.state_ == actionlib::SimpleClientGoalState::RECALLED) 
	{
		goal_status.data = "recalled";
	}
}

void goalActiveCallback()
{
	goal_status.data = "active";
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
	//ROS_INFO("Getting feedback");
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{
	ros::init(argc, argv, "person_following");
	ros::NodeHandle n;

	ros::Subscriber user_pose_sub = n.subscribe<follow_user::TrackerTarget>("/trackedTargets", 2, tracker_target_callback);
	ros::Subscriber current_loc_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 2, loc_pose_callback);

	start_following_service = n.advertiseService("/start_following", startFollowing);
	stop_following_service = n.advertiseService("/stop_following", stopFollowing);

	resume_following_client = n.serviceClient<std_srvs::Empty>("follow_user/resume");
	pause_following_client = n.serviceClient<std_srvs::Empty>("follow_user/pause");

	status_pub  = n.advertise<std_msgs::String>("/following_status", 20);

	following_active = false;
	
	float x_sensor = 0.135; //FIXME
	float dis2target = 1.0;

	// Tell the action client that we want to spin a thread by default
	MoveBaseClient ac("mira_move_base", true);

	ros::Rate r(10);
	// Wait for the action server to come up
	ROS_INFO("Waiting for the action server to come online...");
	if(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_FATAL("action server not running?");
		ROS_BREAK();
	}

	while (ros::ok())
	{

		status_pub.publish(goal_status);

		if(following_active)
		{

			double current_x = current_pose.pose.pose.position.x;
			double current_y = current_pose.pose.pose.position.y;
			double current_theta = tf::getYaw(current_pose.pose.pose.orientation);

			//since target reference system is: x right, y forward
			double global_x = current_x + (current_target.y+x_sensor)*cos(current_theta) + current_target.x*sin(current_theta);
		        double global_y = current_y + (current_target.y+x_sensor)*sin(current_theta) - current_target.x*cos(current_theta);

			// v = (vY, -vX)
			double target_x = global_x - dis2target * current_target.vY;
			double target_y = global_y + dis2target * current_target.vX;

			//std::cout << "target pose " << target_x << " " << target_y << std::endl;

			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.pose.position.x = target_x;
			goal.target_pose.pose.position.y = target_y;
			goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(-current_target.vX, current_target.vY));
			ac.sendGoal(goal, boost::bind(&goalDoneCallback, _1, _2), boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));

			



		}

		ros::spinOnce();
		r.sleep();
	}

	ROS_INFO("Exiting person_following...");
	return 0;

}




