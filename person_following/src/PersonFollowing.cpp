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

int count_num;
int no_new_goal_count;

void tracker_target_callback(const follow_user::TrackerTarget::ConstPtr& msg)
{
	current_target = (*msg);
	new_pose = true;
        
}

void loc_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  current_pose = (*msg);
      
}

bool startFollowing(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{	
	ROS_INFO("person_following request received");
	count_num = 0;
	no_new_goal_count = 0;

	following_active = true;
	std_srvs::Empty srv;
        if (!resume_following_client.call(srv))
        	ROS_DEBUG("Failed to call service resume following");

	goal_status.data = "started";
	status_pub.publish(goal_status);
	return true;
}

bool stopFollowing(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{	
	ROS_INFO("stop_person_following request received");
	following_active = false;
	std_srvs::Empty srv;
        if (!pause_following_client.call(srv))
        	ROS_DEBUG("Failed to call service pause following");

	goal_status.data = "stopped";
	status_pub.publish(goal_status);
	return true;
}

void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
	if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) 
	{
		following_active = false;
		goal_status.data = "reached";
		status_pub.publish(goal_status);
	}
	if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) 
	{
		goal_status.data = "aborted";
		status_pub.publish(goal_status);
	}
	/*if(state.state_ == actionlib::SimpleClientGoalState::PREEMPTED) 
	{
		goal_status.data = "preempted";
		status_pub.publish(goal_status);
	}
	if(state.state_ == actionlib::SimpleClientGoalState::RECALLED) 
	{
		goal_status.data = "recalled";
		status_pub.publish(goal_status);
	}*/
}

void goalActiveCallback()
{
	//goal_status.data = "active";
	//status_pub.publish(goal_status);
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

	double previous_target_x = current_pose.pose.pose.position.x;
	double previous_target_y = current_pose.pose.pose.position.y;
	
	double x_sensor = 0.135; //FIXME
	double dis2target = 1.0; //distance to the user to be kept
	double dis_thres = 0.2; //minimum distance between consecutive targets to be sent

	count_num = 0;
	int count_limit = 10; //number of loops to check if the velocity is close to zero
	double v_sum = 0;
	double v_thres = 0.2; //threshold value to consider if the velocity is close to zero

	new_pose = false;
	

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

		if(following_active)
		{

			//determine local target pose
			double local_dir = atan2(-current_target.x,current_target.y + x_sensor);			
			//since target reference system is: x right, y forward
			double local_target_x = current_target.y + x_sensor - dis2target*cos(local_dir);
			double local_target_y = -current_target.x - dis2target*sin(local_dir);

			//determine global target coordinates
			double current_x = current_pose.pose.pose.position.x;
			double current_y = current_pose.pose.pose.position.y;
			double current_theta = tf::getYaw(current_pose.pose.pose.orientation);
			double target_x = current_x + local_target_x*cos(current_theta) - local_target_y*sin(current_theta);
		        double target_y = current_y + local_target_x*sin(current_theta) + local_target_y*cos(current_theta);
			//check if new target is not too close to the previous one
			if ( (target_x-previous_target_x)*(target_x-previous_target_x) + (target_y-previous_target_y)*(target_y-previous_target_y) > dis_thres)
			{

				//std::cout << "target pose " << target_x << " " << target_y << std::endl;
				//send the new target
				move_base_msgs::MoveBaseGoal goal;
				goal.target_pose.pose.position.x = target_x;
				goal.target_pose.pose.position.y = target_y;
				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(target_y-current_y, target_x-current_x));
				ac.sendGoal(goal, boost::bind(&goalDoneCallback, _1, _2), boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1)); 
				//update current target
				previous_target_x = target_x;
				previous_target_y = target_y; 
				
				

			}
			else no_new_goal_count++;

			//check if the average velocity is close to zero
			v_sum += current_target.vX*current_target.vX + current_target.vY*current_target.vY;
			count_num++;
			if (count_num == count_limit)
			{
				if (v_sum < v_thres*v_thres*count_limit )
				{	
					std::cout << "speed is low, stopping " << std::endl;
					following_active = false;
					goal_status.data = "stopped";
					status_pub.publish(goal_status);
				}

				count_num = 0;
			}
			
		}

		ros::spinOnce();
		r.sleep();
	}

	ROS_INFO("Exiting person_following...");
	return 0;

}




