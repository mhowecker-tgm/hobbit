/* MiraSendingGoals.h
* Author: Paloma de la Puente
*
* Wrapper to send goals from ROS to MIRA
*/

#ifndef MIRASENDINGGOALS_H
#define MIRASENDINGGOALS_H

#define GOAL_POSE "goal_pose"
#define GOAL_STATUS "goal_status"
#define STOP_REQUEST "stop_request"

//#include <fw/Framework.h>

#include <ros/ros.h>
#include <interfaces_mira/MiraRobotModule.h>
#include <interfaces_mira/MiraRobot.h>

#include <navigation/INavigation.h> 

#include <hobbit_msgs/Pose2DStamped.h>
#include <std_msgs/String.h>

#include <actionlib/server/simple_action_server.h>
#include <interfaces_mira/MiraSendingGoalsAction.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <mira_msgs/UserNavMode.h>
#include <mira_msgs/ObsNavMode.h>

#include "hobbit_msgs/GetState.h"
#include <maps/GridMap.h>
#include <maps/OccupancyGrid.h>

#define COST_LIMIT 127


class MiraSendingGoals: public MiraRobotModule {
public:
        static MiraRobotModule* Create() {
                return new MiraSendingGoals();
        }

        void initialize();

	void goal_pose_callback(const hobbit_msgs::Pose2DStamped::ConstPtr& goal_pose);

        void stop_request_callback(const std_msgs::String::ConstPtr& msg);

        void goal_status_channel_callback(mira::ChannelRead<std::string> data);

	actionlib::SimpleActionServer<interfaces_mira::MiraSendingGoalsAction>* as_; 
	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>* as2_; 

private:
        MiraSendingGoals();

	ros::Subscriber goal_pose_subscriber;
	ros::Subscriber stop_sub;

	ros::Publisher goal_status_pub;

	std_msgs::String goal_status;

	void executeCb(const interfaces_mira::MiraSendingGoalsGoalConstPtr& goal_pose);
	void executeCb2(const move_base_msgs::MoveBaseGoalConstPtr& goal_pose);

        bool isQuaternionValid(const geometry_msgs::Quaternion& q);

        void spin();

	ros::ServiceServer user_nav_mode_service;
	ros::ServiceServer obs_nav_mode_service;

	bool user_nav_mode(mira_msgs::UserNavMode::Request &req, mira_msgs::UserNavMode::Response &res);
	bool obs_nav_mode(mira_msgs::ObsNavMode::Request &req, mira_msgs::ObsNavMode::Response &res);
	std::string decay_value;
	std::string max_range_value;

	ros::ServiceServer check_rotation_service;
	bool checkRotationStatus(hobbit_msgs::GetState::Request  &req, hobbit_msgs::GetState::Response &res);

	bool isRotationSafe();
	double outer_dis;
	mira::maps::OccupancyGrid local_map;
	void local_map_callback(mira::ChannelRead<mira::maps::OccupancyGrid> data);

};


#endif

