/* MiraSendingGoals.h
* Author: Paloma de la Puente
*
* Wrapper to send goals from ROS to MIRA
*/

#ifndef MIRASENDINGGOALS_H
#define MIRASENDINGGOALS_H

//#define GOAL_POSE "goal_pose"
//#define GOAL_STATUS "goal_status"

//#include <fw/Framework.h>

#include <ros/ros.h>
#include <interfaces_mira/MiraRobotModule.h>
#include <interfaces_mira/MiraRobot.h>

#include <navigation/INavigation.h> 

#include <hobbit_msgs/Pose2DStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <actionlib/server/simple_action_server.h>
#include <interfaces_mira/MiraSendingGoalsAction.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <mira_msgs/UserNavMode.h>
#include <mira_msgs/ObsNavMode.h>

#include "hobbit_msgs/GetState.h"
#include <maps/GridMap.h>
#include <maps/OccupancyGrid.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "hobbit_msgs/GetOccupancyGrid.h"

#include "std_srvs/Empty.h"
#include "hobbit_msgs/GetPose.h"
#include "hobbit_msgs/SendPose.h"
#include "hobbit_msgs/SendValue.h"

#define COST_LIMIT 127


class MiraSendingGoals: public MiraRobotModule {
public:
        static MiraRobotModule* Create() {
                return new MiraSendingGoals();
        }

        void initialize();

        //void stop_request_callback(const std_msgs::String::ConstPtr& msg);

        void goal_status_channel_callback(mira::ChannelRead<std::string> data);

	void bumper_callback(const std_msgs::Bool::ConstPtr& msg);

	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>* as2_; 

private:
        MiraSendingGoals();
	~MiraSendingGoals();

	//ros::Subscriber stop_sub;

	ros::Publisher goal_status_pub;

	std_msgs::String goal_status;

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

	//ros::Publisher loc_check_pub;

	ros::ServiceServer cancel_goal_service;
	bool cancelGoal(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

	ros::ServiceServer local_map_service;
	bool getLocalMap(hobbit_msgs::GetOccupancyGrid::Request  &req, hobbit_msgs::GetOccupancyGrid::Response &res);

	void cancelGoal();

	ros::Subscriber bumper_subs;

	bool is_bumper_pressed;

	bool is_goal_active;
	ros::ServiceServer is_goal_active_srv;
	bool isGoalActive(hobbit_msgs::GetState::Request  &req, hobbit_msgs::GetState::Response &res);

	geometry_msgs::PoseStamped last_goal;
	ros::ServiceServer get_last_goal_srv;
	bool getLastGoal(hobbit_msgs::GetPose::Request  &req, hobbit_msgs::GetPose::Response &res);
	
	ros::ServiceServer cancel_mira_goal_service;
	bool cancelMiraGoal(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

	bool cancel_received;

	ros::ServiceServer send_mira_goal_service;
	bool sendMiraGoal(hobbit_msgs::SendPose::Request  &req, hobbit_msgs::SendPose::Response &res);

	ros::ServiceServer apply_rotation_service;
	bool applyRotation(hobbit_msgs::SendValue::Request  &req, hobbit_msgs::SendValue::Response &res);

};


#endif

