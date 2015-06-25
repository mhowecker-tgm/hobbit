/* MiraGoRecharge.h
* Author: Paloma de la Puente
*
* Wrapper to send the robot to the charging station
*/

#ifndef MiraGoRecharge_H
#define MiraGoRecharge_H

//#define PLACE_NAME_TARGET "place_name_target"
#define STOP_REQUEST "stop_request"
#define GOAL_STATUS "goal_status"


//#include <fw/Framework.h>

#include <ros/ros.h>
#include <interfaces_mira/MiraRobotModule.h>
#include <interfaces_mira/MiraRobot.h>

#include <navigation/INavigation.h> 

#include <hobbit_msgs/Pose2DStamped.h>
#include <std_msgs/String.h>

#include <actionlib/server/simple_action_server.h>
#include <interfaces_mira/MiraDockingAction.h>

#include <mira_msgs/BatteryState.h>

class MiraGoRecharge: public MiraRobotModule {
public:
        static MiraRobotModule* Create() {
                return new MiraGoRecharge();
        }

        void initialize();

        //void stop_request_callback(const std_msgs::String::ConstPtr& msg);  //FIXME?

        void status_channel_callback(mira::ChannelRead<std::string> data);
	void template_channel_callback(mira::ChannelRead<std::string> data);

	ros::Subscriber batterySubs;

	void battery_state_callback(const mira_msgs::BatteryState::ConstPtr& msg);

private:
        MiraGoRecharge();
	~MiraGoRecharge();

	std_msgs::String status;

	void executeCb(const interfaces_mira::MiraDockingGoalConstPtr& docking_task);
	actionlib::SimpleActionServer<interfaces_mira::MiraDockingAction>* as_;

	bool status_updated;

	bool template_found;

	ros::Time timeout;

	bool is_charging;
};


#endif
