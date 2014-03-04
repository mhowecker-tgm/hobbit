/* MiraSendingGoals.h
* Author: Paloma de la Puente
*
* Wrapper to send goals from ROS to MIRA
*/

#ifndef MIRASENDINGGOALS_H
#define MIRASENDINGGOALS_H

#define GOAL_POSE "goal_pose"
#define GOAL_STATUS "goal_status"

//#include <fw/Framework.h>

#include <ros/ros.h>
#include <interfaces_mira/MiraRobotModule.h>
#include <interfaces_mira/MiraRobot.h>

#include <navigation/INavigation.h> 

#include <hobbit_msgs/Pose2DStamped.h>
#include <std_msgs/String.h>


class MiraSendingGoals: public MiraRobotModule {
public:
        static MiraRobotModule* Create() {
                return new MiraSendingGoals();
        }

        void initialize();

	void goal_pose_callback(const hobbit_msgs::Pose2DStamped::ConstPtr& goal_pose);

        void stop_request_callback(const std_msgs::String::ConstPtr& msg);

        void goal_status_channel_callback(mira::ChannelRead<std::string> data);

private:
        MiraSendingGoals();

	ros::Subscriber goal_pose_subscriber;

	ros::Publisher goal_status_pub;

	std_msgs::String goal_status;

};


#endif

