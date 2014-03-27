/* MiraGoRecharge.h
* Author: Paloma de la Puente
*
* Wrapper to send the robot to the charging station
*/

#ifndef MiraGoRecharge_H
#define MiraGoRecharge_H

#define PLACE_NAME_TARGET "place_name_target"
#define STOP_REQUEST "stop_request"
#define GOAL_STATUS "goal_status"


//#include <fw/Framework.h>

#include <ros/ros.h>
#include <interfaces_mira/MiraRobotModule.h>
#include <interfaces_mira/MiraRobot.h>

#include <navigation/INavigation.h> 

#include <hobbit_msgs/Pose2DStamped.h>
#include <std_msgs/String.h>


class MiraGoRecharge: public MiraRobotModule {
public:
        static MiraRobotModule* Create() {
                return new MiraGoRecharge();
        }

        void initialize();

	void docking_task_callback(const std_msgs::String::ConstPtr& msg);

        //void stop_request_callback(const std_msgs::String::ConstPtr& msg);  //FIXME?

        void goal_status_channel_callback(mira::ChannelRead<std::string> data);

private:
        MiraGoRecharge();


        ros::Subscriber go_to_place_sub;
	ros::Subscriber stop_sub;

	ros::Publisher goal_status_pub;

	std_msgs::String goal_status;

};


#endif

