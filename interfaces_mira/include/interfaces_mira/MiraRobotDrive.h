/* MiraRobotDrive.h
* Author: chris burbridge
*
* Module for interfacing all related to drive: odometry, motor controller state etc etc
* http://www.mira-project.org/MIRA-doc/domains/robot/SCITOS/index.html#Drive_Section
*/

//Modified by Paloma de la Puente

#ifndef ROBOTBASE_H
#define ROBOTBASE_H

//#include <ros/ros.h>
//#include <fw/Framework.h>

#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"

#include <robot/Odometry.h> //# MIRA odometry

#include <mira_msgs/ResetMotorStop.h>
#include <mira_msgs/ResetOdometry.h>
#include <mira_msgs/EmergencyStop.h>
#include <mira_msgs/EnableMotors.h>
#include <mira_msgs/MotorStatus.h>

#include <mira_msgs/GetBoolValue.h>

#include <interfaces_mira/MiraRobotModule.h>

class MiraRobotDrive: public MiraRobotModule {
public:
static MiraRobotModule* Create() {
return new MiraRobotDrive();
}

void initialize();

void velocity_command_callback(const geometry_msgs::Twist::ConstPtr& msg);
void discrete_motion_cmd_callback(const std_msgs::String::ConstPtr& msg);

void odometry_data_callback(mira::ChannelRead<mira::robot::Odometry2> data);

void bumper_data_callback(mira::ChannelRead<bool> data);
void mileage_data_callback(mira::ChannelRead<float> data);
void motor_status_callback(mira::ChannelRead<uint8> data);

bool reset_motor_stop(mira_msgs::ResetMotorStop::Request &req, mira_msgs::ResetMotorStop::Response &res);
bool reset_odometry(mira_msgs::ResetOdometry::Request &req, mira_msgs::ResetOdometry::Response &res);
bool emergency_stop(mira_msgs::EmergencyStop::Request &req, mira_msgs::EmergencyStop::Response &res);
bool enable_motors(mira_msgs::EnableMotors::Request &req, mira_msgs::EnableMotors::Response &res);

bool get_nav_mode(mira_msgs::GetBoolValue::Request &req, mira_msgs::GetBoolValue::Response &res);

private:
MiraRobotDrive();
ros::Subscriber cmd_vel_subscriber_;
ros::Subscriber discrete_sub;

ros::Publisher odometry_pub_;
ros::Publisher bumper_pub_;
ros::Publisher mileage_pub_;
ros::Publisher motorstatus_pub_;
ros::ServiceServer reset_motor_stop_service_;
ros::ServiceServer reset_odometry_service_;
ros::ServiceServer emergency_stop_service_;
ros::ServiceServer enable_motors_service_;

ros::ServiceServer get_nav_mode_service_;

ros::Publisher state_pub;

boost::thread callback_queue_thread_;
void QueueThread();

};

#endif
