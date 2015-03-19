/* MiraGetPose.h
* Author: Paloma de la Puente
*
* Module to publish the localization pose from Mira as a ROS topic
*/

#ifndef MIRAGETPOSE_H
#define MIRAGETPOSE_H


//#include <fw/Framework.h>

#include <ros/ros.h>
#include <interfaces_mira/MiraRobotModule.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <interfaces_mira/MiraRobot.h>

#include <transform/RigidTransform.h>
#include <navigation/INavigation.h> 

#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_broadcaster.h"

#include "std_srvs/Empty.h"
#include "hobbit_msgs/GetState.h"

class MiraGetPose: public MiraRobotModule {
public:
        static MiraRobotModule* Create() {
                return new MiraGetPose();
        }

        void initialize();

	ros::ServiceServer reset_loc_service;
	ros::ServiceServer is_docked_service;


private:
        MiraGetPose();

	ros::Publisher current_pose_pub;
	void loc_pose_callback(mira::ChannelRead<mira::Pose2> data);


	geometry_msgs::TransformStamped robot_trans;
        tf::TransformBroadcaster *p_robot_broadcaster;

	bool resetLoc(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

	mira::PoseCov2 docking_pose;

	geometry_msgs::PoseWithCovarianceStamped pose_msg;

        bool atDockingStation(hobbit_msgs::GetState::Request  &req, hobbit_msgs::GetState::Response &res);

};


#endif

