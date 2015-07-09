/* MiraGetPath.h
* Author: Paloma de la Puente
*
* Module to get the path from the robot to a given goal
*/

#ifndef MIRAGETPATH_H
#define MIRAGETPATH_H


//#include <fw/Framework.h>

#include <ros/ros.h>
#include <interfaces_mira/MiraRobotModule.h>

#include <interfaces_mira/MiraRobot.h>

#include <navigation/INavigation.h> 

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/GetPlan.h"

#include <geometry/Point.h>


class MiraGetPath: public MiraRobotModule {
public:
        static MiraRobotModule* Create() {
                return new MiraGetPath();
        }

        void initialize();

        bool get_path(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

private:
        MiraGetPath();

        ros::ServiceServer get_path_service;
        std::vector<mira::Point2f> global_plan;

        bool new_plan;

        void path_channel_callback(mira::ChannelRead<std::vector<mira::Point2f>> data);

};


#endif

