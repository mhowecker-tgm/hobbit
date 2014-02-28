/* MiraSendingGoals.h
* Author: Paloma de la Puente
*
* Wrapper to send goals from ROS to MIRA
*/

#ifndef MIRASENDINGGOALS_H
#define MIRASENDINGGOALS_H

#define GOAL_POSE "goal_pose"

//#include <fw/Framework.h>

#include <ros/ros.h>
#include <interfaces_mira/MiraRobotModule.h>
#include <interfaces_mira/MiraRobot.h>

#include <navigation/INavigation.h> 

#include <hobbit_msgs/Pose2DStamped.h>


class MiraSendingGoals: public MiraRobotModule {
public:
        static MiraRobotModule* Create() {
                return new MiraSendingGoals();
        }

        void initialize();

	void goal_pose_callback(const hobbit_msgs::Pose2DStamped::ConstPtr& goal_pose);

private:
        MiraSendingGoals();

	ros::Subscriber goal_pose_subscriber;

};


#endif

