/* MiraRobot.h
* Author: chris burbridge
  Modified by Paloma de la Puente
*
* Main class for the robot. Instantiates modules present on the robot, manages
* MIRA and ROS handles.
*
*/

#ifndef MIRAROBOT_H
#define MIRAROBOT_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <fw/Framework.h>
#include <interfaces_mira/MiraRobotModule.h>
#include <vector>
#include <string>

#include <ros/callback_queue.h>

class MiraRobot{
public:
  MiraRobot(std::vector<std::string> modules);
  ~MiraRobot();

  mira::Authority& getMiraAuthority();
  ros::NodeHandle& getRosNode();
  ros::CallbackQueue& getMyQueue();

  void initialize();
  void spin();

  void registerSpinFunction(boost::function<void ()> function);
  tf::TransformBroadcaster& getTFBroadcaster();

private:
  mira::Authority authority_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::NodeHandle node_;
  std::vector<MiraRobotModule*> modules_;
  std::vector< boost::function<void ()> > spin_functions_;

  ros::CallbackQueue my_queue; 

};

#endif
