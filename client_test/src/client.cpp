
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <interfaces_mira/MiraDockingAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "client_test");

  // create the action client
  actionlib::SimpleActionClient<interfaces_mira::MiraDockingAction> ac("docking_task", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  std::cout << "sending to recharge " << std::endl;
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  interfaces_mira::MiraDockingGoal goal;
  goal.docking_task = 0;
  ac.sendGoal(goal); 

  /*sleep(30);
  ac.cancelGoal();*/

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
