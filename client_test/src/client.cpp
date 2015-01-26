
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <interfaces_mira/MiraDockingAction.h>
#include <hobbit_msgs/GeneralHobbitAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "client_test");

  // create the action client
  actionlib::SimpleActionClient<hobbit_msgs::GeneralHobbitAction> ac("come_closer", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  hobbit_msgs::GeneralHobbitGoal goal;
  goal.command.data = "start";
  goal.parameters.resize(2);
  goal.parameters[0].data = "0.4";
  goal.parameters[1].data = "0.4";
  ac.sendGoal(goal); 

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
