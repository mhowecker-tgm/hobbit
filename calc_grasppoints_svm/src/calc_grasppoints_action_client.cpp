#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <hobbit_msgs/CalcGraspPointsServerAction.h>


//ROS includes
#include <ros/package.h>
#include "sensor_msgs/PointCloud2.h"



class CCalcGrasppointsClient
{
public:
	ros::Subscriber pc_sub;

	void save_pc_cb(const sensor_msgs::PointCloud2ConstPtr& pc_in);

	CCalcGrasppointsClient(ros::NodeHandle nh_)
	{

		//subscriber for the pointcloud
		this->pc_sub = nh_.subscribe("/SS/points2_object_in_rcs",1, &CCalcGrasppointsClient::save_pc_cb, this);
	}
};


void CCalcGrasppointsClient::save_pc_cb(const sensor_msgs::PointCloud2ConstPtr& pc_in)
{
	ROS_INFO("\nFrom calc_grasppoints_action_client: point cloud received");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<hobbit_msgs::CalcGraspPointsServerAction> ac("calc_grasppoints_svm_action_server", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	hobbit_msgs::CalcGraspPointsServerGoal goal;
	goal.input_pc = *pc_in;
	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(50.0));

	if (finished_before_timeout)
	{
	    actionlib::SimpleClientGoalState state = ac.getState();
	    ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	    ROS_INFO("Action did not finish before the time out.");

}

int main (int argc, char **argv)
{
  ROS_INFO("ROS NODE calc_grasppoints_client started");
  ros::init(argc, argv, "calc_grasppoint_client");
  ros::NodeHandle nh_;
  CCalcGrasppointsClient grasp_client(nh_);

  ros::spin();
  //exit
  return 0;
}


