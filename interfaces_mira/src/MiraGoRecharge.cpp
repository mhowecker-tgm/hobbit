#include <interfaces_mira/MiraGoRecharge.h>

#include <navigation/Task.h>
#include <navigation/tasks/PreferredDirectionTask.h>
#include <navigation/tasks/DockingTask.h>

#include <string>

using namespace mira::navigation;

MiraGoRecharge::MiraGoRecharge() : MiraRobotModule(std::string ("SendingGoal")) {
}

void MiraGoRecharge::initialize() {
        
  go_to_place_sub = robot_->getRosNode().subscribe(PLACE_NAME_TARGET, 1000, &MiraGoRecharge::go_to_place_callback, this);

  stop_sub = robot_->getRosNode().subscribe(STOP_REQUEST, 2, &MiraGoRecharge::stop_request_callback, this);

  goal_status_pub = robot_->getRosNode().advertise<std_msgs::String>(GOAL_STATUS, 20);

  robot_->getMiraAuthority().subscribe<std::string>("PilotEvent", &MiraGoRecharge::goal_status_channel_callback, this);

  goal_status.data = "idle";
  goal_status_pub.publish(goal_status);

}

void MiraGoRecharge::goal_status_channel_callback(mira::ChannelRead<std::string> data) 
{
        if(data->value().c_str() == "Idle") 
	{
		goal_status.data = "idle";
		goal_status_pub.publish(goal_status);
	}
	if(data->value().c_str() == "PlanAndDrive") 
	{
  		goal_status.data = "active";
		goal_status_pub.publish(goal_status);
	}
	if(data->value().c_str() =="GoalReached") 
	{
		goal_status.data = "reached";
		std::cout << "Goal reached " << std::endl;
		goal_status_pub.publish(goal_status);
	}
	if(data->value().c_str() == "PathTemporarilyLost") 
	{
		goal_status.data = "preempted";
		std::cout << "Goal preempted " << std::endl;
		goal_status_pub.publish(goal_status);
	}
	if(data->value().c_str() == "NoPathPlannable" || data->value().c_str() == "NoValidMotionCommand")
	{
		goal_status.data = "recalled";
		std::cout << "Goal recalled " << std::endl;
		goal_status_pub.publish(goal_status);

	}
	if(data->value().c_str() == "NoData")
	{
		goal_status.data = "recalled";
		std::cout << "Goal recalled " << std::endl;
		goal_status_pub.publish(goal_status);

	}


}

void MiraGoRecharge::go_to_place_callback(const std_msgs::String::ConstPtr& msg)
{

	if (msg->data.compare("docking_station") || msg->data.compare("DockingStation"))
	{

		TaskPtr task(new Task());
		task->addSubTask(SubTaskPtr(new PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 1.0f)));
		task->addSubTask(SubTaskPtr(new mira::navigation::DockingTask()));

		std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
		robot_->getMiraAuthority().callService<void>(navService, "setTask", task);
	}

}

void MiraGoRecharge::stop_request_callback(const std_msgs::String::ConstPtr& msg)
{
	if (msg->data.compare("stop") || msg->data.compare("Stop") || msg->data.compare("STOP"))
	{
		//cancel the task
		std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
  		robot_->getMiraAuthority().callService<void>(navService, "setTask", NULL);

		goal_status.data = "cancelled";
		std::cout << "Docking cancelled " << std::endl;
		goal_status_pub.publish(goal_status);
	}

}


