#include <interfaces_mira/MiraGoRecharge.h>

#include <navigation/Task.h>
#include <navigation/tasks/PreferredDirectionTask.h>
#include <navigation/tasks/DockingTask.h>

#include <rpc/RPCServer.h>

#include <string>

using namespace mira::navigation;

MiraGoRecharge::MiraGoRecharge() : MiraRobotModule(std::string ("GoRecharge")) {
}

void MiraGoRecharge::initialize() {
        
  go_to_place_sub = robot_->getRosNode().subscribe("/docking_task", 1000, &MiraGoRecharge::docking_task_callback, this);

  //stop_sub = robot_->getRosNode().subscribe(STOP_REQUEST, 2, &MiraGoRecharge::stop_request_callback, this);

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
		goal_status.data = "aborted";
		std::cout << "Goal aborted " << std::endl;
		goal_status_pub.publish(goal_status);

	}
	if(data->value().c_str() == "NoData")
	{
		goal_status.data = "aborted";
		std::cout << "Goal aborted " << std::endl;
		goal_status_pub.publish(goal_status);

	}


}

void MiraGoRecharge::docking_task_callback(const std_msgs::String::ConstPtr& msg)
{

	//if (msg->data.compare("docking_station") || msg->data.compare("DockingStation") || msg->data.compare("Recharge") || msg->data.compare("recharge"))

	std::cout << "data " << msg->data << std::endl;
	
	if (msg->data.compare("docking_on") == 0)
	{

		/*TaskPtr task(new Task());
		task->addSubTask(SubTaskPtr(new PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 1.0f)));
		task->addSubTask(SubTaskPtr(new mira::navigation::DockingTask()));

		std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
		robot_->getMiraAuthority().callService<void>(navService, "setTask", task);*/


		unsigned int s = 1; //FIXME

		std::cout << "Docking on" << std::endl;

		// Get docking service
		auto providers = robot_->getMiraAuthority().queryServicesForInterface("IDockingProcess");
		if(providers.empty()) 
		{
		    std::cout << "no providers for IDockingProcess" << std::endl;
		    return;
		}

		// Assume that our DockingProcess is the first and only available IDockingProcess provider.
		const std::string service = providers.front();

		// Let the robot dock on
		auto rpcFuture = robot_->getMiraAuthority().callService<void>(service, "dockOn", s);

		// Get the result, even though it is void. If an exception has occurred, this will throw as well.
		rpcFuture.get();
	}
	else if (msg->data.compare("docking_off") == 0)
	{
		const int s = 1; //FIXME

		std::cout << "Docking off" << std::endl;

		// Get docking service
		auto providers = robot_->getMiraAuthority().queryServicesForInterface("IDockingProcess");
		if(providers.empty()) 
		{
		    std::cout << "no providers for IDockingProcess" << std::endl;
		    return;
		}

		// Assume that our DockingProcess is the first and only available IDockingProcess provider.
		const std::string service = providers.front();

		//docking_off
		auto rpcFuture = robot_->getMiraAuthority().callService<void>(service, "dockOff", s);

	}

}

/*void MiraGoRecharge::stop_request_callback(const std_msgs::String::ConstPtr& msg)
{
	if (msg->data.compare("stop") || msg->data.compare("Stop") || msg->data.compare("STOP"))
	{
		TaskPtr task(new Task());
		//cancel the task
		std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
  		robot_->getMiraAuthority().callService<void>(navService, "setTask", task);

		goal_status.data = "cancelled";
		std::cout << "Docking cancelled " << std::endl;
		goal_status_pub.publish(goal_status);
	}

}*/


