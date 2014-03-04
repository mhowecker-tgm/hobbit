#include <interfaces_mira/MiraSendingGoals.h>

#include <navigation/Task.h>
#include <navigation/tasks/OrientationTask.h>
#include <navigation/tasks/PreferredDirectionTask.h>
#include <navigation/tasks/PositionTask.h>

#include <string>

using namespace mira::navigation;

MiraSendingGoals::MiraSendingGoals() : MiraRobotModule(std::string ("SendingGoal")) {
}

void MiraSendingGoals::initialize() {
        
  goal_pose_subscriber = robot_->getRosNode().subscribe(GOAL_POSE, 1000, &MiraSendingGoals::goal_pose_callback, this);

  goal_status_pub = robot_->getRosNode().advertise<std_msgs::String>(GOAL_STATUS, 20);

  robot_->getMiraAuthority().subscribe<std::string>("PilotEvent", &MiraSendingGoals::goal_status_channel_callback, this);

  goal_status.data = "idle";

}

void MiraSendingGoals::goal_status_channel_callback(mira::ChannelRead<std::string> data) 
{
        if(data->value().c_str() == "Idle") 
	{
		goal_status.data = "idle";
		goal_status_pub.publish(goal_status);
	}
	if(data->value().c_str() == "PlanAndDrive") 
	{
		goal_status.data = "aborted";
		std::cout << "Goal aborted " << std::endl;

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

void MiraSendingGoals::goal_pose_callback(const hobbit_msgs::Pose2DStamped::ConstPtr& goal_pose) 
{

	TaskPtr task(new Task());
	task->addSubTask(SubTaskPtr(new PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 1.0f)));
        task->addSubTask(SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(goal_pose->x, goal_pose->y), 0.1f, 0.1f)));
	task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(goal_pose->theta, mira::deg2rad(15.0f))));

	std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
	robot_->getMiraAuthority().callService<void>(navService, "setTask", task);

}

void MiraSendingGoals::stop_request_callback(const std_msgs::String::ConstPtr& msg)
{
	if (msg->data.compare("stop") || msg->data.compare("Stop") || msg->data.compare("STOP"))
	{
		//cancel the task
		std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
  		robot_->getMiraAuthority().callService<void>(navService, "setTask", NULL);

		goal_status.data = "cancelled";
		std::cout << "Goal cancelled " << std::endl;
		goal_status_pub.publish(goal_status);
	}

}


