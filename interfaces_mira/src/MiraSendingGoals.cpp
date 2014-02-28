#include <interfaces_mira/MiraSendingGoals.h>

#include <navigation/Task.h>
#include <navigation/tasks/OrientationTask.h>
#include <navigation/tasks/PreferredDirectionTask.h>
#include <navigation/tasks/PositionTask.h>

using namespace mira::navigation;

MiraSendingGoals::MiraSendingGoals() : MiraRobotModule(std::string ("SendingGoal")) {
}

void MiraSendingGoals::initialize() {
        
  goal_pose_subscriber = robot_->getRosNode().subscribe(GOAL_POSE, 1000, &MiraSendingGoals::goal_pose_callback, this);

  //virtual_laser_channel_loc = robot_->getMiraAuthority().publish<mira::robot::RangeScan>("/VirtualLaserLoc");

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




