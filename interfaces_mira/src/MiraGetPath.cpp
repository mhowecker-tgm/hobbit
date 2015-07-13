#include <interfaces_mira/MiraGetPath.h>

#include <navigation/Task.h>
#include <navigation/tasks/PreferredDirectionTask.h>
#include <navigation/tasks/PositionTask.h>
#include <navigation/tasks/IgnoreObstaclesTask.h>
#include <mira_msgs/ObsNavMode.h>

using namespace mira::navigation;

MiraGetPath::MiraGetPath() : MiraRobotModule(std::string ("MiraGetPath")) {
}

void MiraGetPath::initialize() {

  get_path_service = robot_->getRosNode().advertiseService("/make_plan", &MiraGetPath::get_path, this);

  robot_->getMiraAuthority().subscribe<std::vector<mira::Point2f>>("/navigation/Path", &MiraGetPath::path_channel_callback, this);

  new_plan = false;

  global_plan.clear();


}

void MiraGetPath::path_channel_callback(mira::ChannelRead<std::vector<mira::Point2f>> data) 
{
  global_plan = data;
  new_plan = true;

}

bool MiraGetPath::get_path(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp) 
{

  new_plan = false;
  global_plan.clear();

  // convert target pose to Mira format
  mira::Point2f goal_pose_mira(req.goal.pose.position.x, req.goal.pose.position.y);

  std::cout << "req.goal.position " << req.goal.pose.position.x << " " << req.goal.pose.position.y << std::endl;

  // mute the pilot
  std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>(navService,"setMute", true);
  r.timedWait(mira::Duration::seconds(1));
  r.get();

  std::cout << "setting the task" << std::endl;

  //Set the task for the target pose
  TaskPtr task(new Task());
  //task->addSubTask(SubTaskPtr(new PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 1.0f)));
  
  task->addSubTask(SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(goal_pose_mira), 0.1f, 0.1f)));
   task->addSubTask(SubTaskPtr(new mira::navigation::IgnoreObstaclesTask()));

  mira::RPCFuture<void> r1 = robot_->getMiraAuthority().callService<void>(navService, "setTask", task);


  /*mira::Pose2 new_goal_target(req.goal.pose.position.x, req.goal.pose.position.y, 0.0);
	  robot_->getMiraAuthority().callService<void>(navService, "setGoal", new_goal_target, 0.1f, mira::deg2rad(5.0f));*/

  /*r1.timedWait(mira::Duration::seconds(1));
  r1.get();*/

 //std::cout << "before waiting" << std::endl;
  
  // wait for new data on the "navigation/Path" channel  for a given duration (10 secs)

  ros::Time timeout = ros::Time::now() + ros::Duration(10);
  while(!new_plan || global_plan.size()==0)
  {
	ros::Duration time_left = timeout - ros::Time::now();
	if (time_left <= ros::Duration(0,0))  //the timeout has been reached
		break;
  }

  if (new_plan)
	std::cout << "new plan was received " << std::endl;

  // Extract the planned path, copy the path into the response
  resp.plan.poses.resize(global_plan.size());
  for(unsigned int i = 0; i < global_plan.size(); ++i)
  {
	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "map";
	pose.pose.position.x = global_plan[i][0];
	pose.pose.position.y = global_plan[i][1];

	resp.plan.poses[i] = pose;
	//std::cout << "pose " << pose << std:endl;
  }

  std::cout << "resp size " << resp.plan.poses.size() << std::endl;

  //cancel the task
  TaskPtr empty_task(new Task());
  mira::RPCFuture<void> r2 = robot_->getMiraAuthority().callService<void>(navService, "setTask", empty_task);
  /*r2.timedWait(mira::Duration::seconds(1));
  r2.get();*/

  //std::cout << "task cancelled" << std::endl;

  // unmute the pilot

  mira::RPCFuture<void> r3 = robot_->getMiraAuthority().callService<void>(navService,"setMute", false);
  r3.timedWait(mira::Duration::seconds(1));
  r3.get();


  std::cout << "pilot is back" << std::endl;

  return true;
}





