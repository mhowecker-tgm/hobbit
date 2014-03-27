#include <interfaces_mira/MiraGetPath.h>

#include <navigation/Task.h>
#include <navigation/tasks/PreferredDirectionTask.h>
#include <navigation/tasks/PositionTask.h>

using namespace mira::navigation;

MiraGetPath::MiraGetPath() : MiraRobotModule(std::string ("MiraGetPath")) {
}

void MiraGetPath::initialize() {

  get_path_service = robot_->getRosNode().advertiseService("/make_plan", &MiraGetPath::get_path, this);

  robot_->getMiraAuthority().subscribe<std::vector<mira::Point2f>>("navigation/Path", &MiraGetPath::path_channel_callback, this);

  new_plan = false;

}

void MiraGetPath::path_channel_callback(mira::ChannelRead<std::vector<mira::Point2f>> data) 
{
  global_plan = data;
  new_plan = true;

}

bool MiraGetPath::get_path(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp) 
{

  // convert target pose to Mira format
  mira::Point2f goal_pose_mira(req.goal.pose.position.x, req.goal.pose.position.y);

  // mute the pilot
  std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>(navService,"setMute", true);
  r.timedWait(mira::Duration::seconds(1));
  r.get();

  //Set the task for the target pose
  TaskPtr task(new Task());
  task->addSubTask(SubTaskPtr(new PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 1.0f)));
  task->addSubTask(SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(goal_pose_mira), 0.1f, 0.1f)));

  mira::RPCFuture<void> r1 = robot_->getMiraAuthority().callService<void>(navService, "setTask", task);
  r1.timedWait(mira::Duration::seconds(1));
  r1.get();
  
  // wait for new data on the "navigation/Path" channel  
  //FIXME
  int max_t = 500;
  int t = 0;
  while(!new_plan && t < max_t) t++;

  if (t>=max_t) return false;

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
  }


  //cancel the task
  TaskPtr empty_task(new Task());
  mira::RPCFuture<void> r2 = robot_->getMiraAuthority().callService<void>(navService, "setTask", empty_task);
  r2.timedWait(mira::Duration::seconds(1));
  r2.get();

  // unmute the pilot
  mira::RPCFuture<void> r3 = robot_->getMiraAuthority().callService<void>("setMute", false);
  r3.timedWait(mira::Duration::seconds(1));
  r3.get();

  return true;
}





