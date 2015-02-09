

#include <interfaces_mira/MiraSendingGoals.h>

#include <navigation/Task.h>
#include <navigation/tasks/OrientationTask.h>
#include <navigation/tasks/PreferredDirectionTask.h>
#include <navigation/tasks/PositionTask.h>

#include <tf/transform_broadcaster.h>

#include <string>
#include <ctime>

#include "occupancy_grid_utils/coordinate_conversions.h"

using namespace mira;
using namespace mira::navigation;

MiraSendingGoals::MiraSendingGoals() : MiraRobotModule(std::string ("SendingGoal")), as_(NULL) 
{
	dis_covered_sq = 0;
	dis_thres = 3;

	loc_check_active = false;

	outer_dis = 0.55;

	is_bumper_pressed = false;

	loc_status = true;
}

void MiraSendingGoals::initialize() {
        
  goal_pose_subscriber = robot_->getRosNode().subscribe("/goal_pose", 1000, &MiraSendingGoals::goal_pose_callback, this);
  //stop_sub = robot_->getRosNode().subscribe("/stop_request", 2, &MiraSendingGoals::stop_request_callback, this);

  bumper_subs = robot_->getRosNode().subscribe("/bumper", 2, &MiraSendingGoals::bumper_callback, this);

  goal_status_pub = robot_->getRosNode().advertise<std_msgs::String>("/goal_status", 20);

  robot_->getMiraAuthority().subscribe<std::string>("/navigation/PilotEvent", &MiraSendingGoals::goal_status_channel_callback, this);

  goal_status.data = "idle";
  goal_status_pub.publish(goal_status);


  as_ = new actionlib::SimpleActionServer<interfaces_mira::MiraSendingGoalsAction>(robot_->getRosNode(), "mira_sending_goals", boost::bind(&MiraSendingGoals::executeCb, this, _1), false);
  as2_ = new actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>(robot_->getRosNode(), "mira_move_base", boost::bind(&MiraSendingGoals::executeCb2, this, _1), false);
  as_->start();
  as2_->start();

  user_nav_mode_service = robot_->getRosNode().advertiseService("/user_nav_mode", &MiraSendingGoals::user_nav_mode, this);
  obs_nav_mode_service = robot_->getRosNode().advertiseService("/obs_nav_mode", &MiraSendingGoals::obs_nav_mode, this);

  //get current DecayRate value
  mira::RPCFuture<std::string> r1 = robot_->getMiraAuthority().callService<std::string>("/navigation/laser/GridMapperLaser#builtin", std::string("getProperty"), std::string("DecayRate"));
  r1.timedWait(mira::Duration::seconds(1));
  decay_value = r1.get();
  std::cout << "current decay rate " << decay_value << std::endl;

  //get current MaxRange value
  mira::RPCFuture<std::string> r2 = robot_->getMiraAuthority().callService<std::string>("/navigation/laser/GridMapperLaser#builtin", std::string("getProperty"), std::string("MaxRange"));
  r2.timedWait(mira::Duration::seconds(1));
  max_range_value = r2.get();

  robot_->getMiraAuthority().subscribe<mira::maps::OccupancyGrid>("/navigation/MergedMap",   &MiraSendingGoals::local_map_callback, this); //FIXME, service?? 
  check_rotation_service = robot_->getRosNode().advertiseService("/check_rotation", &MiraSendingGoals::checkRotationStatus, this);

  loc_status_client = robot_->getRosNode().serviceClient<hobbit_msgs::GetState>("/get_loc_status");

  loc_status_sub = robot_->getRosNode().subscribe<std_msgs::Bool>("/loc_ok", 2, &MiraSendingGoals::loc_status_callback, this);

  discrete_motion_cmd_pub = robot_->getRosNode().advertise<std_msgs::String>("/discrete_motion_cmd", 20);

  //loc_check_pub = robot_->getRosNode().advertise<std_msgs::Bool>("/loc_check_state", 20);

  cancel_goal_service = robot_->getRosNode().advertiseService("/cancel_goal", &MiraSendingGoals::cancelGoal, this);

  local_map_service = robot_->getRosNode().advertiseService("/get_local_map", &MiraSendingGoals::getLocalMap, this);


}

void MiraSendingGoals::spin() 
{
	// ros::spin();
	ros::Rate r(10);  //10 Hz
	while (ros::ok()) 
	{
		ros::spinOnce();
		r.sleep();
		goal_status_pub.publish(goal_status);
	}
}

bool MiraSendingGoals::user_nav_mode(mira_msgs::UserNavMode::Request &req, mira_msgs::UserNavMode::Response &res)
{

	std::cout << "user_mode_received, should only be used for small distances and simple motions " << std::endl;

	mira::RPCFuture<void> r1 = robot_->getMiraAuthority().callService<void>("/navigation/laser/GridMapperLaser#builtin", std::string("setProperty"), std::string("MaxRange"), std::string("0.1"));
        r1.timedWait(mira::Duration::seconds(1));
        r1.get();

        mira::RPCFuture<void> r2 = robot_->getMiraAuthority().callService<void>("/navigation/laser/GridMapperLaser#builtin", std::string("setProperty"), std::string("DecayRate"), std::string("0.0"));
        r2.timedWait(mira::Duration::seconds(1));
        r2.get();
        return true;

}

bool MiraSendingGoals::obs_nav_mode(mira_msgs::ObsNavMode::Request &req, mira_msgs::ObsNavMode::Response &res)
{

	std::cout << "obs_mode_received " << std::endl;

	mira::RPCFuture<void> r1 = robot_->getMiraAuthority().callService<void>("/navigation/laser/GridMapperLaser#builtin", std::string("setProperty"), std::string("MaxRange"), std::string(max_range_value));
        r1.timedWait(mira::Duration::seconds(1));
        r1.get();

	mira::RPCFuture<void> r2 = robot_->getMiraAuthority().callService<void>("/navigation/laser/GridMapperLaser#builtin", std::string("setProperty"), std::string("DecayRate"), std::string(decay_value));
        r2.timedWait(mira::Duration::seconds(1));
        r2.get();
	return true;

}

void MiraSendingGoals::goal_status_channel_callback(mira::ChannelRead<std::string> data) 
{
	//std::cout << "goal_status " << data->value().c_str() << std::endl;
        if(data->value() == "Idle") 
	{
		goal_status.data = "idle";
		goal_status_pub.publish(goal_status);
	}
	if(data->value() == "PlanAndDrive") 
	{
  		goal_status.data = "active";
		goal_status_pub.publish(goal_status);
	}
	if(data->value() =="GoalReached") 
	{
		goal_status.data = "reached";
		std::cout << "Goal reached " << std::endl;
		goal_status_pub.publish(goal_status);
	}
	if(data->value() == "PathTemporarilyLost") 
	{
		goal_status.data = "preempted";
		std::cout << "Goal preempted " << std::endl;
		goal_status_pub.publish(goal_status);
	}
	if(data->value() == "NoPathPlannable" || data->value() == "NoValidMotionCommand")
	{
		goal_status.data = "aborted";
		std::cout << "Goal aborted " << std::endl;
		goal_status_pub.publish(goal_status);

	}

	if(data->value() == "TaskFailed")
	{
		goal_status.data = "aborted";
		std::cout << "Goal aborted " << std::endl;
		goal_status_pub.publish(goal_status);

	}

	if(data->value() == "NoData")
	{
		goal_status.data = "aborted";
		std::cout << "Goal aborted " << std::endl;
		goal_status_pub.publish(goal_status);

	}


}

void MiraSendingGoals::goal_pose_callback(const hobbit_msgs::Pose2DStamped::ConstPtr& goal_pose) 
{

	TaskPtr task(new Task());
	task->addSubTask(SubTaskPtr(new PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 1.0f)));
        task->addSubTask(SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(goal_pose->x, goal_pose->y), 0.1f, 0.1f)));
	task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(goal_pose->theta, mira::deg2rad(10.0f))));

	std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
	robot_->getMiraAuthority().callService<void>(navService, "setTask", task);

}

/*void MiraSendingGoals::stop_request_callback(const std_msgs::String::ConstPtr& msg)
{
	if (msg->data.compare("stop")==0 || msg->data.compare("Stop")==0 || msg->data.compare("STOP")==0)
	{
		std::cout << "stop command received " << std::endl;
		cancelGoal();
	}

}*/

bool MiraSendingGoals::cancelGoal(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	ROS_INFO("cancel goal request received");
	cancelGoal();
	
}

void MiraSendingGoals::cancelGoal()
{

	TaskPtr task(new Task());
	//cancel the task
	std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
  	robot_->getMiraAuthority().callService<void>(navService, "setTask", task);

	goal_status.data = "aborted";
	std::cout << "Goal cancelled " << std::endl;
	goal_status_pub.publish(goal_status);	

}

void MiraSendingGoals::bumper_callback(const std_msgs::Bool::ConstPtr& msg)
{
	if (msg->data && !is_bumper_pressed) //bumper was hit
	{
		std::cout << "bumper was hit, cancelling goal " << std::endl;
		cancelGoal();
	}

	is_bumper_pressed = msg->data;

}



void MiraSendingGoals::executeCb(const interfaces_mira::MiraSendingGoalsGoalConstPtr& goal_pose)
{

       if(!isQuaternionValid(goal_pose->target_pose.pose.orientation))
    {
      as2_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    geometry_msgs::PoseStamped goal = goal_pose->target_pose; //should be in global reference frame already!!
    goal_status.data = "idle";
    goal_status_pub.publish(goal_status);

    std::cout << "goal actionlib x:" << goal.pose.position.x << " y: " << goal.pose.position.y << " theta " << tf::getYaw(goal.pose.orientation)*180/M_PI << std::endl;

     TaskPtr goal_task(new Task());
     goal_task->addSubTask(SubTaskPtr(new PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 1.0f)));
     goal_task->addSubTask(SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(goal.pose.position.x, goal.pose.position.y), 0.1f, 0.1f)));
     goal_task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(tf::getYaw(goal.pose.orientation), mira::deg2rad(10.0f))));

     std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
     robot_->getMiraAuthority().callService<void>(navService, "setTask", goal_task);

    ros::NodeHandle n = robot_->getRosNode();
    while(n.ok())
    {

      //std::cout << "actionlib server executeCb ok" << std::endl;
      if(as2_->isPreemptRequested())
      {
	std::cout << "preempt requested" << std::endl;
        
	TaskPtr task(new Task());
        //cancel the task
        std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
        robot_->getMiraAuthority().callService<void>(navService, "setTask", task);

        goal_status.data = "cancelled";
        std::cout << "Previous goal cancelled " << std::endl;
        goal_status_pub.publish(goal_status);

        if(as2_->isNewGoalAvailable())
	{
	  std::cout << "new goal available" << std::endl;
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::MoveBaseGoal new_goal = *as2_->acceptNewGoal();

          if(!isQuaternionValid(new_goal.target_pose.pose.orientation))
	  {
            as2_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

          goal = new_goal.target_pose;
	  //std::cout << "goal actionlib loop" << goal.pose.position.x << " y: " << goal.pose.position.y << " theta " << tf::getYaw(goal.pose.orientation)*180/M_PI<< std::endl;


          //we have a new goal so make sure the planner is awake
          TaskPtr new_goal_task(new Task());
	  new_goal_task->addSubTask(SubTaskPtr(new PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 1.0f)));
          new_goal_task->addSubTask(SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(goal.pose.position.x, goal.pose.position.y), 0.1f, 0.1f)));
	  new_goal_task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(tf::getYaw(goal.pose.orientation), mira::deg2rad(10.0f))));

	  std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
	  robot_->getMiraAuthority().callService<void>(navService, "setTask", new_goal_task);
          

        }
        else 
	{

          //notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("interfaces_mira","preempting the current goal");
          as2_->setPreempted();
          std::cout << "goal preempted, preemt received" << std::endl;

           //we'll actually return from execute after preempting
          return;
        }
     }

     bool done = false;
     if (goal_status.data == "reached") done=true;

     if(done)
     {
	//std::cout << "done " << std::endl;
	as2_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
        return;
     }

     if (goal_status.data == "preempted")
     {
	as2_->setPreempted();
	std::cout << "goal preempted, path temporarily lost" << std::endl;
	return;
     }
  
     if (goal_status.data == "aborted")
     {
     	as2_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal, probably because the path is blocked or it was cancelled");
	return;
     }

   }

   //if the node is killed then we'll abort and return
    as2_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;


}


bool MiraSendingGoals::isQuaternionValid(const geometry_msgs::Quaternion& q)  //copied from ros move_base
{
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly //FIXME
    /*tf_q.normalize();

    btVector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }*/

    return true;


}

void MiraSendingGoals::executeCb2(const move_base_msgs::MoveBaseGoalConstPtr& goal_pose)
{

    if(!isQuaternionValid(goal_pose->target_pose.pose.orientation))
    {
      as2_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    geometry_msgs::PoseStamped goal = goal_pose->target_pose; //should be in global reference frame already!!
    goal_status.data = "idle";
    goal_status_pub.publish(goal_status);

    std::cout << "goal actionlib x:" << goal.pose.position.x << " y: " << goal.pose.position.y << " theta " << tf::getYaw(goal.pose.orientation)*180/M_PI << std::endl;

     TaskPtr goal_task(new Task());
     goal_task->addSubTask(SubTaskPtr(new PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 1.0f)));
     goal_task->addSubTask(SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(goal.pose.position.x, goal.pose.position.y), 0.1f, 0.1f)));
     goal_task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(tf::getYaw(goal.pose.orientation), mira::deg2rad(10.0f))));

     std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
     robot_->getMiraAuthority().callService<void>(navService, "setTask", goal_task);

    ros::NodeHandle n = robot_->getRosNode();
    while(n.ok())
    {

	if(loc_check_active)
	{
		
		bool loc_ok = loc_status;	
		
		//check if rotation is safe //FIXME, service to get LocalMap, get outer circle...
		if (!loc_ok)
		{
			// call service, remember obstacles
			/*mira_msgs::UserNavMode srv_user;
			user_nav_mode(srv_user.request, srv_user.response);*/

			//cancel the task
			TaskPtr task(new Task());
			std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
			robot_->getMiraAuthority().callService<void>(navService, "setTask", task);

			if (isRotationSafe())
			{
				// rotate
				std_msgs::String rotate_cmd;
				rotate_cmd.data = "Turn 180";
		  		discrete_motion_cmd_pub.publish(rotate_cmd);
				//wait until rotation is finished and the drive mode is back to normal (0)
				double time_limit = 30;
				clock_t begin = clock();
				double elapsed_secs = 0;
				//TODO, check current and initial orientation?
				while (std::stoi(get_mira_param_("MainControlUnit.DriveMode")) && elapsed_secs < time_limit) //FIXME?
				{
					clock_t end = clock();
		  			elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
				} 
				//sleep (1);

				//call service 
				hobbit_msgs::GetState srv;
				if (loc_status_client.call(srv))
				{
				  //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
				  loc_ok = srv.response.state;
				}
				else
				{
				  ROS_DEBUG("Failed to call service get_loc_state");
				}

				//resend the goal
				if (loc_ok)
				{
					//resend the goal
					robot_->getMiraAuthority().callService<void>(navService, "setTask", goal_task);
					loc_status = true;
					continue;
				}

			}

			// call service, forget obstacles
			/*mira_msgs::ObsNavMode srv_obs;
			obs_nav_mode(srv_obs.request, srv_obs.response);*/

			std::cout << "The robot is lost!!!!!!! " << std::endl;  //and it has already rotated or cannot rotate
			// TODO notify that the robot is lost, stop navigation?? !!!! //TODO TODO TODO
		}
      }

      //std::cout << "actionlib server executeCb ok" << std::endl;
      if(as2_->isPreemptRequested())
      {
	std::cout << "preempt requested" << std::endl;
        
	
        //cancel the task
 	/*TaskPtr task(new Task());
        std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
        robot_->getMiraAuthority().callService<void>(navService, "setTask", task);

        goal_status.data = "cancelled";
        std::cout << "Previous goal cancelled " << std::endl;
        goal_status_pub.publish(goal_status);*/

        if(as2_->isNewGoalAvailable())
	{
	  std::cout << "new goal available" << std::endl;
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::MoveBaseGoal new_goal = *as2_->acceptNewGoal();

          if(!isQuaternionValid(new_goal.target_pose.pose.orientation))
	  {
            as2_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

          goal = new_goal.target_pose;
	  std::cout << "goal actionlib loop, x: " << goal.pose.position.x << " y: " << goal.pose.position.y << " theta " << tf::getYaw(goal.pose.orientation)*180/M_PI<< std::endl;

          //we have a new goal so make sure the planner is awake
          TaskPtr new_goal_task(new Task());
	  new_goal_task->addSubTask(SubTaskPtr(new PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 1.0f)));
          new_goal_task->addSubTask(SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(goal.pose.position.x, goal.pose.position.y), 0.1f, 0.1f)));
	  new_goal_task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(tf::getYaw(goal.pose.orientation), mira::deg2rad(10.0f))));

	  std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
	  //robot_->getMiraAuthority().callService<void>(navService, "setTask", new_goal_task);
	  mira::Pose2 new_goal_target(goal.pose.position.x, goal.pose.position.y, tf::getYaw(goal.pose.orientation));
	  robot_->getMiraAuthority().callService<void>(navService, "setGoal", new_goal_target, 0.1f, mira::deg2rad(10.0f));

	  //std::cout << "The new goal task has been set " << std::endl;
         

        }
        else 
	{
	  //cancel the task
	  TaskPtr task(new Task());
	  std::string navService = robot_->getMiraAuthority().waitForServiceInterface("INavigation");
          robot_->getMiraAuthority().callService<void>(navService, "setTask", task);
          goal_status.data = "cancelled";
          std::cout << "Previous goal cancelled " << std::endl;
          goal_status_pub.publish(goal_status);

          //notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("interfaces_mira","preempting the current goal");
          as2_->setPreempted();
          std::cout << "goal preempted, preemt received" << std::endl;

           //we'll actually return from execute after preempting
           return;
        }
     }

     bool done = false;
     if (goal_status.data == "reached") done=true;

     if(done)
     {
	//std::cout << "done " << std::endl;
	as2_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
        return;
     }

     if (goal_status.data == "preempted")
     {
	as2_->setPreempted();
	std::cout << "goal preempted, path temporarily lost" << std::endl;
	return;
     }
  
     if (goal_status.data == "aborted")
     {
     	as2_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal, probably because the path is blocked");
	std::cout << "goal aborted, probably because the path is blocked" << std::endl;
	return;
     }

   }

   //if the node is killed then we'll abort and return
    as2_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;


}


void MiraSendingGoals::local_map_callback(mira::ChannelRead<mira::maps::OccupancyGrid> data) 
{
	local_map = (*data);
  
}

bool MiraSendingGoals::getLocalMap(hobbit_msgs::GetOccupancyGrid::Request  &req, hobbit_msgs::GetOccupancyGrid::Response &res)
{
	
	nav_msgs::OccupancyGrid local_grid;
	
	local_grid.info.resolution =local_map.getCellSize();
	local_grid.info.width = local_map.width();
	local_grid.info.height = local_map.height();

	mira::Point2i grid_point(0,0);
	mira::Point2f rel_point = local_map.map2world(grid_point);
	const mira::Pose2 global_origin = robot_->getMiraAuthority().getTransform<mira::Pose2>("/robot/OdometryFrame", "/maps/MapFrame", Time::now()) * mira::Pose2(rel_point(0), rel_point(1), 0.0f);

	local_grid.info.origin.position.x = global_origin.x();
	local_grid.info.origin.position.y =global_origin.y();
	local_grid.info.origin.orientation =tf::createQuaternionMsgFromYaw(global_origin.phi());

	local_grid.data.resize(local_grid.info.width*local_grid.info.height);
	int index = 0;
	for (int i=0; i<local_grid.info.width; i++)
	{
		for (int j=0; j< local_grid.info.height; j++)
		{

			if (local_map.data()[index] > 127)
				local_grid.data[index] = occupancy_grid_utils::OCCUPIED;
			else if (local_map.data()[index] < 127)
				local_map.data()[index] = occupancy_grid_utils::UNOCCUPIED;
			else local_map.data()[index] = occupancy_grid_utils::UNKNOWN;

			index++;
		}
	}


	res.occupancy_grid = local_grid;
	
	return true;
}


bool MiraSendingGoals::checkRotationStatus(hobbit_msgs::GetState::Request  &req, hobbit_msgs::GetState::Response &res)
{
	
	ROS_INFO("check_rotation_state request received");

	bool rotation_ok = isRotationSafe();
	std::cout << "rotation_ok " << rotation_ok << std::endl;

	res.state = (rotation_ok);

	ROS_INFO("sending back check_rotation_state response");
        std::cout << "********************* " << std::endl;


	return true;

}

bool MiraSendingGoals::isRotationSafe()
{
	//call service to get MergedMap
	//local_map should be updated...

	mira::Point2f loc_offset = local_map.getWorldOffset();
	/*std::cout << "loc_offset " << loc_offset.x() << " " << loc_offset.y() << std::endl;

	std::cout << "local_map.width() " << local_map.width() << std::endl;
	std::cout << "local_map.height() " << local_map.height() << std::endl;

	std::cout << "local_map.channels() " << local_map.channels() << std::endl;*/


	int ind = 0;
	for(int i=0; i<local_map.width();i++)
	{
		for(int j=0; j<local_map.height();j++)
	 	{
			mira::Point2i grid_point(i,j);
			mira::Point2f rel_point = local_map.map2world(grid_point); //FIXME check, local coord?
			//std::cout << "grid point " << grid_point.x() << " " << grid_point.y() << std::endl;
			//std::cout << "rel point " << rel_point.x() << " " << rel_point.y() << std::endl;

			const mira::PoseCov2 global_pose = robot_->getMiraAuthority().getTransform<mira::PoseCov2>("/robot/RobotFrame", "/maps/MapFrame", Time::now());
			double current_x = global_pose.x();
			double current_y = global_pose.y();

			const mira::Pose2 global_point = robot_->getMiraAuthority().getTransform<mira::Pose2>("/robot/OdometryFrame", "/maps/MapFrame", Time::now()) * mira::Pose2(rel_point(0), rel_point(1), 0.0f);

			double global_x = global_point.x();
		        double global_y = global_point.y();

			//std::cout << "glob point " << global_x << " " << global_y << std::endl;

			if ((global_x-current_x)*(global_x-current_x) + (global_y-current_y)*(global_y-current_y) < outer_dis*outer_dis)
			{
				double cost = local_map.data()[ind];
				if (cost > COST_LIMIT) //FIXME, check costs
					return false;
			}
			
			ind++;
			
			
			

	 	}
	}

	return true;
}

void MiraSendingGoals::loc_status_callback(const std_msgs::Bool::ConstPtr& msg)
{
    loc_status = msg->data;
}



