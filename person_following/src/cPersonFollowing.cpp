#include "ros/ros.h"
#include "../include/PersonFollowing/cPersonFollowing.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor. Initialises member attributes and allocates resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cPersonFollowing::cPersonFollowing(int argc, char **argv) : init_argc(argc), init_argv(argv)
{
	as_ = new actionlib::SimpleActionServer<hobbit_msgs::FollowMeAction>(ros::NodeHandle(), "person_following", boost::bind(&cPersonFollowing::executeCb, this, _1), false);

	ros::NodeHandle nh;
	nh.param("x_sensor", x_sensor, 0.135);
	nh.param("dis2target", dis2target, 1.0);
	nh.param("dis_thres", dis_thres, 0.2);
	nh.param("v_thres", v_thres, 0.2);
	//nh.param("it_limit", it_limit, 10.0);
	nh.param("time_limit_secs", time_limit_secs, 20.0);

	ros::NodeHandle n;
	user_pose_sub = n.subscribe<follow_user::TrackerTarget>("/trackedTargets", 2, &cPersonFollowing::tracker_target_callback, this);
	current_loc_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 2, &cPersonFollowing::loc_pose_callback, this);

	start_following_service = n.advertiseService("/start_following", &cPersonFollowing::startFollowing, this);
	stop_following_service = n.advertiseService("/stop_following", &cPersonFollowing::stopFollowing, this);

	resume_following_client = n.serviceClient<std_srvs::Empty>("follow_user/resume");
	pause_following_client = n.serviceClient<std_srvs::Empty>("follow_user/pause");

	cancel_goal_client = n.serviceClient<std_srvs::Empty>("cancel_goal");

	status_pub  = n.advertise<std_msgs::String>("/following_status", 20);

	following_active = false;

	init();

        as_->start();
	


}

cPersonFollowing::~cPersonFollowing()
{
  printf("cPersonFollowing::~cPersonFollowing(): shutting down ROS\n");
  usleep(100000);
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  usleep(100000);
  printf(" - done\n");
}

void cPersonFollowing::init()
{

	std::cout << "init " << std::endl;
	starting = true;
	count_num = 0;
	v_sum = 0;
	new_pose = false;
	new_target = false;	

	previous_target_x = current_pose.pose.pose.position.x;
	previous_target_y = current_pose.pose.pose.position.y;



}


void cPersonFollowing::tracker_target_callback(const follow_user::TrackerTarget::ConstPtr& msg)
{
	current_target = (*msg);
	//std::cout << "target received " << std::endl;
	new_target = true;
        
}

void cPersonFollowing::loc_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  current_pose = (*msg);
      
}

bool cPersonFollowing::startFollowing()
{

	init();

	following_active = true;
	std_srvs::Empty srv;
        if (!resume_following_client.call(srv))
        	ROS_DEBUG("Failed to call service resume following");

	goal_status.data = "started";
	status_pub.publish(goal_status);
	return true;

}

bool cPersonFollowing::startFollowing(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	ROS_INFO("person_following request received");
	return startFollowing();	
	
}

bool cPersonFollowing::stopFollowing()
{
	following_active = false;
	std_srvs::Empty srv;
        if (!pause_following_client.call(srv))
        	ROS_DEBUG("Failed to call service pause following");

	goal_status.data = "stopped";
	status_pub.publish(goal_status);
	return true;

}

bool cPersonFollowing::stopFollowing(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	ROS_INFO("stop_person_following request received");
	return stopFollowing();	
	
}

void cPersonFollowing::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
	if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) 
	{
		following_active = false;
		goal_status.data = "reached";
		status_pub.publish(goal_status);
	}
	if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) 
	{
		goal_status.data = "aborted";
		status_pub.publish(goal_status);
	}
	if(state.state_ == actionlib::SimpleClientGoalState::PREEMPTED) 
	{
		goal_status.data = "preempted";
		status_pub.publish(goal_status);
	}
	if(state.state_ == actionlib::SimpleClientGoalState::RECALLED) 
	{
		goal_status.data = "recalled";
		status_pub.publish(goal_status);
	}

	std::cout << "goal state " << state.state_ << std::endl;

}


void cPersonFollowing::executeCb(const hobbit_msgs::FollowMeGoalConstPtr& goal)
{

	if (goal->command == "start")
	{
		startFollowing();
	}

	else return;

	clock_t begin;
	clock_t begin_target;

	// Tell the action client that we want to spin a thread by default
	MoveBaseClient ac("mira_move_base", true);

	ros::Rate r(10);
	// Wait for the action server to come up
	ROS_INFO("Waiting for the action server to come online...");
	if(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_FATAL("action server not running?");
		ROS_BREAK();
	}

	ros::NodeHandle n;
	while (n.ok())
	{

		if(following_active)
		{


			if (!new_target) //no new target
			{
				if (starting) //the follow-me behavior is just starting
				{
					begin_target = clock();
					std::cout << "starting, no new target " << std::endl;
					starting = false;
					continue;

				}
				
				else 
				{
					//check the time
					clock_t end_target = clock();
		  			double elapsed_secs_target = double(end_target - begin_target) / CLOCKS_PER_SEC;
					//std::cout << "no new target " << std::endl;
					if (elapsed_secs_target >= time_limit_secs) //the timeout has been reached
					{
						//the user has been lost for a while, so the following behaviour is stopped
						std::cout << "no target for a while, stopping " << std::endl;
						stopFollowing();
						as_->setAborted(hobbit_msgs::FollowMeResult(), "Follow-me aborted, no target was detected at all"); //
						return;
					}
					else
						continue;

				}

			}

			//at least one target was detected

			//determine local target pose
			double local_dir = atan2(-current_target.x,current_target.y + x_sensor);			
			//since target reference system is: x right, y forward
			double local_target_x = current_target.y + x_sensor - dis2target*cos(local_dir);
			double local_target_y = -current_target.x - dis2target*sin(local_dir);

			//determine global target coordinates
			double current_x = current_pose.pose.pose.position.x;
			double current_y = current_pose.pose.pose.position.y;
			double current_theta = tf::getYaw(current_pose.pose.pose.orientation);
			double target_x = current_x + local_target_x*cos(current_theta) - local_target_y*sin(current_theta);
		        double target_y = current_y + local_target_x*sin(current_theta) + local_target_y*cos(current_theta);

			//std::cout << "target pose " << target_x << " " << target_y << std::endl;
			//check if new target is not too close to the previous one
			if ( (target_x-previous_target_x)*(target_x-previous_target_x) + (target_y-previous_target_y)*(target_y-previous_target_y) > dis_thres)
			{

				//std::cout << "target pose " << target_x << " " << target_y << std::endl;
				//send the new target
				move_base_msgs::MoveBaseGoal goal;
				goal.target_pose.pose.position.x = target_x;
				goal.target_pose.pose.position.y = target_y;
				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(target_y-current_y, target_x-current_x));
				ac.sendGoal(goal, 
					    boost::bind(&cPersonFollowing::goalDoneCallback, this, _1, _2), actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(), actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleFeedbackCallback());
				//update current target
				previous_target_x = target_x;
				previous_target_y = target_y; 
				
				new_pose = true;
				

			}
			else //no new pose (new target is too close to previous one)
			{
				if (new_pose || starting) //there was a new pose in the previous iteration or the follow-me behavior is just starting
				{
					begin = clock();
					new_pose = false;

				}
				
				else // there was no new pose in the previous iteration
				{
					//check the time
					clock_t end = clock();
		  			double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
					if (elapsed_secs >= time_limit_secs) //the timeout has been reached
					{
						//the user has not moved for a while, so the following behaviour is stopped
						std::cout << "no new poses for a while, stopping " << std::endl; // no new positions will be sent
						stopFollowing();
					}

				}

			}

			//check if the average velocity is low
			/*v_sum += current_target.vX*current_target.vX + current_target.vY*current_target.vY;
			count_num++;
			if (count_num == it_limit)
			{
				if (v_sum < v_thres*v_thres*it_limit ) //the average speed in the last iterations has been low
				{	
					std::cout << "speed is low, stopping " << std::endl; // no new positions will be sent
					stopFollowing();
				}

				count_num = 0;
				v_sum = 0;
			}*/

			if (starting) starting = false;
			
		}

		//std::cout << "status " << goal_status.data << std::endl;

		if(as_->isPreemptRequested())
      		{
			std::cout << "preempt requested" << std::endl;
			stopFollowing();
			//stop the robot, cancel current goal!!!
			std_srvs::Empty srv;
        		if (!cancel_goal_client.call(srv))
        			ROS_DEBUG("Failed to call service cancel goal"); //This would be a problem
			as_->setPreempted();
			return;

		}

		if (goal_status.data == "reached")
		{
			as_->setSucceeded(hobbit_msgs::FollowMeResult(), "Last detected position reached");
			std::cout << "reached, succeeded " << std::endl; 
			return;
		}

		if (goal_status.data == "aborted")
		{
			as_->setAborted(hobbit_msgs::FollowMeResult(), "Following aborted, probably because the path is blocked");
			std::cout << "aborted, blocked " << std::endl; 
			return;
		}

		if (goal_status.data == "preempted" || goal_status.data == "recalled")
		{
			as_->setPreempted();
			std::cout << "preempted " << std::endl; 
			return;
		}

		//FIXME, check all possible status	

	}

	//if the node is killed then we'll abort and return
        as_->setAborted(hobbit_msgs::FollowMeResult(), "Aborting on the goal because the node has been killed");
        return;

}




