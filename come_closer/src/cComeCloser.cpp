#include "ros/ros.h"
#include "../include/ComeCloser/cComeCloser.h"

#include "occupancy_grid_utils/ray_tracer.h"

#include "angles/angles.h"

#include <string>

#include <mira_msgs/ResetMotorStop.h>
#include <mira_msgs/EmergencyStop.h>

#include <mira_msgs/GetBoolValue.h>

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor. Initialises member attributes and allocates resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cComeCloser::cComeCloser(int argc, char **argv) : init_argc(argc), init_argv(argv)
{
	as_ = new actionlib::SimpleActionServer<hobbit_msgs::GeneralHobbitAction>(ros::NodeHandle(), "come_closer", boost::bind(&cComeCloser::executeCb, this, _1), false);

	ros::NodeHandle nh("~");
	nh.param("margin", margin, 0.05);
	nh.param("front_dis", front_dis, 0.18);
	nh.param("x_sensor", x_sensor, 0.135);
	nh.param("time_limit_secs", time_limit_secs, 60.0);

	nh.param("ang_margin", ang_margin, 22.0); //minimum value to consider a minimum width of 40 cm from a maximum distance of 1.0m

	nh.param("x_offset", x_offset, -0.24);

	nh.param("dis_thres", dis_thres, 0.1);
	nh.param("ang_thres", ang_thres, 15.0);

	nh.param("range_max", range_max, 2.0);

	started_rotation = false;
	finished_rotation = false;
	started_movement = false;
	finished_movement = false;
	movement_cmd_sent = false;

	ros::NodeHandle n;
	discrete_motion_cmd_pub = n.advertise<std_msgs::String>("/DiscreteMotionCmd", 20);

	get_local_map_client = n.serviceClient<hobbit_msgs::GetOccupancyGrid>("/get_local_map");

	current_loc_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 2, &cComeCloser::loc_pose_callback, this);

	laserPublisher = n.advertise<sensor_msgs::LaserScan>("loc_scan", 1); //only for debugging!! //FIXME!!!!!

	motion_state_sub = n.subscribe<std_msgs::String>("/DiscreteMotionState", 2, &cComeCloser::motion_state_callback, this);

        as_->start();

	current_motion_state.data = "Idle";

	event_sub = n.subscribe("/Event", 2, &cComeCloser::event_callback, this);
	emergency_stop_client = n.serviceClient<mira_msgs::EmergencyStop>("/emergency_stop");
  	reset_motorstop_client = n.serviceClient<mira_msgs::ResetMotorStop>("/reset_motorstop");

	stop_sub = n.subscribe("/stop_request", 2, &cComeCloser::stop_callback, this);

	get_nav_mode_client = n.serviceClient<mira_msgs::GetBoolValue>("/get_nav_mode");

}

cComeCloser::~cComeCloser()
{
  printf("cComeCloser::~cComeCloser(): shutting down ROS\n");
  usleep(100000);
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  usleep(100000);
  printf(" - done\n");

  if(as_ != NULL)
  	delete as_;
}

void cComeCloser::loc_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  currentPose = (*msg);  
}

void cComeCloser::motion_state_callback(const std_msgs::String::ConstPtr& msg)
{
  current_motion_state = (*msg);  

  if (!started_rotation && current_motion_state.data == "Turning") //based on odometry
  	started_rotation = true;
  if (started_rotation && !finished_rotation && current_motion_state.data == "Idle")
	finished_rotation = true;
  if (finished_rotation && current_motion_state.data == "Moving")
	started_movement = true;
  if (started_movement && !finished_movement && current_motion_state.data == "Idle")
  {
	bool dist_mode = true;
	mira_msgs::GetBoolValue get_nav_mode_srv;
  	if (get_nav_mode_client.call(get_nav_mode_srv))
	{	
		dist_mode = get_nav_mode_srv.response.value;
	}
	else
		ROS_INFO("Failed to call service get_nav_mode");
		
	if (!dist_mode) //ends later but is safer to check if the robot really stopped
	{
		finished_movement = true;
		std::cout << "finished movement " << std::endl;
	}
  }

}



void cComeCloser::executeCb(const hobbit_msgs::GeneralHobbitGoalConstPtr& goal)
{

	std::cout << "come_closer callback " << std::endl;

	if (goal->command.data != "start")
	{
		return;
	}

	started_rotation = false;
	finished_rotation = false;
	started_movement = false;
	finished_movement = false;
	movement_cmd_sent = false;

	hobbit_msgs::GetOccupancyGrid srv;
	
	if (get_local_map_client.call(srv))
        {
          //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	   local_grid = srv.response.occupancy_grid; //we should get the latest map, with the remembered obstacles, in the proper format
	   std::cout << "********************* " << std::endl;
	   std::cout << "local_grid received " << std::endl;
        }
        else
        {
           ROS_DEBUG("Failed to call service get_local_map");
        }

	float user_rel_x = atof (goal->parameters[0].data.c_str());//+x_offset; // parameters[0] must provide person_msg.z/1000
	float user_rel_y = atof (goal->parameters[1].data.c_str()); // parameters[0] must provide -person_msg.x/1000

	std::cout << "user_relative_position " << user_rel_x << " " << user_rel_y << std::endl;
	double orientation = atan2(user_rel_y, user_rel_x);


	///////////////////////////////////////////////////////////////////////////////////////
	//ray_tracing

	const nav_msgs::OccupancyGrid const_local_grid = local_grid;

	/*std::cout << "origin_x " << local_grid.info.origin.position.x << std::endl;
	std::cout << "origin_y " << local_grid.info.origin.position.y << std::endl;
	std::cout << "orientation " << tf::getYaw(local_grid.info.origin.orientation)*180/M_PI << std::endl; */

	geometry_msgs::Pose sensor_pose;
	double current_orientation = tf::getYaw(currentPose.pose.pose.orientation);
	sensor_pose.position.x = currentPose.pose.pose.position.x + x_sensor*cos(current_orientation);
	sensor_pose.position.y = currentPose.pose.pose.position.y + x_sensor*sin(current_orientation);
	sensor_pose.orientation = currentPose.pose.pose.orientation;

	/*std::cout << "sensor_x " << sensor_pose.position.x << std::endl;
	std::cout << "sensor_y " << sensor_pose.position.y << std::endl;
	std::cout << "sensor_orientation " << tf::getYaw(sensor_pose.orientation)*180/M_PI << std::endl;*/
	

	sensor_msgs::LaserScan scanner_info;
	scanner_info.angle_min = -M_PI/2;
	scanner_info.angle_max = M_PI/2;
	scanner_info.angle_increment = 1*M_PI/180; //FIXME add param
	scanner_info.range_max = range_max;

	const geometry_msgs::Pose sensor_pose_ = sensor_pose;
	const sensor_msgs::LaserScan scanner_info_ = scanner_info;
	sensor_msgs::LaserScanPtr scan = occupancy_grid_utils::simulateRangeScan(const_local_grid, sensor_pose_, scanner_info_, false);

	double init_ang = scan->angle_min;
	double min_dis = scanner_info.range_max; 

	if (user_rel_x < 0)
	{
		std::cout << "user pose out of field of view, aborting " << std::endl;
		ROS_INFO ("user pose out of field of view, aborting ");
		as_->setAborted(hobbit_msgs::GeneralHobbitResult(), "aborted, user pose out of fov");
		return;

	}

	int init_index = ((angles::shortest_angular_distance(init_ang, orientation)-ang_margin*M_PI/180)/scan->angle_increment) +1;
	int end_index = ((angles::shortest_angular_distance(init_ang, orientation)+ang_margin*M_PI/180)/scan->angle_increment) +1;

	/*std::cout << "ang init " << (angles::shortest_angular_distance(ang, orientation)-ang_margin*M_PI/180) * 180/M_PI << std::endl;
	std::cout << "ang end " << (angles::shortest_angular_distance(ang, orientation)+ang_margin*M_PI/180) * 180/M_PI << std::endl;

	std::cout << "init_ind " << init_index << std::endl;
	std::cout << "end_ind " << end_index << std::endl;*/

	for (int i=init_index; i<=end_index && i<scan->ranges.size(); i++)
	{
		//project point onto user direction
		double ang = scan->angle_min + (i-1)*scan->angle_increment;

		double projected_dis = fabs(scan->ranges[i]*cos(angles::shortest_angular_distance(orientation, ang)));
		//double projected_dis = scan->ranges[i];
		if (projected_dis < min_dis)
		{
			min_dis = projected_dis;
		}


	}

	std::cout << "min_dis " << min_dis << std::endl;

	if (!(min_dis < scanner_info.range_max))
	{
		as_->setAborted(hobbit_msgs::GeneralHobbitResult(), "aborted, no detection"); //FIXME, should it move anyway?
		std::cout << "aborted, no detection" << std::endl; 
		ROS_INFO ("aborted, no detection ");
		return;
	}


	double dis2move = min_dis + x_sensor-front_dis-margin;
	
	double sensor_orientation = tf::getYaw(sensor_pose.orientation);  //should be relative
	double angle2turn = angles::shortest_angular_distance(0, orientation);

	if (dis2move >= dis_thres) 
	{
				
		std::cout << "angle2turn " << angle2turn * 180/M_PI << std::endl;
		std::cout << "distance2move " << dis2move << std::endl;

		ROS_INFO ("angle2turn %f ", angle2turn * 180/M_PI);
		ROS_INFO ("distance2move %f ", dis2move);

		//first rotate to face the detected user (should already be quite close to current orientation...)

		if (fabs(angle2turn) < ang_thres*M_PI/180) 
		{
			std::cout << "rotation is too small " << std::endl;
			//finished_rotation = true;
			if (angle2turn > 0.5*ang_thres*M_PI/180)
			{
				angle2turn = ang_thres*M_PI/180;
			}
			else if (angle2turn < -0.5*ang_thres*M_PI/180 )
			{
				angle2turn = -ang_thres*M_PI/180;
			}
			else
			{
				finished_rotation=true;
				std::cout << "no rotation " << std::endl;
				ROS_INFO ("no rotation ");
			}
		
		}
		
		
		std_msgs::String rotate_cmd;
		std::ostringstream s;
		s << "Turn " << angle2turn*180/M_PI;
		rotate_cmd.data = s.str();

		discrete_motion_cmd_pub.publish(rotate_cmd);

		std::cout << "rotation command sent " << std::endl;
		

	}
	
	else
	{
		as_->setAborted(hobbit_msgs::GeneralHobbitResult(), "aborted, too close");
		std::cout << "aborted, too close to obstacle" << std::endl;
		ROS_INFO ("aborted, too close to obstacle"); 
		return;

	}

	ros::Time timeout = ros::Time::now() + ros::Duration(time_limit_secs);


	///////////////////////////////////////////////////////////////////////////////////////
	ros::NodeHandle n;
	while (n.ok())
	{

		//laserPublisher.publish(scan);  //FIXME, to be removed, only for visualization purposes

		if(as_->isPreemptRequested())
      		{
			std::cout << "preempt requested" << std::endl;
			ROS_INFO ("preempt requested ");
			
			stop();

			as_->setPreempted();
			return;

		}

		if (finished_rotation && !started_movement && !movement_cmd_sent)
		{
			//sleep(2);
			std_msgs::String move_cmd;
			std::ostringstream s;
			s << "Move " << dis2move;
			move_cmd.data = s.str();
			discrete_motion_cmd_pub.publish(move_cmd);
			movement_cmd_sent = true;
			std::cout << "finished rotation, sending move command" << std::endl;

		}

		if (finished_movement)
		{
			
			as_->setSucceeded(hobbit_msgs::GeneralHobbitResult(), "succeded");
			std::cout << "discrete motion finished, succeeded " << std::endl; 
			ROS_INFO ("discrete motion finished, succeeded ");
			return;

		}
		

		//check the time
		ros::Duration time_left = timeout - ros::Time::now();
		if (time_left <= ros::Duration(0,0) ) //the timeout has been reached
		{
			
			as_->setAborted(hobbit_msgs::GeneralHobbitResult(), "aborted, it's taking too long, something went wrong?");
			std::cout << "aborted, it's taking too long, something went wrong?" << std::endl;
			ROS_INFO ("aborted, it's taking too long, something went wrong? "); 
			return;

		}


	}

	//if the node is killed then we'll abort and return
			
        as_->setAborted(hobbit_msgs::GeneralHobbitResult(), "Aborting on the goal because the node has been killed");
        return;

}

void cComeCloser::stop() //It would be better to have this in a different node, but for now let's keep it here
{
	//std::cout << "stop command received " << std::endl;

	//check if movement
	if(current_motion_state.data == "Turning" || current_motion_state.data == "Moving")
	{

		mira_msgs::EmergencyStop srv;
		if (!emergency_stop_client.call(srv))
		{
		  ROS_INFO("Failed to call service stop_request");
		}

		sleep(3);

		mira_msgs::ResetMotorStop srv2;
		if (!reset_motorstop_client.call(srv2))
		{
		  ROS_INFO("Failed to call service reset_motorstop");
		}
	}


}

void cComeCloser::stop_callback(const std_msgs::String::ConstPtr& msg)
{
	if (msg->data.compare("stop")==0 || msg->data.compare("Stop")==0 || msg->data.compare("STOP")==0)
	{
		stop();
	}

}

void cComeCloser::event_callback(const hobbit_msgs::Event::ConstPtr& msg)
{
	if (msg->event.compare("E_RELEASEBUTTON")==0)
	{
		//std::cout << "Release button pressed" << std::endl;
		//call service 
		mira_msgs::ResetMotorStop srv;
	        if (!reset_motorstop_client.call(srv))
	        {
	          ROS_DEBUG("Failed to call service reset_motorstop");
	        }
		
	}

}


