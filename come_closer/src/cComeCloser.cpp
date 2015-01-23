#include "ros/ros.h"
#include "../include/ComeCloser/cComeCloser.h"

#include "occupancy_grid_utils/ray_tracer.h"

#include "angles/angles.h"

#include <string>
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor. Initialises member attributes and allocates resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cComeCloser::cComeCloser(int argc, char **argv) : init_argc(argc), init_argv(argv)
{
	as_ = new actionlib::SimpleActionServer<hobbit_msgs::GeneralHobbitAction>(ros::NodeHandle(), "come_closer", boost::bind(&cComeCloser::executeCb, this, _1), false);

	ros::NodeHandle nh;
	nh.param("margin", margin, 0.05);
	nh.param("front_dis", front_dis, 0.18);
	nh.param("x_sensor", x_sensor, 0.135);
	nh.param("time_limit_secs", time_limit_secs, 60.0);

	ros::NodeHandle n;
	discrete_motion_cmd_pub = n.advertise<std_msgs::String>("/discrete_motion_cmd", 20);

	get_local_map_client = n.serviceClient<hobbit_msgs::GetOccupancyGrid>("/get_local_map");

	current_loc_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 2, &cComeCloser::loc_pose_callback, this);

	laserPublisher = n.advertise<sensor_msgs::LaserScan>("loc_scan", 1); //only for debugging!! //FIXME!!!!!

	discrete_motion_cmd_pub = n.advertise<std_msgs::String>("/discrete_motion_cmd", 20);

	motion_state_sub = n.subscribe<std_msgs::String>("/DiscreteMotionState", 2, &cComeCloser::motion_state_callback, this);

        as_->start();

	current_motion_state.data = "Idle";

	started_rotation = false;
	finished_rotation = false;
	started_movement = false;
	finished_movement = false;
	


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
}

void cComeCloser::loc_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  currentPose = (*msg);  
}

void cComeCloser::motion_state_callback(const std_msgs::String::ConstPtr& msg)
{
  current_motion_state = (*msg);  

  if (!started_rotation && current_motion_state.data == "Turning")
  	started_rotation = true;

  if (started_rotation && current_motion_state.data == "Idle")
	finished_rotation = true;
  if (finished_rotation && current_motion_state.data == "Moving")
	started_movement = true;
  if (started_movement && current_motion_state.data == "Idle")
	finished_moving = true;


}



void cComeCloser::executeCb(const hobbit_msgs::GeneralHobbitGoalConstPtr& goal)
{

	if (goal->command.data != "start")
	{
		return;
	}

	hobbit_msgs::GetOccupancyGrid srv;
	
	if (get_local_map_client.call(srv))
        {
          //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	   local_grid = srv.response.occupancy_grid; //we should get the latest map, with the remembered obstacles
	   std::cout << "local_grid received " << std::endl;
        }
        else
        {
           ROS_DEBUG("Failed to call service get_local_map");
        }

	float user_rel_x = atof (goal->parameters[0].data.c_str());
	float user_rel_y = atof (goal->parameters[1].data.c_str());
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
	scanner_info.angle_increment = 5*M_PI/360; //FIXME add params
	scanner_info.range_max = 1;

	const geometry_msgs::Pose sensor_pose_ = sensor_pose;
	const sensor_msgs::LaserScan scanner_info_ = scanner_info;
	sensor_msgs::LaserScanPtr scan = occupancy_grid_utils::simulateRangeScan(const_local_grid, sensor_pose_, scanner_info_, false);

	double ang = scan->angle_min;
	double min_dis = scanner_info.range_max; 

	for (int i=0; i<scan->ranges.size();i++)
	{
		
		//project point onto user direction

		double projected_dis = fabs(scan->ranges[i]*cos(angles::shortest_angular_distance(orientation, ang)));
		if (projected_dis < min_dis)
			min_dis = projected_dis;

		ang+=scan->angle_increment;


	}

	double dis2move = min_dis+x_sensor-front_dis-margin;
	
	double sensor_orientation = tf::getYaw(sensor_pose.orientation);
	double angle2turn = angles::shortest_angular_distance(sensor_orientation, orientation);

	if (dis2move > 0)
	{

		//first rotate to face detected user
		std_msgs::String rotate_cmd;
		std::ostringstream s;
		s << "Turn " << angle2turn*180/M_PI;
		rotate_cmd.data = s.str();
		discrete_motion_cmd_pub.publish(rotate_cmd);

	}
	
	else
	{
		as_->setAborted(hobbit_msgs::GeneralHobbitResult(), "aborted, user is already too close");
		std::cout << "aborted, too close" << std::endl; 
		return;

	}

	clock_t begin = clock();


	///////////////////////////////////////////////////////////////////////////////////////
	ros::NodeHandle n;
	while (n.ok())
	{

		//laserPublisher.publish(scan);  //FIXME, to be removed, only for visualization purposes

		if (finished_rotation)
		{
			std_msgs::String move_cmd;
			std::ostringstream s;
			s << "Move " << dis2move;
			move_cmd.data = s.str();
			discrete_motion_cmd_pub.publish(move_cmd);
			std::cout << "finished rotation, sending move command" << std::endl;

		}

		if (finished_movement)
		{
			//discrete motion finished
			as_->setSucceeded(hobbit_msgs::GeneralHobbitResult(), "succeded");
			std::cout << "discrete motion finished, succeeded " << std::endl; 
			return;

		}
		

		if(as_->isPreemptRequested())
      		{
			std::cout << "preempt requested" << std::endl;
			as_->setPreempted();
			return;

		}

		clock_t end = clock();
		double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		if (elapsed_secs_target >= time_limit_secs)
		{
			
			as_->setAborted(hobbit_msgs::GeneralHobbitResult(), "aborted, it's taking too long, something went wrong?");
			std::cout << "aborted, it's taking too long, something went wrong?" << std::endl; 
			return;

		}


		//FIXME, check all possible status	

	}

	//if the node is killed then we'll abort and return
        as_->setAborted(hobbit_msgs::GeneralHobbitResult(), "Aborting on the goal because the node has been killed");
        return;

}




