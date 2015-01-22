#include "ros/ros.h"
#include "../include/ComeCloser/cComeCloser.h"

#include "occupancy_grid_utils/ray_tracer.h"
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

	ros::NodeHandle n;
	discrete_motion_cmd_pub = n.advertise<std_msgs::String>("/discrete_motion_cmd", 20);

	get_local_map_client = n.serviceClient<hobbit_msgs::GetOccupancyGrid>("/get_local_map");

	current_loc_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 2, &cComeCloser::loc_pose_callback, this);

	laserPublisher = n.advertise<sensor_msgs::LaserScan>("loc_scan", 1); //only for debugging!! //FIXME!!!!!

        as_->start();
	


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
        }
        else
        {
           ROS_DEBUG("Failed to call service get_local_map");
        }


	///////////////////////////////////////////////////////////////////////////////////////
	//ray_tracing

	const nav_msgs::OccupancyGrid const_local_grid = local_grid;

	std::cout << "origin_x " << local_grid.info.origin.position.x << std::endl;
	std::cout << "origin_y " << local_grid.info.origin.position.y << std::endl;
	std::cout << "orientation " << tf::getYaw(local_grid.info.origin.orientation)*180/M_PI << std::endl;

	geometry_msgs::Pose sensor_pose;
	double current_orientation = tf::getYaw(currentPose.pose.pose.orientation);
	sensor_pose.position.x = currentPose.pose.pose.position.x + x_sensor*cos(current_orientation);
	sensor_pose.position.y = currentPose.pose.pose.position.y + x_sensor*sin(current_orientation);
	sensor_pose.orientation = currentPose.pose.pose.orientation;

	std::cout << "sensor_x " << sensor_pose.position.x << std::endl;
	std::cout << "sensor_y " << sensor_pose.position.y << std::endl;
	std::cout << "sensor_orientation " << tf::getYaw(sensor_pose.orientation)*180/M_PI << std::endl;
	

	sensor_msgs::LaserScan scanner_info;
	scanner_info.angle_min = -M_PI/2;
	scanner_info.angle_max = M_PI/2;
	scanner_info.angle_increment = 5*M_PI/360; //FIXME add params
	scanner_info.range_max = 1;

	const geometry_msgs::Pose sensor_pose_ = sensor_pose;
	const sensor_msgs::LaserScan scanner_info_ = scanner_info;
	sensor_msgs::LaserScanPtr scan = occupancy_grid_utils::simulateRangeScan(const_local_grid, sensor_pose_, scanner_info_, false);

	//TODO add publisher to visualize if the published virtual scan looks fine

	

	///////////////////////////////////////////////////////////////////////////////////////
	ros::NodeHandle n;
	while (n.ok())
	{

		laserPublisher.publish(scan);



		//std::cout << "status " << goal_status.data << std::endl;

		/*if(as_->isPreemptRequested())
      		{
			std::cout << "preempt requested" << std::endl;
			as_->setPreempted();

		}

		if (true) //FIXME
		{
			as_->setSucceeded(hobbit_msgs::GeneralHobbitResult(), "succeded");
			std::cout << "reached, succeeded " << std::endl; 
			return;
		}

		if (false) //FIXME
		{
			as_->setAborted(hobbit_msgs::GeneralHobbitResult(), "aborted");
			std::cout << "aborted" << std::endl; 
			return;
		}*/



		//FIXME, check all possible status	

	}

	//if the node is killed then we'll abort and return
        as_->setAborted(hobbit_msgs::GeneralHobbitResult(), "Aborting on the goal because the node has been killed");
        return;

}




