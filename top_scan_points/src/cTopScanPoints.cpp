#include "ros/ros.h"
#include "../include/TopScanPoints/cTopScanPoints.h"
#include "pcl/conversions.h"
#include "pcl_ros/transforms.h"
#include <pcl_ros/point_cloud.h>


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor. Initialises member attributes and allocates resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cTopScanPoints::cTopScanPoints(int argc, char **argv) : init_argc(argc), init_argv(argv)
{

// Set up the localisation virtual laser scan message.
    //Setup laserscan message
	//output->header = cloud_in->header;
        
	output.header.frame_id = "/obstacle_link";
	output.angle_min = -M_PI/2;
	output.angle_max = M_PI/2;
	output.angle_increment = M_PI/180.0;
	output.time_increment = 0.0;
	output.scan_time = 1.0/30.0;
	output.range_min = 0.1;
	output.range_max = 2;


	obstacle_trans.header.frame_id = "base_link";
    	obstacle_trans.child_frame_id = "obstacle_link";
    	obstacle_trans.transform.translation.x = -0.05;
    	obstacle_trans.transform.translation.y = 0.019;
   	obstacle_trans.transform.translation.z = 0.5;
    	obstacle_trans.transform.rotation.x = 0.0;
    	obstacle_trans.transform.rotation.y = 0.0;
    	obstacle_trans.transform.rotation.z = 0.0;
    	obstacle_trans.transform.rotation.w = 1.0;
  
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Destructor. Shuts down ROS, ends the thread and released allocated
// resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cTopScanPoints::~cTopScanPoints()
{
  printf("cTopScanPoints::~cTopScanPoints(): shutting down ROS\n");
  usleep(100000);
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  usleep(100000);
  printf(" - done\n");
}


////************************************************************************************
  //Pointcloud and tf transform received
void cTopScanPoints::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	sensor_msgs::PointCloud2 point_cloud2;
	point_cloud2.data = msg-> data;

	float *pCloud;
	pCloud = (float *)(&(msg->data[0]));

	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

	int i = 0;
	for (int x = 0; x < DEPTH_DATA_WIDTH; x++)
  	{
		for (int y = 0; y < DEPTH_DATA_HEIGHT; y++)
		{
			pcl::PointXYZ point(pCloud[i], pCloud[i+1], pCloud[i+2]);
			//pcl::PointXYZ point(0,0,1);
			pcl_cloud.push_back(point);
			i+=4;
		
		}
	}


	pcl::PointCloud<pcl::PointXYZ> point_cloud_new_frame;


	try
	{

		tf::StampedTransform headcam_trans;
  		headcam_trans.setOrigin(tf::Vector3(0, 0, 0));
  		//headcam_trans.setRotation(tf::createQuaternionFromRPY(0,140*M_PI/180,-90*M_PI/180));
		headcam_trans.setRotation(tf::createQuaternionFromRPY(-0.005, 0.682, -0.014));

		//Transform pointcloud to new reference frame
		pcl_ros::transformPointCloud(pcl_cloud, point_cloud_new_frame, headcam_trans);
		
		//pcl_ros::transformPointCloud(pcl_cloud, point_cloud_new_frame, transform);

	}
	catch (tf::TransformException& e)
	{
		std::cout << "CloudToScanHoriz failed" << std::endl;
		std::cout << e.what();
		return;
	}


	uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
	output.ranges.assign(ranges_size, output.range_max + 1.0);

	//"Thin" laser height from pointcloud
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = point_cloud_new_frame.begin(); it != point_cloud_new_frame.end(); ++it)
	{
		const float &x = it->x;
		const float &y = it->y;
		const float &z = it->z;
		
		float robot_front = 0.22;
		float robot_width = 0.44;

		//points from robot base or tablet should be ignored
		if ( fabs(x) < robot_front && fabs(y) < 0.5*robot_width)
			continue;

		if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
		{
			continue;  
		}
		
		if (z > max_height_ || z < min_height_)
		{
			continue;
		}

		std::cout << "Point " << x << " " << y << " " << z << std::endl;
		
		double angle = atan2(y, x);
		//std::cout << "angle " << angle * 180/M_PI << std::endl;
		if (angle < output.angle_min || angle > output.angle_max)
		{
			continue;
		}
		int index = (angle - output.angle_min) / output.angle_increment;
		//Calculate hypoteneuse distance to point
		double range_sq = y*y+x*x;
		//std::cout << "range sq " << range_sq << std::endl;

		if (output.ranges[index] * output.ranges[index] > range_sq)
		{
			output.ranges[index] = sqrt(range_sq);
			//std::cout << "index " << index << std::endl;
			//std::cout << "dis " << output.ranges[index] << std::endl;
		}
	} //for it

	output.header.stamp = ros::Time::now ();
	laserPublisher.publish(output);

	obstacle_trans.header.stamp = ros::Time::now ();
        (*p_obstacle_broadcaster).sendTransform(obstacle_trans);


} //callback

//run
void cTopScanPoints::Run(void)
{

	min_height_ = -2;
        max_height_ = 1;

	 ros::init(init_argc, init_argv, "topScan");
         ros::NodeHandle n;

        cloudSubscriber = n.subscribe<sensor_msgs::PointCloud2>("/headcam/depth_registered/points", 1, &cTopScanPoints::callback, this);

        laserPublisher = n.advertise<sensor_msgs::LaserScan>("obstacle_scan", 1);

	 tf::TransformBroadcaster obstacle_broadcaster;
         p_obstacle_broadcaster = &obstacle_broadcaster;

	// ROS main loop.
         ros::spin();

}


