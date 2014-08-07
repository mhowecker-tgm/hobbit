#include "ros/ros.h"
#include "../include/TopScanPoints/cTopScanPoints.h"
#include "pcl/conversions.h"
#include "pcl_ros/transforms.h"
#include <pcl_ros/point_cloud.h>

//#include <pcl/visualization/cloud_viewer.h>

/* PARTIALLY based on SOURCE cloud_to_scanHoriz.cpp: 
* Copyright (c) 2010, Willow Garage, Inc.
* All rights reserved. 

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

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
	output.range_max = 1;


	obstacle_trans.header.frame_id = "base_link";
    	obstacle_trans.child_frame_id = "obstacle_link";
    	obstacle_trans.transform.translation.x = -0.248;
    	obstacle_trans.transform.translation.y = -0.208;
   	obstacle_trans.transform.translation.z = 1.108;
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
  //Pointcloud received
void cTopScanPoints::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	//std::cout << "callback " << std::endl;
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

	//fromROSMsg not working

	pcl::PointCloud<pcl::PointXYZ> point_cloud_new_frame;
	pcl::PointCloud<pcl::PointXYZ> point_cloud_new_frame2;

	try
	{

		// Define first tf transformation
		tf::StampedTransform headcam_trans;
		//Translation to the neck
  		headcam_trans.setOrigin(tf::Vector3(-0.12, -0.052, -0.086));
		//Apply transform
		pcl_ros::transformPointCloud(point_cloud, point_cloud_new_frame, headcam_trans_height);

		// Define rotations tf transformation
		tf::StampedTransform headcam_trans_rot;
		//RPY angles obtained from the matrix composition of the following transformations:
                // 1.) 90 degrees rotation around fixed global y axis
		// 2.) -90 degrees around fixed global x axis
		// 3.)-35 degrees around new x axis (head_inclination_angle)
		//roll value is obtained as asin(cos(head_inclination_angle))
		headcam_trans_rot.setRotation(tf::createQuaternionFromRPY(0.96, M_PI, M_PI/2));
		//Apply transform to new reference frame
		pcl_ros::transformPointCloud(point_cloud_new_frame, point_cloud_new_frame2, headcam_trans);

		// Define the tf transformation for the height
		tf::StampedTransform headcam_trans_height;
		//Vertical translation in order to check heights
  		headcam_trans_height.setOrigin(tf::Vector3(0, 0, 1.108)); //FIXME, check height value
		//Transform pointcloud to new reference frame
		pcl_ros::transformPointCloud(point_cloud_new_frame2, point_cloud_new_frame, headcam_trans_height);

	}
	catch (tf::TransformException& e)
	{
		std::cout << "CloudToScanHoriz failed" << std::endl;
		std::cout << e.what();
		return;
	}

	/*pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

	pcl::PointCloud<pcl::PointXYZ>::Ptr mycloudPtr (new pcl::PointCloud<pcl::PointXYZ> (point_cloud_new_frame)); 
        viewer.showCloud(mycloudPtr);
	sleep (50);*/

	uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
	output.ranges.assign(ranges_size, output.range_max);

        bool process = false;
	//"Thin" laser height from pointcloud
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = point_cloud_new_frame.begin(); it != point_cloud_new_frame.end(); ++it)
	{
		process = !process;
		if (!process) continue;
		const float &x = it->x;
		const float &y = it->y;
		const float &z = it->z;
		
		float robot_front = 0.22;
		float robot_width = 0.44;
		
		float cam_disp_x = -0.248;
		float cam_disp_y = -0.208;

		float min_height_ = 0.3;
        	float max_height_ = 1;

		if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
		{
			continue;  
		}

		//points from robot base or tablet should be ignored
		if ( fabs(x) < (robot_front - cam_disp_x) && y < (0.5*robot_width-cam_disp_y) && y > (-cam_disp_y - 0.5*robot_width))
		{
			continue;
		}

		//std::cout << "Point " << x << " " << y << " " << z << std::endl;
		
		if (z > max_height_ || z < min_height_)
		{
			continue;
		}
		
		double angle = atan2(y, x);
		
		if (angle < output.angle_min || angle > output.angle_max)
		{
			continue;
		}
		int index = (angle - output.angle_min) / output.angle_increment;

		double range_sq = y*y+x*x;
		//std::cout << "range sq " << range_sq << std::endl;

		if (output.ranges[index] * output.ranges[index] > range_sq)
		{
			output.ranges[index] = sqrt(range_sq);
			//std::cout << "index " << index << std::endl;
			//std::cout << "dis " << output.ranges[index] << std::endl;
			//std::cout << "Point " << x << " " << y << " " << z << std::endl;
			//std::cout << "angle " << angle * 180/M_PI << std::endl;
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

	 ros::init(init_argc, init_argv, "topScan");
         ros::NodeHandle n;

        cloudSubscriber = n.subscribe<sensor_msgs::PointCloud2>("/headcam/depth_registered/points", 1, &cTopScanPoints::callback, this);

        laserPublisher = n.advertise<sensor_msgs::LaserScan>("obstacle_scan", 1);

	 tf::TransformBroadcaster obstacle_broadcaster;
         p_obstacle_broadcaster = &obstacle_broadcaster;

	// ROS main loop.
         ros::spin();

}


