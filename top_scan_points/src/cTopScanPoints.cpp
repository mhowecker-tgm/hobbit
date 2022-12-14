
#include "ros/ros.h"
#include "../include/TopScanPoints/cTopScanPoints.h"
#include "pcl/conversions.h"
#include "pcl_ros/transforms.h"
#include <pcl_ros/point_cloud.h>

//#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf_conversions/tf_eigen.h>

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

//Modified by Paloma de la Puente

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
	output.range_max = 1.5;


	obstacle_trans.header.frame_id = "base_link";
    	obstacle_trans.child_frame_id = "obstacle_link";
    	obstacle_trans.transform.translation.x = 0;
    	obstacle_trans.transform.translation.y = 0;
   	obstacle_trans.transform.translation.z = 0;
    	obstacle_trans.transform.rotation.x = 0.0;
    	obstacle_trans.transform.rotation.y = 0.0;
    	obstacle_trans.transform.rotation.z = 0.0;
    	obstacle_trans.transform.rotation.w = 1.0;

	ros::NodeHandle nh("~");
	nh.param("robot_front", robot_front, 0.22);
	nh.param("robot_width", robot_width, 0.44);
	nh.param("min_height", min_height_, 0.15);
	nh.param("max_height", max_height_, 1.4);

	nh.param<std::string>("origin_link", origin_link, "base_link");
	nh.param<std::string>("target_link", target_link, "headcam_rgb_optical_frame");

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

void cTopScanPoints::open(ros::NodeHandle & n)
{
	cloudSubscriber = n.subscribe<sensor_msgs::PointCloud2>("/headcam/depth_registered/points", 1, &cTopScanPoints::callback, this);

        laserPublisher = n.advertise<sensor_msgs::LaserScan>("obstacle_scan", 1); 

}


////************************************************************************************
  //Pointcloud received
void cTopScanPoints::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	//std::cout << "callback " << std::endl;
	sensor_msgs::PointCloud2 point_cloud2;
	point_cloud2 = *msg;

	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

  	fromROSMsg(point_cloud2, pcl_cloud);

	pcl::PointCloud<pcl::PointXYZ> point_cloud_new_frame;
	/*pcl::PointCloud<pcl::PointXYZ> point_cloud_new_frame2;

	 Eigen::Vector3f trans_base_to_neck(-0.260, 0.000, 1.090);
    	 Eigen::Matrix4f base_to_neck;
    	 base_to_neck.setIdentity();
         base_to_neck.block<3,1>(0,3) = trans_base_to_neck; //brings point in basis CS to

    	 Eigen::AngleAxisf rollAngle(0.0 *M_PI/180, Eigen::Vector3f::UnitZ());
    	 Eigen::AngleAxisf yawAngle(34.0*M_PI/180, Eigen::Vector3f::UnitY());

    	 Eigen::Matrix4f rotation_learning;
    	 rotation_learning.setIdentity();
    	 rotation_learning.block<3,3>(0,0) = rollAngle.toRotationMatrix()*yawAngle.toRotationMatrix();


	 Eigen::Vector3f trans_neck_to_cam(0.012, -0.045, 0.166);
    	 Eigen::Quaternionf q_neck_to_cam (0.508, -0.486, 0.492, -0.514);

    	 Eigen::Matrix4f neck_to_cam;
    	 neck_to_cam.setIdentity();
    	 neck_to_cam.block<3,3>(0,0) = q_neck_to_cam.toRotationMatrix();
    	 neck_to_cam.block<3,1>(0,3) = trans_neck_to_cam;

    	 Eigen::Matrix4f matrix = base_to_neck * rotation_learning * neck_to_cam;*/

	 try
	 {

/*		//Apply transform
		Eigen::Matrix4d matrix_d(matrix.cast<double>());
		Eigen::Affine3d affine_trans(matrix_d);
		tf::Transform trans;
		tf::transformEigenToTF(affine_trans, trans);
		//std::cout << "trans " << trans.getOrigin()[0] << " " << trans.getOrigin()[1] << " " << trans.getOrigin()[2] << std::endl;
		float x = trans.getRotation().x();
		float y = trans.getRotation().y();
		float z = trans.getRotation().z();
		float w = trans.getRotation().w();
		//std::cout << "quaternion" << x << " " << y << " " << z << " " << w << std::endl;
		pcl_ros::transformPointCloud(pcl_cloud, point_cloud_new_frame, trans); */

	/*	// Define tf transformation
		tf::StampedTransform trans;
  		trans.setOrigin(tf::Vector3(-0.160, -0.046, 1.223));
		//trans.setRotation(tf::createQuaternionFromRPY(0.96, M_PI, M_PI/2));
		trans.setRotation(tf::Quaternion(-0.610, 0.616, -0.355, 0.351));
		//Apply transform to new reference frame */
		pcl_ros::transformPointCloud(pcl_cloud, point_cloud_new_frame, transform);
	

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
	int count = 0;
	int num_points_proc = 5; //FIXME

	//"Thin" laser height from pointcloud
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = point_cloud_new_frame.begin(); it != point_cloud_new_frame.end(); ++it)
	{
		count++;
		if (count < num_points_proc) continue; //use only one point from every n points
		count = 0;

		const float &x = it->x;
		const float &y = it->y;
		const float &z = it->z;

		/*float cam_disp_x = -0.248;
		float cam_disp_y = -0.208;*/


		if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) )
		{
			continue;
		}

		//points from robot base or tablet should be ignored
		//if ( fabs(x) < (robot_front - cam_disp_x) && y < (0.5*robot_width-cam_disp_y) && y > (-cam_disp_y - 0.5*robot_width))
		if ( fabs(x) < (robot_front) && y < (0.5*robot_width) && y > (- 0.5*robot_width))
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

		}
	} 

	output.header.stamp = ros::Time::now ();
	laserPublisher.publish(output);
	obstacle_trans.header.stamp = ros::Time::now ();
        (*p_obstacle_broadcaster).sendTransform(obstacle_trans);


} //callback

//run
void cTopScanPoints::Run(void)
{
	try 
	{
	    listener.waitForTransform(origin_link, target_link, ros::Time(0), ros::Duration(10.0));
	    listener.lookupTransform(origin_link, target_link, ros::Time(0), transform);
	} 
	catch (tf::TransformException ex) 
	{
    	     ROS_ERROR("%s",ex.what());
	}

}

