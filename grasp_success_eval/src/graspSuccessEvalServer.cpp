/*
 * David Fischinger: modified to use code of Ester Martinez as a service and reduce need for high CPU resources
 * 29.1.2015
 *
 */


#include <ros/ros.h>

//Include for publishing and subscribing to images
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/String.h>
#include <sstream>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/ros/conversions.h>

#include <pcl_ros/point_cloud.h>

// OpenCV library
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Point Cloud library
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>

// C includes
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <limits>

#include "checkGrasping.h"

using namespace cv;
using namespace std;

// Set how many images are required for camera calibration
#define thresholdCalibration 0


//int nimg = 0;

int waitCalibration = 0;
int objeto = -2; // Variable representing the evaluation result
                 //     -2: evaluation process deactivated
                 //     -1: the gripper is open
                 //      0: no object grasped
                 //      1: object grasped



bool check_grasp_get_result(hobbit_msgs::GraspSuccessCheck::Request  &req,
							hobbit_msgs::GraspSuccessCheck::Response &res)
{
  res.result = procesaDepthImg(req.input_pc);
  ROS_INFO("sending back response: [%d]", (int)res.result);
  return true;
}




// Evaluate the grasping task
bool procesaDepthImg(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) {

	pcl::PointCloud<pcl::PointXYZRGB> cloud (*msg);

	// ****************** SAVING IMAGES *********************************
	/*
  	  char nombre [100];
  	  nimg++;
  	  sprintf(nombre, "pointCloud_%d.pcd", nimg);
  	  pcl::io::savePCDFileASCII(nombre, cloud);
	 */
	// ***********************************************************************


	// Call to the function evaluating the grasping task. Its parameters are:
    //   * minDepth - maxDepth: defines the depth range in which the object could be located
    //   * minCol, maxCol, minRow, maxRow: defines the portion of the image to be analysed

    // int checkGraspTask (pcl::PointCloud<pcl::PointXYZRGB> cloud, double minDepth=0.9, double maxDepth=1.03, int minCol=220, int maxCol=370, int minRow=260, int maxRow=380);

	objeto = -1;
    objeto = checkGraspTask (cloud);
    printf("checkGraspTask return value: [%d]\n", (int)objeto);
    
    if (objeto == 1)
    {
    	return true
    } else {
    	return false
    }


}

int main (int argc, char** argv) {
	// ROS Initialization
	ros::init(argc, argv, "grasp_success_check_server");
	// Node Handle Definition
	ros::NodeHandle n;

	ros::ServiceServer grasp_check_service = n.advertiseService("grasp_success_check", check_grasp_get_result);
	ROS_INFO("Ready to check if a grasp from floor was successful.");
	ros::spin();

	return 0;


 // Subscribe to the publication of depth maps on the base topic “/headcam/depth_registered/points”
 //ros::Subscriber pcl_sub; = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/headcam/depth_registered/points", 1, &procesaDepthImg);
 // Define the publisher of the evaluation result
 //ros::Publisher result_pub = n.advertise<std_msgs::String>("checkGrasping", 1);



 //while (ros::ok()) {
  // Publishing the result of grasping task evaluation in the topic /checkGrasping
 /* std_msgs::String msg;



  std::stringstream ss;
  switch (objeto) {

   case -1: ss << "-1"; // process not started
            break;
   case 0: ss << "0"; // No object grasped
            break;
   case 1: ss << "1"; // Object grasped
            break;
  }

  msg.data = ss.str();
  result_pub.publish(msg);

  ros::spinOnce();
 }

 return 0;*/
}
