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

bool gripperOpen = true; // Variable containing the gripper status (open by default)
bool check = false; // Variable establishing the activation/deactivation of the evaluation process (false by default)

int nimg = 0;

int waitCalibration = 0;
int objeto = -2; // Variable representing the evaluation result
                 //     -2: evaluation process deactivated
                 //     -1: the gripper is open
                 //      0: no object grasped
                 //      1: object grasped

// Check the status of the gripper (open/close), gathering that information from the topic /gripperInfo
void gripperInfoCallback (const std_msgs::String::ConstPtr& msg) {
 string str = msg->data.c_str();
 size_t found = str.find("True");

 if (found != string::npos) {
  gripperOpen = false;
 }
 else {
  gripperOpen = true;
 }
}

// Check the grasping evaluation is required or not, gathering that information from the topic /checkGripper
void activationCheckCallback (const std_msgs::String::ConstPtr& msg) {
 string str = msg->data.c_str();
 size_t found = str.find("True");

 if (found != string::npos) {
  check = true;
 }
 else {
  check = false;
 }
}

// Evaluate the grasping task
void procesaDepthImg(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) {
 pcl::PointCloud<pcl::PointXYZRGB> cloud (*msg);

// ****************** SAVING IMAGES *********************************
/*
  char nombre [100];
  nimg++;
  sprintf(nombre, "pointCloud_%d.pcd", nimg);
  pcl::io::savePCDFileASCII(nombre, cloud);
*/
// ***********************************************************************

 // Check if the evaluation process is required or not
 if (check) {

  // Check if the gripper is open or close
  if (gripperOpen) {
   objeto = -1;
   waitCalibration = 0;
  }
  else {
   waitCalibration++;
   // Wait for the camera calibration
   if (waitCalibration < thresholdCalibration) {
    //calibrating
   }
   else {
       // Call to the function evaluating the grasping task. Its parameters are:
       //   * minDepth - maxDepth: defines the depth range in which the object could be located
       //   * minCol, maxCol, minRow, maxRow: defines the portion of the image to be analysed

       // int checkGraspTask (pcl::PointCloud<pcl::PointXYZRGB> cloud, double minDepth=0.9, double maxDepth=1.03, int minCol=220, int maxCol=370, int minRow=260, int maxRow=380);

       objeto = checkGraspTask (cloud);
   }
  }
 }
 else {
  objeto = -2;
 }

 //printf("%d\n", objeto);
    
}

int main (int argc, char** argv) {
 // ROS Initialization
 ros::init(argc, argv, "gripperAnalysis");

 // Node Handle Definition
 ros::NodeHandle n;

 // Subscribe to the publication of gripper information on the base topic “/gripperInfo”
 ros::Subscriber sub = n.subscribe("gripperInfo", 1, gripperInfoCallback);

 // Subscribe to the publication of activation information on the base topic “/checkGripper”
 ros::Subscriber sub2 = n.subscribe("checkGripper", 1, activationCheckCallback);

 // Subscribe to the publication of depth maps on the base topic “/headcam/depth_registered/points”
 ros::Subscriber pcl_sub = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/headcam/depth_registered/points", 1, &procesaDepthImg);

 // Define the publisher of the evaluation result
 ros::Publisher resultPub = n.advertise<std_msgs::String>("checkGrasping", 1);

 while (ros::ok()) {
  // Publishing the result of grasping task evaluation in the topic /checkGrasping
  std_msgs::String msg;

  std::stringstream ss;
  switch (objeto) {
   case -2: ss << "-2"; // Process deactivated
            break;
   case -1: ss << "-1"; // Gripper open
            break;
   case 0: ss << "0"; // No object grasped
            break;
   case 1: ss << "1"; // Object grasped
            break;
  }

  msg.data = ss.str();
  resultPub.publish(msg);

  ros::spinOnce();
 }

 return 0;
}
