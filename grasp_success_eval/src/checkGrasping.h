#include <ros/ros.h>

//Include for publishing and subscribing to images
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/String.h>

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

using namespace cv;
using namespace std;

// Function for evaluating the grasping task performance. Its parameters are:
//   * minDepth - maxDepth: defines the depth range in which the object could be located
//   * minCol, maxCol, minRow, maxRow: defines the portion of the image to be analysed

int checkGraspTask (pcl::PointCloud<pcl::PointXYZRGB> cloud, double minDepth=0.9, double maxDepth=1.03, int minCol=220, int maxCol=370, int minRow=260, int maxRow=380);