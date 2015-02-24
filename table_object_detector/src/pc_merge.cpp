/*
 *
 *
 * David Fischinger -TUW
 * 04.08.2014
 * earlier version: 26.03.2012
 *
 * input:
 *
 *   pointcloud from topcamera read from topic /SS/headcam/depth_registered/points
 *
 * output:
 *   pointcloud edited
 *
 *   output point cloud w.r.t. tf_frame /base_link (base of hobbit) on topic /SS/headcam/depth_registered/points_edited_in_rcs
 *
 *
 */

#include <pcl_ros/point_cloud.h>
#include <pc_merge_with_basket_rot.hpp>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"

#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

#include "table_object_detector/CheckFreeSpace.h"
#include "table_object_detector/CheckMeanValuesForDefinedSpace.h"
#include "table_object_detector/CheckCameraDistanceCenter.h"


CPCMerge::CPCMerge(ros::NodeHandle nh_)
{
  nh = nh_;
  pc_cam1_filled = false;
  //define publishers
  pc_for_basketdet_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/SS/headcam/depth_registered/points_edited_in_rcs",1); //rcs:robot coordinate sys.
  pc_from_check_free_space_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/SS/headcam/depth_registered/points_from_check_free_space",1); //frame_id not fixed
  position_highestpoint_pub = nh.advertise<std_msgs::String>("/SS/basket_position",1);
  //define service check_free_space
  service_check_free_space = nh.advertiseService("check_free_space", &CPCMerge::check_free_space, this);
  service_check_mean_values_for_defined_space = nh.advertiseService("check_mean_values_for_defined_space", &CPCMerge::check_mean_values_for_defined_space, this);
  service_check_camera_distance_center = nh.advertiseService("check_camera_distance_center", &CPCMerge::check_camera_distance_center, this);

  //df dddd new 11.2.2015 for debugging:
  //pc_cfs_old_cs = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/df_pc_in_for_cfs_pcmerge_old_cs",1); //pc in for cfs
  //pc_cfs_new_cs = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/df_pc_in_for_cfs_pcmerge_new_cs",1); //pc in for cfs


  //define subscriber
  pc_cam1_sub = nh.subscribe("/SS/headcam/depth_registered/points",1, &CPCMerge::pc_cam1_callback, this);
}


bool CPCMerge::check_free_space(table_object_detector::CheckFreeSpace::Request  &req,
         table_object_detector::CheckFreeSpace::Response &res)
{
  m.lock();
  ROS_INFO("pc_merge.cpp: check_free_space: point cloud received");
  //search for tf transform for pc for the 2 given input and output tf-frames
  ros::Time now = req.cloud.header.stamp; //ros::Time::now();
  bool foundTransform = tf_listener.waitForTransform(req.frame_id_desired.data.c_str(), req.frame_id_original.data.c_str(), now, ros::Duration(5.0));
  if (!foundTransform)
  {
	ROS_WARN("check_free_space: No transform for point cloud found");
	m.unlock();
	return false;
  }

  //ROS_INFO(tf_listener);
  ROS_INFO("check_free_space: Transform for point cloud found");

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_check_free_space_old_cs;
  pcl::fromROSMsg(req.cloud, pcl_cloud_check_free_space_old_cs); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
  //df 11.2.2015
  //pc_cfs_old_cs.publish(pcl_cloud_check_free_space_old_cs);
  pc_check_free_space_new_cs = pcl_cloud_check_free_space_old_cs;

  pcl_ros::transformPointCloud(req.frame_id_desired.data.c_str(), pcl_cloud_check_free_space_old_cs, pc_check_free_space_new_cs, tf_listener);

  //df 11.2.2015
  //pc_cfs_new_cs.publish(pc_check_free_space_new_cs);

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_merged;  //needed zwischenstep?
  pcl_cloud_merged = pc_check_free_space_new_cs;


  filter_pc(pcl_cloud_merged, false, req.x1, req.x2, req.y1, req.y2, req.z1, req.z2); //second entry false <=> coordinates of highest points of scene are published instead of basket center point
  ROS_INFO("%d",(int)pcl_cloud_merged.points.size());

  //publish point clouds of area where free space was checked
  if (pcl_cloud_merged.points.size() > 0)
  {
  	pc_from_check_free_space_pub.publish(pcl_cloud_merged);
  }

  res.nr_points_in_area = pcl_cloud_merged.points.size();
  ROS_INFO("sending back response res.nr_points_in_area: [%ld]", (long int)res.nr_points_in_area);
  m.unlock();
  return true;
}


/* 23.2.2015
 * This service takes a point cloud, transforms the point cloud to the desired tf-frame, uses given limits to cut a block,
 * and calculates the average x,y,and z-values of this block. In addition it returns the number of points in this block
 *
 * input:
 * 		  point cloud
 * 		  original tf_frame
 *        disired_tf_frame
 *        limit min_x, max_x, min_y, max_y, min_z, max_z
 *
 * output:
 * 		  average height in mm of points in cloud (new tf-system) in box defined by limits
 */
bool CPCMerge::check_mean_values_for_defined_space/*free_space*/(table_object_detector::CheckMeanValuesForDefinedSpace::Request  &req,
		 table_object_detector::CheckMeanValuesForDefinedSpace::Response &res)
{
  m.lock();
  ROS_INFO("pc_merge.cpp: check_mean_values_for_defined_space: point cloud received");
  //search for tf transform for pc for the 2 given input and output tf-frames
  ros::Time now = req.cloud.header.stamp;
  bool foundTransform = tf_listener.waitForTransform(req.frame_id_desired.data.c_str(), req.frame_id_original.data.c_str(), now, ros::Duration(5.0));
  if (!foundTransform)
  {
	ROS_WARN("check_mean_values_for_defined_space: No transform for point cloud found");
	m.unlock();
	return false;
  }

  //ROS_INFO(tf_listener);
  ROS_INFO("check_mean_values_for_defined_space: Transform for point cloud found");

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_old_cs;
  pcl::fromROSMsg(req.cloud, pcl_cloud_old_cs); // !

  pc_new_cs = pcl_cloud_old_cs;

  pcl_ros::transformPointCloud(req.frame_id_desired.data.c_str(), pcl_cloud_old_cs, pc_new_cs, tf_listener);

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_merged;  //needed zwischenstep?
  pcl_cloud_merged = pc_new_cs;


  filter_pc(pcl_cloud_merged, false, req.x1, req.x2, req.y1, req.y2, req.z1, req.z2); //second entry false <=> coordinates of highest points of scene are published instead of basket center point
  ROS_INFO("%d",(int)pcl_cloud_merged.points.size());

  //publish point clouds of area where free space was checked
  if (pcl_cloud_merged.points.size() > 0)
  {
	  pc_from_check_mean_values_for_defined_space_pub.publish(pcl_cloud_merged);
  }

  //go through points and calculate mean height
  float x_sum,y_sum,z_sum;
  x_sum=0;
  y_sum=0;
  z_sum=0;

  for (int i=0; i< pcl_cloud_merged.points.size(); i++){
	  x_sum += pcl_cloud_merged.points[i].x;
	  y_sum += pcl_cloud_merged.points[i].y;
	  z_sum += pcl_cloud_merged.points[i].z;
  }

  res.nr_points_in_area = pcl_cloud_merged.points.size();
  res.x_mean = x_sum/pcl_cloud_merged.points.size();
  res.y_mean = y_sum/pcl_cloud_merged.points.size();
  res.z_mean = z_sum/pcl_cloud_merged.points.size();

  ROS_INFO("sending back response res.nr_points_in_area: [%ld]", (long int)res.nr_points_in_area);
  ROS_INFO("sending back response res.x_mean: [%f]", (float)res.x_mean);
  ROS_INFO("sending back response res.y_mean: [%f]", (float)res.y_mean);
  ROS_INFO("sending back response res.z_mean: [%f]", (float)res.z_mean);
  m.unlock();
  return true;
}




// check how far away the center of the depth cloud is away from the camera (in m)
bool CPCMerge::check_camera_distance_center(table_object_detector::CheckCameraDistanceCenter::Request  &req,
         table_object_detector::CheckCameraDistanceCenter::Response &res)
{
  m.lock();
  ROS_INFO("pc_merge.cpp: check_camera_distance_center: point cloud received");


  //pcl::PointCloud<pcl::PointXYZ> pcl_cloud_check_free_space_old_cs;
  //pcl::fromROSMsg(req.cloud, pcl_cloud_check_free_space_old_cs); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
  //ROS_INFO("=======================> %d",req.cloud.points.size());
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_input_as_pcl;
  pcl::fromROSMsg(req.cloud, pcl_cloud_input_as_pcl); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
  int middle_point = pcl_cloud_input_as_pcl.points.size()/2;
  float acc_distance;
  acc_distance = 0.0;
  int cnt;
  cnt = 0;
  for (int i=148000; i< 154000; i++){
	res.camera_center_object_distance_in_m = pcl_cloud_input_as_pcl.points[i].z;  //!!!!!!! probably very buggy for getting real center value
	if (res.camera_center_object_distance_in_m > -1.0 and res.camera_center_object_distance_in_m < 1000.0 and pcl_cloud_input_as_pcl.points[i].x < 0.2 and 
            pcl_cloud_input_as_pcl.points[i].x > -0.2 and pcl_cloud_input_as_pcl.points[i].y < 0.1 and pcl_cloud_input_as_pcl.points[i].y > -0.1){  //if it's a real value
		acc_distance += res.camera_center_object_distance_in_m;
		cnt++;
	}
	//ROS_INFO("numer of points in point cloud: [%f]", (float)res.camera_center_object_distance_in_m);
  	//ROS_INFO("sending back response res.camera_center_object_distance_in_m: [x: \t %f \t y: %f \t z: %f]", pcl_cloud_input_as_pcl.points[i].x,pcl_cloud_input_as_pcl.points[i].y,(float)res.camera_center_object_distance_in_m);
  }
  ROS_INFO("\n check_camera_distance_center: counter of valid distance values: \t %f]", (float)(cnt));
  ROS_INFO("check_camera_distance_center: distance in camera z-axis: [z-center-mean: \t %f]", (float)(acc_distance/cnt));
  res.camera_center_object_distance_in_m = (float)(acc_distance/cnt);
  //pcl_cloud_merged.points.size();
  //ROS_INFO("sending back response res.camera_center_object_distance_in_m: [%f]", res.camera_center_object_distance_in_m);
  m.unlock();
  return true;
}








void CPCMerge::pc_cam1_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_in)
{
  m.lock();

  ROS_INFO("pc_cam1 received");

  this->pc_cam1_filled = false;

  //search for tf transform for pc from cam1
  ros::Time now = ros::Time::now();
  bool foundTransform = tf_listener.waitForTransform("/base_link", "/headcam_rgb_optical_frame", now
		                                             /*(*pcl_in).header.stamp*/, ros::Duration(13.0));
  if (!foundTransform)
  {
	ROS_WARN("No pc_cam1 transform found");
	m.unlock();
	return;
  }

  //ROS_INFO(tf_listener);
  ROS_INFO("Transform pc_cam1: headcam_rgb_optical_frame to base_link found");
  pcl_ros::transformPointCloud("/base_link", *pcl_in, pc_cam1, tf_listener);
  this->pc_cam1_filled = true;

  //publishes point cloud
  publish_merged_pc();
  
  m.unlock();
}


void CPCMerge::publish_merged_pc()
{
	ROS_INFO("publish_merged_pc() started");
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud_merged;
	pcl_cloud_merged = pc_cam1;

	//Filter Data (with basket detection and basket point elimination)
	filter_pc(pcl_cloud_merged, false); //last entry false <=> coordinates of highest points of scene are published instead of basket center point

    	//publish manipulated and merged point cloud data (in base_link (not world!) coordinate system)
    	pc_for_basketdet_out = pcl_cloud_merged;
    	pc_for_basketdet_out.header.frame_id = "/base_link";
	pc_for_basketdet_pub.publish(pc_for_basketdet_out);

	ROS_INFO("point cloud of objects (rcs) and highest point position (of objects) were published");

        this->pc_cam1_filled = false;
}

// filter point cloud and cut off points outside a defined region
//if pub_basket_position == false than the position of the highest point is returned! otherwise the center of basket
void CPCMerge::filter_pc(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_merged, bool pub_basket_position, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max )
{
	// Create the filtering object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ>);
   	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    	*cloud = pcl_cloud_merged;
	pcl::PassThrough<pcl::PointXYZ> pass;

	ROS_INFO("Filtering outliers and cutting region");
	ROS_INFO("points before filtering: %d", (int)pcl_cloud_merged.points.size());
	//Filter w.r.t. axis z
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (z_min, z_max);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered_z);
	ROS_INFO("points after z filtering: %d", (int)cloud_filtered_z->points.size());
	//Filter w.r.t. axis y
	pass.setInputCloud(cloud_filtered_z);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (y_min, y_max);
	pass.filter (*cloud_filtered_y);
	ROS_INFO("points after y filtering: %d", (int)cloud_filtered_y->points.size());
	//Filter w.r.t. axis x
	pass.setInputCloud(cloud_filtered_y);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (x_min, x_max);
	pass.filter (*cloud_filtered_x);
	ROS_INFO("points after x filtering: %d", (int)cloud_filtered_x->points.size());
	//Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_filtered_x);
	sor.setMeanK(50);
	sor.setStddevMulThresh (1.0);
	sor.filter(pcl_cloud_merged);

	if (pub_basket_position == false){ // than the position of the highest point is published!
	    ROS_INFO("%d",(int)pcl_cloud_merged.points.size());
		float z_max = -1000;			//max heigt val for z
		float x_for_z_max = -1000;		//coordinates for x,y at position of maximal z
		float y_for_z_max = -1000;
	    for (unsigned int i = 0; i < pcl_cloud_merged.points.size(); ++i)
		{
		  if (pcl_cloud_merged.points[i].z > z_max)
		  {
			z_max = pcl_cloud_merged.points[i].z;
			x_for_z_max = pcl_cloud_merged.points[i].x;
			y_for_z_max = pcl_cloud_merged.points[i].y;
		  }
		}
	    //Publish coordinates of heighest points ("basket center" if there is no basket)
		std_msgs::String msgStrCorners;
		std::stringstream ss;
		ss << x_for_z_max << " "  << y_for_z_max << " " << 0;
		msgStrCorners.data = ss.str();
		position_highestpoint_pub.publish(msgStrCorners);
	} else {
		basket_detection<pcl::PointXYZ>(pcl_cloud_merged,pub_basket_position);
	}
}



int main (int argc, char** argv)
{
  ROS_INFO("ROS NODE pc_merge_1cam_nobox started (point_cloud_edit)");
  ros::init(argc, argv, "point_cloud_edit");
  ros::NodeHandle nh;
  CPCMerge * pc_merge = new CPCMerge(nh);

  ros::spin();
  return (0);
}
