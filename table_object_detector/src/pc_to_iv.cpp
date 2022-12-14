/*
 *
 * David Fischinger -TUW
 * 04.08.2014 (modified from earlier version)
 * 
 * Description: 
 * Programm listens to input point cloud and generates a iv-mesh in folder /tmp/
 *
 * input:
 *
 *   pointcloud from topic /pickup/graspableobjectCCS
 *
 * output:
 *   pointcloud as iv file (saved at /tmp/<FILENAME>)
 *	 output point cloud w.r.t. tf_frame /base_link (=> base of hobbit)
 *
 *	 publish filename at topic: /pc_to_iv/generated_ivfilename
 *
 *

*/

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

//PCL includes
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <iv_io.h>

//========begin=====
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
//==========end====

//#include <opencv2/highgui.h>

//Ros includes  
#include <ros/ros.h>
#include "pcl_ros/publisher.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"

#define BUFSIZE 512

using namespace pcl;
using namespace pcl::io;
using namespace std;



class CPCTOIV
{
protected:
	ros::NodeHandle nh;

public:
	int meshcnt; //mesh count
	ros::Publisher iv_filename_pub;
	tf::TransformListener tf_listener;
	ros::Subscriber sub;

	void generateInventor(const sensor_msgs::PointCloud2::ConstPtr& msg);

	CPCTOIV(ros::NodeHandle nh_)
	{
		nh = nh_;
		iv_filename_pub = nh.advertise<std_msgs::String>("/pc_to_iv/generated_ivfilename",10);
		sub = nh.subscribe("/pickup/graspableobjectCCS", 1, &CPCTOIV::generateInventor, this);
	}
};


void CPCTOIV::generateInventor(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

  ROS_INFO("Generating iv-file");
  ROS_INFO(" - Read point cloud");
  sensor_msgs::PointCloud2 pc2_rcs;
  PointCloud<PointXYZ>::Ptr pc_rcs (new PointCloud<PointXYZ>);
  //fromROSMsg(*msg,*pc_in);


  //search for tf transform for pc_in from cam1
  ros::Time now = ros::Time::now();
  bool foundTransform = tf_listener.waitForTransform("/base_link", "/headcam_rgb_optical_frame", now
  		                                             /*(*pcl_in).header.stamp*/, ros::Duration(13.0));
  if (!foundTransform)
  {
  	ROS_WARN("pc_to_iv.cpp: No transform found");
  	//m.unlock();
  	return;
  }
  ROS_INFO("Transform pc_cam1: headcam_rgb_optical_frame to base_link found");

  //bool 	transformPointCloud (const std::string &target_frame, const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out, const tf::TransformListener &tf_listener)
  pcl_ros::transformPointCloud("/base_link", *msg, pc2_rcs, tf_listener);
  // convert pc_rcs (sensormsgs::pointcloud2 already in rcs) to pcd pointcloud
  fromROSMsg(pc2_rcs,*pc_rcs);

//===============7.1.2015 ende======

  ROS_INFO(" - Convert point cloud");
  

  // Normal estimation
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
  search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
  tree->setInputCloud (pc_rcs);
  n.setInputCloud (pc_rcs);
  n.setSearchMethod (tree);
  n.setKSearch (30);
  n.compute (*normals);
  //cout << *normals;

  // Concatenate the XYZ and normal fields
  PointCloud<PointNormal>::Ptr pc_with_normals (new PointCloud<PointNormal>);
  pcl::concatenateFields (*pc_rcs, *normals, *pc_with_normals);

  // Create search tree
  search::KdTree<PointNormal>::Ptr tree2 (new search::KdTree<PointNormal>);
  tree2->setInputCloud (pc_with_normals);

  // Initialize objects
  GreedyProjectionTriangulation<PointNormal> gp3;
  PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.065); 	//default 0.025

  // Set typical values for the parameters
  gp3.setMu (3.5); 		//default 2.5
  gp3.setMaximumNearestNeighbors (150); //default 100
  gp3.setMaximumSurfaceAngle(M_PI/2); // default M_PI/4 => 45 degrees
  gp3.setMinimumAngle(M_PI/36); // default M_PI/18 => 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false); //default: true

  // Get result
  gp3.setInputCloud (pc_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);
  cout << *pc_with_normals << endl;

  //saveVTKFile ("mesh.vtk", triangles);
  ROS_INFO(" Save mesh_i.iv in /tmp/" );
  //ROS_INFO(" Save mesh.iv in ~/.ros/(data) (roslaunch) or in pc_to_iv/(data) (rosrun) " );

  //cpp int to string conversion and publish iv name
  meshcnt = 1;
  string s;
  stringstream ss;
  std_msgs::String ivfilename;
  //ss << ++meshcnt;
  ss << meshcnt;
  s = ss.str();
  s = "/tmp/mesh_"+ s +".iv"; //name of new iv file
  ivfilename.data = s;

  //save iv File
  saveIVFile (s, triangles);
  cout << "File " << s << " saved \n";
  //publish filename of new iv file

  //if (meshcnt == 1){
  iv_filename_pub.publish(ivfilename);
  //}

  return;
}





int main(int argc, char **argv){

  ROS_INFO("Starting closed mesh generator");
  //Subscribe to point cloud topic
  //meshcnt = 0;
  ros::init (argc, argv, "pc_to_iv");
  ros::NodeHandle nh;
  CPCTOIV * pc_to_iv = new CPCTOIV(nh);


  ros::spin();
  return 0;
}
