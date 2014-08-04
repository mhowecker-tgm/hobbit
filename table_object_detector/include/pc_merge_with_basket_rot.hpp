/*
 * pc_merge_with_basket_rot.hpp
 *
 *  Created on: Dec 29, 2011
 *  Changed on: Aug 04, 2014
 *      Author: df
 */

#ifndef PC_MERGE_WITH_BASKET_ROT_HPP_
#define PC_MERGE_WITH_BASKET_ROT_HPP_
#define SOFTSTUFFHEIGHT 0.035		//david, new 30.1.2012 => 1.5 cm for inaccurace

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl/io/io.h"
#include "pcl_ros/publisher.h"
#include "pcl_ros/transforms.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <boost/thread/mutex.hpp>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"

#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

#include <sstream>

using namespace std;

class CPCMerge
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	bool pc_cam1_filled, pc_cam2_filled;
	bool use_two_cams;
	ros::Publisher pc_merged_pub;
	ros::Publisher nr_segmented_pc_pub;
	ros::Publisher pc_for_basketdet_pub;
	ros::Publisher pc_for_basketdet_cam1_pub;		//in camera coordinate system, basket points eliminated
	ros::Publisher position_highestpoint_pub;
	ros::Subscriber pc_cam1_sub;
	ros::Subscriber pc_cam1_sub2;
	ros::Subscriber pc_cam2_sub;
	pcl::PointCloud<pcl::PointXYZ> pc_cam1;
	pcl::PointCloud<pcl::PointXYZ> pc_cam2;
	pcl::PointCloud<pcl::PointXYZ> pc_for_basketdet_out;
  	ros::NodeHandle nh;
  	boost::mutex m;

	pcl::PointCloud<pcl::PointXYZ> pcl_cloud_cam1_without_basket;

	tf::TransformListener tf_listener;

	CPCMerge(ros::NodeHandle nh_);

	// merges point clouds from 2 different cameras
	void publish_merged_pc();
	// filters outliers and points outside defined region
/*	void filter_pc(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_merged,
			       bool pub_basket_position=true, float x_min=0.25, float x_max=0.90,
			       float y_min=-0.55, float y_max=0.25, float z_min=0.035, float z_max = 0.4);
*/      //bigger scope
	void filter_pc(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_merged,
			       bool pub_basket_position=true, float x_min=0.0, float x_max=1.50,
			       float y_min=-1.0, float y_max=0.25, float z_min=0.035, float z_max = 0.9);

	void segment_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_merged );
	// receives point clouds from camera #1
	void pc_cam1_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_in);

	// basket detection
	template <typename PointT>
	void basket_detection(pcl::PointCloud<PointT>& pcl_cloud_merged,
			              bool pub_basket_position);
};

template <typename PointT>
void CPCMerge::basket_detection(pcl::PointCloud<PointT>& pcl_cloud_merged,
		                        bool pub_basket_position)
{
	pcl::PointCloud<PointT> pc_for_basketdet;
	pcl::PointCloud<PointT> pc_for_basketdet2;
	pcl::PointCloud<PointT> pc_for_basketdet3;
	pcl::PointCloud<PointT> pc_for_basketdet_help;
	pcl::PointCloud<PointT> final;

	pcl::PointCloud<PointT> cloud_tmp_rotated;
    pcl::PointCloud<PointT> cloud_out;

    pc_for_basketdet.header.frame_id = pcl_cloud_merged.header.frame_id;
    pc_for_basketdet2.header.frame_id = pcl_cloud_merged.header.frame_id;
    pc_for_basketdet3.header.frame_id = pcl_cloud_merged.header.frame_id;
    pc_for_basketdet_help.header.frame_id = pcl_cloud_merged.header.frame_id;

	//BASKET DETECTION and box deletion

	//step_1
	//filter by height
    ROS_INFO("%d",pcl_cloud_merged.points.size());
	for (unsigned int i = 0; i < pcl_cloud_merged.points.size(); ++i)
	{
	  if (pcl_cloud_merged.points[i].z < 0.1)
	  {
		PointT pnt_tmp = pcl_cloud_merged.points[i];
		pc_for_basketdet.push_back(pnt_tmp);
	  }
	}
	ROS_INFO("%d",pc_for_basketdet.points.size());
	//step_2
	//find max/min for x/y
	double x_mi = 1000;
	double x_ma = -1000;
	double y_mi = 1000;
	double y_ma = -1000;
	double z_ma = -1000;
	for (unsigned int i = 0; i < pc_for_basketdet.points.size(); ++i)
	{
	  if (pc_for_basketdet.points[i].x > x_ma)
		x_ma = pc_for_basketdet.points[i].x;
	  if (pc_for_basketdet.points[i].x < x_mi)
		x_mi = pc_for_basketdet.points[i].x;
	  if (pc_for_basketdet.points[i].y > y_ma)
		y_ma = pc_for_basketdet.points[i].y;
	  if (pc_for_basketdet.points[i].y < y_mi)
	    y_mi = pc_for_basketdet.points[i].y;

	  if (pc_for_basketdet.points[i].z > z_ma)
		z_ma = pc_for_basketdet.points[i].z;
	}

	//step_3
	//define approximate center from step_2
	double x_cent = (x_ma + x_mi)/2;
	double y_cent = (y_ma + y_mi)/2;
	ROS_INFO("nx_cent and y_cent approximately: %f %f",x_cent,y_cent);

	//step_4 and step_5
	//delete points which are in circle around center point or in the corner opposite the camera, set z-coordinate to 0
	// @@@ why 0.12 -- ?? this parameter derived from what?
	float box_rad_min = 0.12;
	for (unsigned int i = 0; i < pc_for_basketdet.points.size(); ++i)
	{
	  if (( sqrt(pow(pc_for_basketdet.points[i].x - x_cent, 2.0) +
			     pow(pc_for_basketdet.points[i].y - y_cent, 2.0)) > box_rad_min ) and //cout out cirqule arround approximate center
		  ((pc_for_basketdet.points[i].x < x_cent) or pc_for_basketdet.points[i].y < y_cent))
	  {   //cout out "left upper corner
		PointT pnt_tmp = pc_for_basketdet.points[i];
		pnt_tmp.z = 0.0;
		pc_for_basketdet2.push_back(pnt_tmp);
	  }
	}

	//while-loop to fix bug which occurs because plane in the background was selected
	bool front_plane_not_found = true;
	bool front_plane_x_val_ok = false;
	bool front_plane_y_val_ok = false;
	Eigen::VectorXf model_coefficients;
	float lpnt_x, lpnt_y;
	while (front_plane_not_found){

	  ROS_INFO("DAVID schleife mit step_6");
	  ROS_INFO("%d",pc_for_basketdet2.height);
	  ROS_INFO("%f",pc_for_basketdet2.points[100].x);
	  ROS_INFO("david ende");
	  ROS_INFO("%d",pc_for_basketdet2.width);
	  //step_6, find front boarder of box (best visible from camera view)
	  std::vector<int> inliers;

	  boost::shared_ptr<pcl::SampleConsensusModelLine<PointT> >
	        model_l (new pcl::SampleConsensusModelLine<PointT> (pc_for_basketdet2.makeShared()));
	  pcl::RandomSampleConsensus<PointT> ransac (model_l);
	  ransac.setDistanceThreshold (.007);
	  ransac.computeModel();
	  ransac.getInliers(inliers);
	  ransac.getModelCoefficients(model_coefficients);

	  ROS_INFO("model_coefficients : %f, %f, %f, %f, %f, %f",
			   model_coefficients[0],model_coefficients[1],model_coefficients[2],
			   model_coefficients[3],model_coefficients[4],model_coefficients[5]);
	  lpnt_x = model_coefficients[0];
	  lpnt_y = model_coefficients[1];

	  // copies all inliers of the model computed to another PointCloud
	  pc_for_basketdet3.clear();
	  pcl::copyPointCloud<PointT>(pc_for_basketdet2, inliers, pc_for_basketdet3);

	  //step_7
	  // Calculate max length of detected (2d) box-edge => to know if we took "long" or "short" edge
	  bool edge_front_ok = false;		//if still false after loop, selected edge is for sure not in front
	  front_plane_x_val_ok = false;
	  front_plane_y_val_ok = false;
	  ROS_INFO("front_plane_x_val_ok: %c",front_plane_x_val_ok);
	  ROS_INFO("front_plane_y_val_ok: %c",front_plane_y_val_ok);

	  for (unsigned int i = 0; i < pc_for_basketdet3.points.size(); ++i)
	  {
		float dist = sqrt(pow(pc_for_basketdet3.points[i].x - lpnt_x, 2.0) +
				          pow(pc_for_basketdet3.points[i].y - lpnt_y, 2.0));
		if (pc_for_basketdet3.points[i].x < x_cent)
		{
		  front_plane_x_val_ok = true;
		}
		if (pc_for_basketdet3.points[i].y < y_cent)
		{
		  front_plane_y_val_ok = true;
		}
	  }

	  //cerr << "\nfront_plane_x_val_ok: " << front_plane_x_val_ok << "\nfront_plane_y_val_ok: " << front_plane_y_val_ok << endl;
	  edge_front_ok = front_plane_x_val_ok and front_plane_y_val_ok;
	  //cerr << "\n edge_front_ok: " << edge_front_ok << endl;
	  if (edge_front_ok)
	  {
		//cerr << "\n edge_front_ok !!!!! \n";
		front_plane_not_found = false;	// front_plane probably found => can quit while-loop
	  } else {							//delete wrong plane and do "plane"-selection again
		  ROS_INFO("edge_front_ NOT_ok !!!!!");

		  //get cloud without the first found plane inliers
		  //copy pc_basketdet2 to pc_basketdet2 but without wrong found inliers
		  pc_for_basketdet_help.clear();
		  pcl::copyPointCloud<PointT>(pc_for_basketdet2, pc_for_basketdet_help);
		  pc_for_basketdet2.clear();

		  int inlier_array[pc_for_basketdet_help.points.size()];

		  for (int k=0; k < pc_for_basketdet_help.points.size(); k++)
		  {
			inlier_array[k] = 0;
		  }
		  for (int k=0; k<inliers.size();k++)
		  {
			int idc = inliers[k];
			inlier_array[idc] = 1;
		  }
		  //copy pc_for_basketdet_help to pc_for_basketdet2 without inliers
		  for (unsigned int i = 0; i < pc_for_basketdet_help.points.size(); ++i)
		  {
			if (inlier_array[i] == 0)
			{
			  PointT pnt_tmp = pc_for_basketdet_help.points[i];
			  pc_for_basketdet2.push_back(pnt_tmp);
			}
		  }
		}
	  }


	  double dist=0, dist1=0,dist2=0;	//biggest distance of the boarder length (=>dist) is (~) equal dist1+dist2 (=> distances to the returned point from model_coefficients)
	  double p1_x,p1_y, p2_x,p2_y;

	  if (fabs(model_coefficients[3]) > fabs(model_coefficients[4]))
	  {	//if boarder is more aligned with x- than in y-direction
	    for (unsigned int i = 0; i < pc_for_basketdet3.points.size(); ++i)
	    {
	      if (pc_for_basketdet3.points[i].x > lpnt_x)
	      {
	      	if (sqrt(pow(pc_for_basketdet3.points[i].x - lpnt_x, 2.0) +
	      			 pow(pc_for_basketdet3.points[i].y - lpnt_y, 2.0)) > dist1)
	      	{
	          dist1 = sqrt(pow(pc_for_basketdet3.points[i].x - lpnt_x, 2.0) + pow(pc_for_basketdet3.points[i].y - lpnt_y, 2.0));
	          p1_x = pc_for_basketdet3.points[i].x;
	          p1_y = pc_for_basketdet3.points[i].y;
	        }
	      } else {
	      	if (sqrt(pow(pc_for_basketdet3.points[i].x - lpnt_x, 2.0) +
	      			 pow(pc_for_basketdet3.points[i].y - lpnt_y, 2.0)) > dist2)
	      	{
	          dist2 = sqrt(pow(pc_for_basketdet3.points[i].x - lpnt_x, 2.0) + pow(pc_for_basketdet3.points[i].y - lpnt_y, 2.0));
	          p2_x = pc_for_basketdet3.points[i].x;
	          p2_y = pc_for_basketdet3.points[i].y;
	        }
	      }
	    }
	  } else {	// boarder direction is more aligned with y- than in x-direction
	    for (unsigned int i = 0; i < pc_for_basketdet3.points.size(); ++i)
	    {
	      if (pc_for_basketdet3.points[i].y > lpnt_y)
	      {
	      	if (sqrt(pow(pc_for_basketdet3.points[i].x - lpnt_x, 2.0) +
	      			 pow(pc_for_basketdet3.points[i].y - lpnt_y, 2.0)) > dist1)
	      	{
	           dist1 = sqrt(pow(pc_for_basketdet3.points[i].x - lpnt_x, 2.0) + pow(pc_for_basketdet3.points[i].y - lpnt_y, 2.0));
	           p1_x = pc_for_basketdet3.points[i].x;
	           p1_y = pc_for_basketdet3.points[i].y;
	        }
	      } else {
	      	if (sqrt(pow(pc_for_basketdet3.points[i].x - lpnt_x, 2.0) +
	      			 pow(pc_for_basketdet3.points[i].y - lpnt_y, 2.0)) > dist2)
	      	{
	           dist2 = sqrt(pow(pc_for_basketdet3.points[i].x - lpnt_x, 2.0) + pow(pc_for_basketdet3.points[i].y - lpnt_y, 2.0));
	           p2_x = pc_for_basketdet3.points[i].x;
	           p2_y = pc_for_basketdet3.points[i].y;
	        }
	      }
	    }
	  }
	  dist = dist1+dist2;
	  ROS_INFO("dist: %f",dist);
	  double const box_height = 32;
	  double const box_width = 44;
	  double length_edge,length_other_edge;
	  bool long_edge_selected  = false;
	  if (dist*100 < (box_height+box_width)/2)
	  {
	  	length_edge = box_height;
	   	length_other_edge = box_width;
	  } else {
	  	long_edge_selected = true;
	   	length_edge = box_width;
	   	length_other_edge = box_height;
	  }
	  ROS_INFO("variable: long_edge_selected: %f",long_edge_selected);
	  //make 90 degree rotation of direction vector from segment_line model (swap x,y-coordinates and make x negativ)
	  double n_vec_line_x = - model_coefficients[4];
	  double n_vec_line_y =   model_coefficients[3];
	  if (fabs(n_vec_line_x) > fabs(n_vec_line_y))
	  {
	  	//normal vector is manly pointing in x or -x direction
	   	if (n_vec_line_x < 0)
	   	{	//swap direction s.t. it points to center
	      n_vec_line_x = -n_vec_line_x;
	      n_vec_line_y = -n_vec_line_y;
	    }
	  } else {
	   	//normal vector is manly pointing in y or -y direction
	   	if (n_vec_line_y < 0)
	   	{	//swap direction s.t. it points to center
	      n_vec_line_x = -n_vec_line_x;
	      n_vec_line_y = -n_vec_line_y;
	    }
	  }

	  double box_center_x = (p1_x + p2_x)/2 + n_vec_line_x*length_other_edge/2*0.01;
	  double box_center_y = (p1_y + p2_y)/2 + n_vec_line_y*length_other_edge/2*0.01;
	  double alpha;

	  ROS_INFO("box_center_x and box_center_y (genau): %f %f",box_center_x,box_center_y);

	  //assume that model_coefficients[3] and model_coefficients[4] is never 0 !!
	  if (long_edge_selected){
	  	alpha = atan(model_coefficients[3]/model_coefficients[4]);
	  } else {
	   	alpha = atan(-model_coefficients[4]/model_coefficients[3]);
	}
	alpha = -alpha;

	//step_11
	//delete points which belong to basket: rotate that pc is axis allignedm, than delete box-points and rotate back

	//NEW begin
	Eigen::Matrix4f mat_sh_orig = Eigen::Matrix4f::Identity(); //matrix which defines shift of point cloud from center to origin
	Eigen::Matrix4f mat_rot = Eigen::Matrix4f::Identity(); //matrix which rotates pc and then shifts pc back to box center
	Eigen::Matrix4f mat_transform = Eigen::Matrix4f::Identity(); //matrix for transformation of pointcloud (rototion about box center)

	mat_sh_orig(0,3) = -box_center_x;  //shift x value
	mat_sh_orig(1,3) = -box_center_y;  //shift y-value
	mat_rot(0,3) = box_center_x;
	mat_rot(1,3) = box_center_y;

	float angel = -alpha;
	mat_rot(0,0) = cos(angel);
	mat_rot(0,1) = -sin(angel);
	mat_rot(1,0) = sin(angel);
	mat_rot(1,1) = cos(angel);
	mat_transform = mat_rot*mat_sh_orig;

	pcl::transformPointCloud(pcl_cloud_merged, cloud_tmp_rotated, mat_transform);
	float th = 0.03;
	float th_z = 0.098;
	x_mi = box_center_x - 0.01*box_height/2;
	x_ma = box_center_x + 0.01*box_height/2;
	y_mi = box_center_y - 0.01*box_width/2;
	y_ma = box_center_y + 0.01*box_width/2;
	for (unsigned int i = 0; i < cloud_tmp_rotated.points.size(); ++i)
	{
	  //delete foam plastic on ground
	  if (cloud_tmp_rotated.points[i].z < SOFTSTUFFHEIGHT)
		  continue;	// <==> delete point
	  //delete foam plastic on side (the sloped one)
	  const float cm4 = 0.04;
	  const float cm1 = 0.01;
	  float th1 = th + cm4;	//threshhold for points th+3cm (=> 6cm) near the box face is 0.05
	  if ((cloud_tmp_rotated.points[i].z < 0.06) and
		  ((cloud_tmp_rotated.points[i].x < x_mi + th1 and cloud_tmp_rotated.points[i].x > x_mi - th1 ) or
		   (cloud_tmp_rotated.points[i].x < x_ma + th1 and cloud_tmp_rotated.points[i].x > x_ma - th1 ) or
		   (cloud_tmp_rotated.points[i].y < y_mi + th1 and cloud_tmp_rotated.points[i].y > y_mi - th1 ) or
		   (cloud_tmp_rotated.points[i].y < y_ma + th1 and cloud_tmp_rotated.points[i].y > y_ma - th1 )))
	  {
		continue;	// <==> delete point
	  }
	  float th2 = th + cm1;	//threshhold for points th+1cm (=> 4cm) near the box face is 0.07
	  if ((cloud_tmp_rotated.points[i].z < 0.08) and
		  ((cloud_tmp_rotated.points[i].x < x_mi + th2 and cloud_tmp_rotated.points[i].x > x_mi - th2 ) or
		   (cloud_tmp_rotated.points[i].x < x_ma + th2 and cloud_tmp_rotated.points[i].x > x_ma - th2 ) or
		   (cloud_tmp_rotated.points[i].y < y_mi + th2 and cloud_tmp_rotated.points[i].y > y_mi - th2 ) or
		   (cloud_tmp_rotated.points[i].y < y_ma + th2 and cloud_tmp_rotated.points[i].y > y_ma - th2 )))
	  {
		continue;	// <==> delete point
	  }

	  //delete box
	  if ((cloud_tmp_rotated.points[i].z < th_z) and
		  ((cloud_tmp_rotated.points[i].x < x_mi + th and cloud_tmp_rotated.points[i].x > x_mi - th ) or
		   (cloud_tmp_rotated.points[i].x < x_ma + th and cloud_tmp_rotated.points[i].x > x_ma - th ) or
		   (cloud_tmp_rotated.points[i].y < y_mi + th and cloud_tmp_rotated.points[i].y > y_mi - th ) or
		   (cloud_tmp_rotated.points[i].y < y_ma + th and cloud_tmp_rotated.points[i].y > y_ma - th )))
	  {
		continue;	// <==> delete point
	  } else {
		PointT pnt_tmp = pcl_cloud_merged.points[i];
		cloud_out.push_back(pnt_tmp);
	  }
	}

	//assign (basket-)filtered point cloud to input point cloud
	pcl_cloud_merged = cloud_out;

	if (pub_basket_position)
	{
	  //Publish corner points of basket
	  std_msgs::String msgStrCorners;
	  std::stringstream ss;
	  ss << box_center_x << " "  << box_center_y << " " << alpha;
	  msgStrCorners.data = ss.str();
	  position_highestpoint_pub.publish(msgStrCorners);
	}

	//END BASKET DETECTION
}

#endif /* PC_MERGE_WITH_BASKET_ROT_HPP_ */
