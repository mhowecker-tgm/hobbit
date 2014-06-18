#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/PointCloud2.h>
#include "HobbitMsgs/PointCloud2FeatureHistogram.h"

ros::Publisher pub;
ros::Subscriber sub;


class PlaneFinder
{
public:
	ros::NodeHandle nh_;
	vector<vector<vector<bool> > > lut;
    ros::ServiceServer service;

    PlaneFinder(): nh_ ("~")
	{
		lut.resize(GRIDSIZE);
		for (int i = 0; i < GRIDSIZE; ++i)
		{
			lut[i].resize(GRIDSIZE);
			for (int j = 0; j < GRIDSIZE; ++j)
				lut[i][j].resize(GRIDSIZE);
		}

    	this->service = nh_.advertiseService("do_feature_D2C", &D2C::do_D2C, this);
	}

	bool do_D2C(recognition_msgs::PointCloud2FeatureHistogram::Request& req, recognition_msgs::PointCloud2FeatureHistogram::Response& res)
	{

		ros::Time t1 = ros::Time::now ();
	    pcl::PointCloud<pcl::PointXYZ> cloud;
	    pcl::fromROSMsg (req.point_cloud, cloud);
	    scale_points_unit_sphere(cloud,GRIDSIZE_H);
	    this->voxelize(cloud);
	    this->export_grid_cells("cluster_1.pcd");
	    this->voxelize9(cloud);
	    this->export_grid_cells("cluster_9.pcd");
		//this->D2CMN(cloud,res.hist);
		//his->D2C_bin(cloud,res.hist);
	    //ROS_INFO ("cloude size: %d", cloud.size());
		this->D2C_binRsw(cloud,res.hist);

		this->cleanup9(cloud);
	    ROS_WARN ("SERVICE_CALL done :: Spent %f seconds in 'do_D2C'.", (ros::Time::now () - t1).toSec ());
	    return true;
	}
}



void findTurnTablePlane(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::fromROSMsg (*input, cloud);

	pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;



	  // Create the filtering object
	pcl::PassThrough<pcl::PointXYZRGB>  pass;
	pass.setInputCloud (cloud.makeShared ());
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1);
	pass.filter (cloud_filtered);

	pcl::PassThrough<pcl::PointXYZRGB>  passy;
	passy.setInputCloud (cloud_filtered.makeShared ());
	passy.setFilterFieldName ("y");
	passy.setFilterLimits (-1.0, 0);
	passy.filter (cloud_filtered);

	pcl::PassThrough<pcl::PointXYZRGB>  passx;
	passx.setInputCloud (cloud_filtered.makeShared ());
	passx.setFilterFieldName ("x");
	passx.setFilterLimits (0.0, 0.9);
	passx.filter (cloud_filtered);

	pcl::ModelCoefficients coefficients;
	//pcl::PointIndices inliers;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud_filtered.makeShared ());
	seg.segment (*inliers, coefficients);

	std::cout << coefficients.values[0] << " " << coefficients.values[1] << " " << coefficients.values[2] << " " << coefficients.values[3] << " \n";
	if (inliers->indices.size () == 0)
	{
	std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	return;
	}

	// Extract the inliers
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud (cloud_filtered.makeShared ());
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (cloud_filtered);

	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg (cloud_filtered, output);

	// Publish the data
	pub.publish (output);
}




void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	findTurnTablePlane(input);
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "tod");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("clusters", 1);
  ros::spin();


}
