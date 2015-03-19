#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <ros/node_handle.h>
//#include "/opt/ros/hydro/include/ros/node_handle.h"
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
#include <vector>
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
#include "hobbit_msgs/PointCloud2FeatureHistogram.h"
#include "hobbit_msgs/ClustersOnPlane.h"
#include "hobbit_msgs/SingleShotPC.h"
#include <string>

class CloudSegmenthor {
public:
	ros::NodeHandle nh_;
	ros::ServiceServer fttp_service;
	ros::ServiceServer fgp_service;
	ros::ServiceServer ftp_service; //df: find table plane

	ros::ServiceServer foott_service;
	ros::ServiceServer foof_service;
	ros::ServiceServer foofs_service;
	ros::ServiceServer foot_service; //df: find objects on table
	ros::ServiceServer singleshot_service;
	ros::ServiceServer tsout_service;

	double cloud_stamp_;
	std::string frame;
	ros::Time ct;
	pcl::PointCloud<pcl::PointXYZRGB> pc;
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_;
	ros::Subscriber cloud_sub;
	ros::Publisher pub;
	CloudSegmenthor() :
			nh_("~"), cloud_stamp_(0), frame("") {
		this->fttp_service = nh_.advertiseService("findTurnTablePlane",
				&CloudSegmenthor::do_service_findTurnTablePlane, this);
		this->fgp_service = nh_.advertiseService("findGroundPlane",
				&CloudSegmenthor::do_service_findGroundPlane, this);
		//df start
		this->ftp_service = nh_.advertiseService("findTablePlane",
				&CloudSegmenthor::do_service_findTablePlane, this);
		//df end
		this->tsout_service = nh_.advertiseService("touchScreenOutCheck",
				&CloudSegmenthor::do_service_touchScreenOutCheck, this);
		this->foott_service = nh_.advertiseService("findObjectOnTurnTable",
				&CloudSegmenthor::do_service_findObjectOnTurnTable, this);
		this->foof_service = nh_.advertiseService("findObjectsOnFloor",
				&CloudSegmenthor::do_service_findObjectsOnFloor, this);
		//df new 19.3.2015
		this->foofs_service = nh_.advertiseService("findObjectsOnFloorSmall",
				&CloudSegmenthor::do_service_findObjectsOnFloorSmall, this);
		//df start: find objects on table
		this->foot_service = nh_.advertiseService("findObjectsOnTable",
				&CloudSegmenthor::do_service_findObjectsOnTable, this);
		//df end
		this->singleshot_service = nh_.advertiseService("getSingleShot",
				&CloudSegmenthor::do_service_getSingleShot, this);
		this->cloud_sub = nh_.subscribe("/SS/headcam/depth_registered/points", 1,
				&CloudSegmenthor::callbackPointCloud, this);
//	    this->cloud_sub = nh_.subscribe ("/headcam/depth_registered/points", 1, &CloudSegmenthor::callbackPointCloud, this);
		this->pub = nh_.advertise<sensor_msgs::PointCloud2>("/debug", 1);

	}

	// input: pointcloud2, output: float array
	bool do_service_touchScreenOutCheck(
			hobbit_msgs::PointCloud2FeatureHistogram::Request& req,
			hobbit_msgs::PointCloud2FeatureHistogram::Response& res) {
		ROS_INFO("SERVICE_CALL touchScreenOutCheck");
		ros::Time t1 = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		pcl::fromROSMsg(req.point_cloud, cloud);
		this->checkTSisOut(cloud, res.hist);
		ROS_INFO(
				"SERVICE_CALL touchScreenOutCheck  done :: Spent %f seconds.", (ros::Time::now () - t1).toSec ());
		return true;
	}

	// input: pointcloud2, output: float array
	bool do_service_findGroundPlane(
			hobbit_msgs::PointCloud2FeatureHistogram::Request& req,
			hobbit_msgs::PointCloud2FeatureHistogram::Response& res) {
		ROS_INFO("SERVICE_CALL findGroundPlane");
		ros::Time t1 = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		pcl::fromROSMsg(req.point_cloud, cloud);
		this->findGroundPlane(cloud, res.hist);
		ROS_INFO(
				"SERVICE_CALL findGroundPlane  done :: Spent %f seconds.", (ros::Time::now () - t1).toSec ());
		return true;
	}

	//df start 31.01.2013
	// input: pointcloud2, output: float array
	bool do_service_findTablePlane(
			hobbit_msgs::PointCloud2FeatureHistogram::Request& req,
			hobbit_msgs::PointCloud2FeatureHistogram::Response& res) {
		ROS_INFO("SERVICE_CALL findTablePlane");
		ros::Time t1 = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		pcl::fromROSMsg(req.point_cloud, cloud);
		this->findTablePlane(cloud, res.hist);
		ROS_INFO(
				"SERVICE_CALL findTablePlane  done :: Spent %f seconds.", (ros::Time::now () - t1).toSec ());
		return true;
	}
	//df end

	// input: pointcloud2, output: float array
	bool do_service_findTurnTablePlane(
			hobbit_msgs::PointCloud2FeatureHistogram::Request& req,
			hobbit_msgs::PointCloud2FeatureHistogram::Response& res) {
		ROS_INFO("SERVICE_CALL findTurnTablePlane");
		ros::Time t1 = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		pcl::fromROSMsg(req.point_cloud, cloud);
		this->findTurnTablePlane(cloud, res.hist);
		ROS_INFO(
				"SERVICE_CALL findTurnTablePlane  done :: Spent %f seconds.", (ros::Time::now () - t1).toSec ());
		return true;
	}

	// input: turntable plane as float array, output: vector of pointcloud2
	bool do_service_findObjectOnTurnTable(
			hobbit_msgs::ClustersOnPlane::Request& req,
			hobbit_msgs::ClustersOnPlane::Response& res) {
		ROS_INFO("SERVICE_CALL findOBJECTonTurnTablePlane");
		ros::Time t1 = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		pcl::fromROSMsg(req.point_cloud, cloud);
		this->findObjectOnTurnTable(cloud, req.plane, res.clusters);
		res.clusters[0].header.frame_id = req.point_cloud.header.frame_id;
		res.clusters[0].header.stamp = req.point_cloud.header.stamp;

		ROS_WARN(
				"SERVICE_CALL findObjectOnTurnTable  done :: Spent %f seconds.", (ros::Time::now () - t1).toSec ());
		return true;
	}

	// input: turntable plane as float array, output: vector of pointcloud2
	bool do_service_findObjectsOnFloor(
			hobbit_msgs::ClustersOnPlane::Request& req,
			hobbit_msgs::ClustersOnPlane::Response& res) {
		ROS_INFO("SERVICE_CALL findOBJECTonFLOOR");
		ROS_INFO(" %s ",req.point_cloud.header.frame_id.c_str());
		ros::Time t1 = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		pcl::fromROSMsg(req.point_cloud, cloud);
		this->findObjectsOnFloor(cloud, req.plane, res.clusters);

		for (int i = 0; i < res.clusters.size(); i++) {
			res.clusters[i].header.frame_id = req.point_cloud.header.frame_id;
			res.clusters[i].header.stamp = req.point_cloud.header.stamp;
		}

		ROS_WARN(
				"SERVICE_CALL findObjectOnFloor  done :: Spent %f seconds.", (ros::Time::now () - t1).toSec ());
		return true;
	}


	// new 19.3.2015
	bool do_service_findObjectsOnFloorSmall(
			hobbit_msgs::ClustersOnPlane::Request& req,
			hobbit_msgs::ClustersOnPlane::Response& res) {
		ROS_INFO("SERVICE_CALL findOBJECTonFLOORSmall");
		ROS_INFO(" %s ",req.point_cloud.header.frame_id.c_str());
		ros::Time t1 = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		pcl::fromROSMsg(req.point_cloud, cloud);
		this->findObjectsOnFloorSmall(cloud, req.plane, res.clusters);

		for (int i = 0; i < res.clusters.size(); i++) {
			res.clusters[i].header.frame_id = req.point_cloud.header.frame_id;
			res.clusters[i].header.stamp = req.point_cloud.header.stamp;
		}

		ROS_WARN(
				"SERVICE_CALL findObjectOnFloor  done :: Spent %f seconds.", (ros::Time::now () - t1).toSec ());
		return true;
	}


	//df start 31.01.2013
	// input: table plane as float array, output: vector of pointcloud2
	bool do_service_findObjectsOnTable(
			hobbit_msgs::ClustersOnPlane::Request& req,
			hobbit_msgs::ClustersOnPlane::Response& res) {
		ROS_INFO("SERVICE_CALL findOBJECTonTABLE");
		ROS_INFO(" %s ",req.point_cloud.header.frame_id.c_str());
		ros::Time t1 = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		pcl::fromROSMsg(req.point_cloud, cloud);
		this->findObjectsOnTable(cloud, req.plane, res.clusters);

		for (int i = 0; i < res.clusters.size(); i++) {
			res.clusters[i].header.frame_id = req.point_cloud.header.frame_id;
			res.clusters[i].header.stamp = req.point_cloud.header.stamp;
		}

		ROS_WARN(
				"SERVICE_CALL findObjectOnTable  done :: Spent %f seconds.", (ros::Time::now () - t1).toSec ());
		return true;
	}
	//df end


	// input: string, output: pointcloud2
	bool do_service_getSingleShot(hobbit_msgs::SingleShotPC::Request& req,
			hobbit_msgs::SingleShotPC::Response& res) {
		ros::Time t1 = ros::Time::now();
		//pcl::toROSMsg (this->pc, res.point_cloud);
		pcl::toROSMsg(*this->cloud_, res.point_cloud);

		res.point_cloud.header.stamp = this->ct;
		res.point_cloud.header.frame_id = this->frame;
		ROS_WARN(
				"SERVICE_CALL getSingleShot  done :: Spent %f seconds.", (ros::Time::now () - t1).toSec ());
		return true;
	}

	void callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud2_in) {
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		pcl::fromROSMsg(*cloud2_in, cloud);
//		this->cloud_ = boost::make_shared<const pcl::PointCloud<Point> > (cloud);
		this->cloud_ = cloud.makeShared();
		this->cloud_stamp_ = cloud2_in->header.stamp.toSec();
		this->ct = cloud2_in->header.stamp;
		this->frame = cloud2_in->header.frame_id;
	}

	void findTurnTablePlane(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
			std::vector<float> &planeparams) {
		pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
		// Create the filtering object
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(cloud.makeShared());
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.4, 0.8);
		pass.filter(cloud_filtered);

		pcl::PassThrough<pcl::PointXYZRGB> passy;
		passy.setInputCloud(cloud_filtered.makeShared());
		passy.setFilterFieldName("y");
		passy.setFilterLimits(-0.8, 0.1);
		passy.filter(cloud_filtered);

		pcl::PassThrough<pcl::PointXYZRGB> passx;
		passx.setInputCloud(cloud_filtered.makeShared());
		passx.setFilterFieldName("x");
		passx.setFilterLimits(0.0, 0.9);
		passx.filter(cloud_filtered);

		pcl::ModelCoefficients coefficients;
		//pcl::PointIndices inliers;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.01);

		seg.setInputCloud(cloud_filtered.makeShared());
		seg.segment(*inliers, coefficients);

		if (inliers->indices.size() == 0) {
			std::cerr
					<< "Could not estimate a planar model for the given dataset."
					<< std::endl;
			return;
		}

		std::cout << coefficients.values[0] << " " << coefficients.values[1]
				<< " " << coefficients.values[2] << " "
				<< coefficients.values[3] << " \n";

		ROS_INFO(
				"table coefficients ::  %f %f %f %f ", coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3]);

		planeparams.push_back(coefficients.values[0]);
		planeparams.push_back(coefficients.values[1]);
		planeparams.push_back(coefficients.values[2]);
		planeparams.push_back(coefficients.values[3]);

		// Extract the inliers
		//pcl::ExtractIndices<pcl::PointXYZRGBRGB> extract;
		//extract.setInputCloud (cloud_filtered.makeShared ());
		//extract.setIndices (inliers);
		//extract.setNegative (false);
		//extract.filter (cloud_filtered);

		//sensor_msgs::PointCloud2 output;
		//pcl::toROSMsg (cloud_filtered, output);

		// Publish the data
		//pub.publish (output);
	}

	void checkTSisOut(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
			std::vector<float> &planeparams) {
		pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
		// Create the filtering object
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(cloud.makeShared());
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.4, 0.8);
		pass.filter(cloud_filtered);

		pcl::PassThrough<pcl::PointXYZRGB> passy;
		passy.setInputCloud(cloud_filtered.makeShared());
		passy.setFilterFieldName("y");
		passy.setFilterLimits(-0.8, 0.1);
		passy.filter(cloud_filtered);

		pcl::PassThrough<pcl::PointXYZRGB> passx;
		passx.setInputCloud(cloud_filtered.makeShared());
		passx.setFilterFieldName("x");
		passx.setFilterLimits(0.0, 0.9);
		passx.filter(cloud_filtered);

		pcl::ModelCoefficients coefficients;
		//pcl::PointIndices inliers;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.01);

		seg.setInputCloud(cloud_filtered.makeShared());
		seg.segment(*inliers, coefficients);

		if (inliers->indices.size() == 0) {
			std::cerr
					<< "Could not estimate a planar model for the given dataset."
					<< std::endl;
			return;
		}

		std::cout << coefficients.values[0] << " " << coefficients.values[1]
				<< " " << coefficients.values[2] << " "
				<< coefficients.values[3] << " \n";

		ROS_INFO(
				"table coefficients ::  %f %f %f %f ", coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3]);

		planeparams.push_back(coefficients.values[0]);
		planeparams.push_back(coefficients.values[1]);
		planeparams.push_back(coefficients.values[2]);
		planeparams.push_back(coefficients.values[3]);

		// Extract the inliers
		//pcl::ExtractIndices<pcl::PointXYZRGBRGB> extract;
		//extract.setInputCloud (cloud_filtered.makeShared ());
		//extract.setIndices (inliers);
		//extract.setNegative (false);
		//extract.filter (cloud_filtered);

		//sensor_msgs::PointCloud2 output;
		//pcl::toROSMsg (cloud_filtered, output);

		// Publish the data
		//pub.publish (output);
	}

	void findGroundPlane(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
			std::vector<float> &planeparams) {
		pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
		// Create the filtering object
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(cloud.makeShared());
		pass.setFilterFieldName("z");
		pass.setFilterLimits(1.0, 1.8);
		pass.filter(cloud_filtered);

//		pcl::PassThrough<pcl::PointXYZRGB>  passy;
//		passy.setInputCloud (cloud_filtered.makeShared ());
//		passy.setFilterFieldName ("y");
//		passy.setFilterLimits (-2, 0.7);
//		passy.filter (cloud_filtered);

//		pcl::PassThrough<pcl::PointXYZRGB>  passx;
//		passx.setInputCloud (cloud_filtered.makeShared ());
//		passx.setFilterFieldName ("x");
//		passx.setFilterLimits (0.0, 0.9);
//		passx.filter (cloud_filtered);

		pcl::ModelCoefficients coefficients;
		//pcl::PointIndices inliers;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.015);

		seg.setInputCloud(cloud_filtered.makeShared());
		seg.segment(*inliers, coefficients);

		if (inliers->indices.size() == 0) {
			std::cerr
					<< "Could not estimate a planar model for the given dataset."
					<< std::endl;
			return;
		}

		planeparams.push_back(coefficients.values[0]);
		planeparams.push_back(coefficients.values[1]);
		planeparams.push_back(coefficients.values[2]);
		planeparams.push_back(coefficients.values[3]);

		std::cout << coefficients.values[0] << " " << coefficients.values[1]
				<< " " << coefficients.values[2] << " "
				<< coefficients.values[3] << " \n";

		ROS_INFO(
				"table coefficients ::  %f %f %f %f ", coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3]);

		// Extract the inliers
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(cloud_filtered.makeShared());
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(cloud_filtered);

		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(cloud_filtered, output);

		// Publish the data
		pub.publish(output);
	}

	//df start 31.01.2013
	void findTablePlane(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
			std::vector<float> &planeparams) {
		pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
		// Create the filtering object
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(cloud.makeShared());
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.4, 1.3);
		pass.filter(cloud_filtered);

//		pcl::PassThrough<pcl::PointXYZRGB>  passy;
//		passy.setInputCloud (cloud_filtered.makeShared ());
//		passy.setFilterFieldName ("y");
//		passy.setFilterLimits (-2, 0.7);
//		passy.filter (cloud_filtered);

		pcl::ModelCoefficients coefficients;
		//pcl::PointIndices inliers;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.015);

		seg.setInputCloud(cloud_filtered.makeShared());
		seg.segment(*inliers, coefficients);

		if (inliers->indices.size() == 0) {
			std::cerr
					<< "Could not estimate a planar model for the given dataset."
					<< std::endl;
			return;
		}

		planeparams.push_back(coefficients.values[0]);
		planeparams.push_back(coefficients.values[1]);
		planeparams.push_back(coefficients.values[2]);
		planeparams.push_back(coefficients.values[3]);

		std::cout << coefficients.values[0] << " " << coefficients.values[1]
				<< " " << coefficients.values[2] << " "
				<< coefficients.values[3] << " \n";

		ROS_INFO(
				"table coefficients ::  %f %f %f %f ", coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3]);

		// Extract the inliers
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(cloud_filtered.makeShared());
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(cloud_filtered);

		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(cloud_filtered, output);

		// Publish the data
		pub.publish(output);
	}
	//df end


	void findObjectOnTurnTable(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
			std::vector<float> &planeparams,
			std::vector<sensor_msgs::PointCloud2> &clusters) {
		pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
		// Create the filtering object
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(cloud.makeShared());
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.4, 0.8);
		pass.filter(cloud_filtered);

		pcl::PassThrough<pcl::PointXYZRGB> passy;
		passy.setInputCloud(cloud_filtered.makeShared());
		passy.setFilterFieldName("y");
		passy.setFilterLimits(-0.8, 0.1);
		passy.filter(cloud_filtered);

		pcl::PassThrough<pcl::PointXYZRGB> passx;
		passx.setInputCloud(cloud_filtered.makeShared());
		passx.setFilterFieldName("x");
		passx.setFilterLimits(0.0, 0.9);
		passx.filter(cloud_filtered);

		//pcl::ModelCoefficients coefficients;
		//coefficients.values[0] = planeparams[0];
		//coefficients.values[1] = planeparams[1];
		//coefficients.values[2] = planeparams[2];
		//coefficients.values[3] = planeparams[3];

		//sensor_msgs::PointCloud2 debug;
		//pcl::toROSMsg (cloud_filtered, debug);
		//pub.publish(debug);

		pcl::PointCloud<pcl::PointXYZRGB> objectcloud;
		float distthresh = 0.009;
		int cntr = 0;
		float dist = 0;
		for (size_t i = 0; i < cloud_filtered.points.size(); ++i) {
			dist = cloud_filtered.points[i].x * planeparams[0]
					+ cloud_filtered.points[i].y * planeparams[1]
					+ cloud_filtered.points[i].z * planeparams[2]
					+ planeparams[3];
			if (fabs(dist) < distthresh)
				continue;
			if (dist < distthresh) {
				objectcloud.push_back(cloud_filtered.points[i]);
				cntr++;
			}

		}
		objectcloud.height = 1;
		objectcloud.width = cntr;
		//objectcloud.size() = objectcloud.height * objectcloud.width;
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(objectcloud, output);
		clusters.push_back(output);

		//sensor_msgs::PointCloud2 output;
		//pcl::toROSMsg (cloud_filtered, output);

		// Publish the data
		//pub.publish (output);
	}

	void findObjectsOnFloor(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
			std::vector<float> &planeparams,
			std::vector<sensor_msgs::PointCloud2> &clusters) {
		pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
		// Create the filtering object
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(cloud.makeShared());
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.95, 1.7);
		pass.filter(cloud_filtered);

//		pcl::PassThrough<pcl::PointXYZRGB> passx;
//		passx.setInputCloud(cloud_filtered.makeShared());
//		passx.setFilterFieldName("y");
//		passx.setFilterLimits(-0.5, 1);
//		passx.filter(cloud_filtered);

		pcl::PointCloud<pcl::PointXYZRGB> objectcloud;
		{
			pcl::ModelCoefficients coefficients;
			//pcl::PointIndices inliers;
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			// Create the segmentation object
			pcl::SACSegmentation<pcl::PointXYZRGB> seg;
			// Optional
			seg.setOptimizeCoefficients(true);
			// Mandatory
			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.03);
			//float axisX = 0.0, axisY=-0.70710678, axisZ= -0.70710678, epsAngle = 40.0; //df axis for normal vector in ccs
			//seg.setAxis(Eigen::Vector3f(0.100505, 0.372986, 0.922378));	//df funkt nicht mal mit SACMODEL_PERPENDICULAR_PLANE
			//seg.setEpsAngle( /*epsAngle*/15.0/180.0*3.14 ); 			//df deviation angle in rad
			//std::cout << "epsangle: " << 15.0/180.0*3.14 << " \n";
			seg.setInputCloud(cloud_filtered.makeShared());
			seg.segment(*inliers, coefficients);

			if (inliers->indices.size() == 0) {
				std::cerr
						<< "Could not estimate a planar model for the given dataset."
						<< std::endl;
				return;
			}

			planeparams.push_back(coefficients.values[0]);
			planeparams.push_back(coefficients.values[1]);
			planeparams.push_back(coefficients.values[2]);
			planeparams.push_back(coefficients.values[3]);
			std::cout << coefficients.values[0] << " " << coefficients.values[1] << " " << coefficients.values[2] << " " << coefficients.values[3] << " \n";
			//0.0630656 0.467184 0.881908 -1.17102
			float distthresh = 0.019;
			int cntr = 0;
			for (size_t i = 0; i < cloud_filtered.points.size(); ++i)
			{
				float dist = 0.0;
				dist = cloud_filtered.points[i].x * coefficients.values[0]
						+ cloud_filtered.points[i].y * coefficients.values[1]
						+ cloud_filtered.points[i].z * coefficients.values[2]
						+ coefficients.values[3];
				if (dist > -distthresh)
					continue;

				//if (dist > distthresh)
//				if (dist < 0.20)
//					std::cout << dist << " " ;


				{
					objectcloud.push_back(cloud_filtered.points[i]);
					cntr++;
					}

			}
			objectcloud.height = 1;
			objectcloud.width = cntr;
		}
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg (objectcloud, output);
		output.header.frame_id = "/headcam_rgb_optical_frame";
		// Publish the data (= data without detected plane => ros topic /debug)
		pub.publish (output);
		//return;
		/*
		pcl::ModelCoefficients coefficients;
		//pcl::PointIndices inliers;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.02);

		seg.setInputCloud(objectcloud.makeShared());
		seg.segment(*inliers, coefficients);

		if (inliers->indices.size() == 0) {
			std::cerr
					<< "Could not estimate a planar model for the given dataset."
					<< std::endl;
			return;
		}

		planeparams.push_back(coefficients.values[0]);
		planeparams.push_back(coefficients.values[1]);
		planeparams.push_back(coefficients.values[2]);
		planeparams.push_back(coefficients.values[3]);
		std::cout << coefficients.values[0] << " " << coefficients.values[1] << " " << coefficients.values[2] << " " << coefficients.values[3] << " \n";
		ROS_INFO ("table coefficients ::  %f %f %f %f ", coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3]);

		// Extract the inliers
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(cloud_filtered.makeShared());
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(cloud_filtered);
*/

		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
				new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud(objectcloud.makeShared());

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
		ec.setClusterTolerance(0.01); // 2cm
		ec.setMinClusterSize(200); //df: before 700
		ec.setMaxClusterSize(5000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(objectcloud.makeShared());
		ec.extract(cluster_indices);

		for (std::vector<pcl::PointIndices>::const_iterator it =
				cluster_indices.begin(); it != cluster_indices.end(); ++it) {
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(
					new pcl::PointCloud<pcl::PointXYZRGB>);
			for (std::vector<int>::const_iterator pit = it->indices.begin();
					pit != it->indices.end(); pit++)
				cloud_cluster->points.push_back(objectcloud.points[*pit]); //*

			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			sensor_msgs::PointCloud2 cloud_out;
			pcl::toROSMsg(*cloud_cluster, cloud_out);
			clusters.push_back(cloud_out);



		}
	}

	//df new 19.3.2015
	void findObjectsOnFloorSmall(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
			std::vector<float> &planeparams,
			std::vector<sensor_msgs::PointCloud2> &clusters) {
		pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
		// Create the filtering object
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(cloud.makeShared());
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.95, 1.7);
		pass.filter(cloud_filtered);

		pcl::PointCloud<pcl::PointXYZRGB> objectcloud;
		{
			pcl::ModelCoefficients coefficients;
			//pcl::PointIndices inliers;
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			// Create the segmentation object
			pcl::SACSegmentation<pcl::PointXYZRGB> seg;
			// Optional
			seg.setOptimizeCoefficients(true);
			// Mandatory
			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.03);
			seg.setInputCloud(cloud_filtered.makeShared());
			seg.segment(*inliers, coefficients);

			if (inliers->indices.size() == 0) {
				std::cerr
						<< "Could not estimate a planar model for the given dataset."
						<< std::endl;
				return;
			}

			planeparams.push_back(coefficients.values[0]);
			planeparams.push_back(coefficients.values[1]);
			planeparams.push_back(coefficients.values[2]);
			planeparams.push_back(coefficients.values[3]);
			std::cout << coefficients.values[0] << " " << coefficients.values[1] << " " << coefficients.values[2] << " " << coefficients.values[3] << " \n";
			float distthresh = 0.017;
			int cntr = 0;
			for (size_t i = 0; i < cloud_filtered.points.size(); ++i)
			{
				float dist = 0.0;
				dist = cloud_filtered.points[i].x * coefficients.values[0]
						+ cloud_filtered.points[i].y * coefficients.values[1]
						+ cloud_filtered.points[i].z * coefficients.values[2]
						+ coefficients.values[3];
				if (dist > -distthresh)
					continue;
				{
					objectcloud.push_back(cloud_filtered.points[i]);
					cntr++;
				}

			}
			objectcloud.height = 1;
			objectcloud.width = cntr;
		}
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg (objectcloud, output);
		output.header.frame_id = "/headcam_rgb_optical_frame";
		// Publish the data (= data without detected plane => ros topic /debug)
		pub.publish (output);


		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
				new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud(objectcloud.makeShared());

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
		ec.setClusterTolerance(0.01); // 2cm
		ec.setMinClusterSize(50); //df: before 200 for normal objects
		ec.setMaxClusterSize(5000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(objectcloud.makeShared());
		ec.extract(cluster_indices);

		for (std::vector<pcl::PointIndices>::const_iterator it =
				cluster_indices.begin(); it != cluster_indices.end(); ++it) {
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(
					new pcl::PointCloud<pcl::PointXYZRGB>);
			for (std::vector<int>::const_iterator pit = it->indices.begin();
					pit != it->indices.end(); pit++)
				cloud_cluster->points.push_back(objectcloud.points[*pit]);

			ROS_INFO("==> findObjectsOnFloorSmall() -> cluster size: %f ", cloud_cluster->points.size() );

			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			sensor_msgs::PointCloud2 cloud_out;
			pcl::toROSMsg(*cloud_cluster, cloud_out);
			clusters.push_back(cloud_out);



		}
	}

	//df end 19.3.2015

	//df start 31.01.2013
	void findObjectsOnTable(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
			std::vector<float> &planeparams,
			std::vector<sensor_msgs::PointCloud2> &clusters) {
		//FILTER DATA (cut out what is not needed)
		pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
		// Create the filtering object
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(cloud.makeShared());
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.4, 1.3); //df before: pass.setFilterLimits(1.15, 1.6);
		pass.filter(cloud_filtered);

	//		pcl::PassThrough<pcl::PointXYZRGB> passx;
	//		passx.setInputCloud(cloud_filtered.makeShared());
	//		passx.setFilterFieldName("y");
	//		passx.setFilterLimits(-0.5, 1);
	//		passx.filter(cloud_filtered);

		//DETECT PLANE
		pcl::PointCloud<pcl::PointXYZRGB> objectcloud;
		{
			pcl::ModelCoefficients coefficients;
			//pcl::PointIndices inliers;
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			// Create the segmentation object
			pcl::SACSegmentation<pcl::PointXYZRGB> seg;
			// Optional
			seg.setOptimizeCoefficients(true);
			// Mandatory
			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.03);

			seg.setInputCloud(cloud_filtered.makeShared());
			seg.segment(*inliers, coefficients);

			if (inliers->indices.size() == 0) {
				std::cerr
						<< "Could not estimate a planar model for the given dataset."
						<< std::endl;
				return;
			}

			planeparams.push_back(coefficients.values[0]);
			planeparams.push_back(coefficients.values[1]);
			planeparams.push_back(coefficients.values[2]);
			planeparams.push_back(coefficients.values[3]);
			std::cout << coefficients.values[0] << " " << coefficients.values[1] << " " << coefficients.values[2] << " " << coefficients.values[3] << " \n";
			//0.0630656 0.467184 0.881908 -1.17102

			//SEPARATE POINTS FROM PLANE AND POINTS BELONGING TO OBJECTS
			//float distthresh = 0.019;	//cut out and thrown away: to avoid parts of table as clusters
			float distthresh = 0.019;	//cut out and thrown away: to avoid parts of table as clusters
			int cntr = 0;
			for (size_t i = 0; i < cloud_filtered.points.size(); ++i)
			{
				float dist = 0.0;
				dist = cloud_filtered.points[i].x * coefficients.values[0]
						+ cloud_filtered.points[i].y * coefficients.values[1]
						+ cloud_filtered.points[i].z * coefficients.values[2]
						+ coefficients.values[3];
				//if (dist > -distthresh)
				//	continue;
				if (dist > -distthresh and dist < distthresh)
					continue;
				//if (dist > distthresh)
	//				if (dist < 0.20)
	//					std::cout << dist << " " ;


				{
					objectcloud.push_back(cloud_filtered.points[i]);
					cntr++;
					}

			}
			objectcloud.height = 1;
			objectcloud.width = cntr;
		}
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg (objectcloud, output);
		output.header.frame_id = "/headcam_rgb_optical_frame";
		// Publish the data
		pub.publish (output);

		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
				new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud(objectcloud.makeShared());

		//CLUSTER OBJECTS (INPUT: CLOUD OF ALL OBJECTS)
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
		ec.setClusterTolerance(0.01); // 2cm
		ec.setMinClusterSize(200);	//df 1.3.2013, before 700
		ec.setMaxClusterSize(5000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(objectcloud.makeShared());
		ec.extract(cluster_indices);

		for (std::vector<pcl::PointIndices>::const_iterator it =
				cluster_indices.begin(); it != cluster_indices.end(); ++it) {
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(
					new pcl::PointCloud<pcl::PointXYZRGB>);
			for (std::vector<int>::const_iterator pit = it->indices.begin();
					pit != it->indices.end(); pit++)
				cloud_cluster->points.push_back(objectcloud.points[*pit]); //*

			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			sensor_msgs::PointCloud2 cloud_out;
			pcl::toROSMsg(*cloud_cluster, cloud_out);
			clusters.push_back(cloud_out);


		}
	}
	//df end
};


int main(int argc, char** argv) {
	ros::init(argc, argv, "CloudSegmenthor");
	CloudSegmenthor thor;
	ros::spin();
	return 0;
}
