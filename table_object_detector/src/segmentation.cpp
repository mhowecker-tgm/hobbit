/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: table_object_detector.cpp 30899 2010-07-16 04:56:51Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu
\modified Walter Wohlkinger Dec 2010: segmentation, sending pointclouds with timestamps
\modified Walter Wohlkinger Jan 2010: bb, timing, 2d, camera_info, 2d-seg

@b table_object_detector detects tables and objects.
@b segments objects, calculates bounding boxes, extracts contours, etc.

 **/
#include <sstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/highgui.h>

//#include "pcl_ros/subscriber.h"
#include "pcl_ros/publisher.h"
#include "pcl/point_types.h"

#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
//#include "pcl/features/normal_3d.h"
#include "pcl/features/normal_3d.h"

#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/kdtree/organized_data.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include "pcl/segmentation/extract_clusters.h"

#include "pcl/ModelCoefficients.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/surface/convex_hull.h"


// ROS messages
#include <sensor_msgs/PointCloud2.h>

#include "recognition_msgs/Plane.h"
#include "recognition_msgs/OOBB_3D.h"
#include "recognition_msgs/OOBB_3D_2D.h"
#include "recognition_msgs/OOBB_2D.h"
#include "recognition_msgs/AABB_2D.h"
#include "recognition_msgs/Contour.h"


#include <image_geometry/pinhole_camera_model.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include "pcl/PointIndices.h"
#include "pcl/ModelCoefficients.h"

#define DEBUG_PUBLISH

class TableObjectDetector
{
  typedef pcl::PointXYZRGB Point;
  typedef pcl::KdTree<Point>::Ptr KdTreePtr;

  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  	ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    ros::Subscriber cloud_sub_;
    ros::Subscriber caminfo_sub_;
    image_transport::Subscriber img_sub_;
    image_transport::Subscriber disp_sub_;

    ros::Publisher cloud_pub_;
    ros::Publisher table_pub_;
    ros::Publisher oobb3D_pub_;
    ros::Publisher oobb3D2D_pub_;
    ros::Publisher oobb2D_pub_;
    ros::Publisher aabb2D_pub_;

    ros::Publisher contour_pub_;
    ros::Publisher diag_pub_;

    image_transport::Publisher img_cluster_pub_;
	sensor_msgs::CvBridge bridge_i;
	sensor_msgs::CvBridge bridge_d;



#ifdef DEBUG_PUBLISH
    ros::Publisher table_plane_pub_;
    ros::Publisher voxel_grid_pub_;
    ros::Publisher table_convex_hull_pub_;
#endif
    KdTreePtr normals_tree_, clusters_tree_;
    pcl::PassThrough<Point> pass_;
    pcl::VoxelGrid<Point> grid_, grid_objects_;
    pcl::NormalEstimation<Point, pcl::Normal> n3d_;
    pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
    pcl::ProjectInliers<Point> proj_;
    pcl::ProjectInliers<Point> bb_cluster_proj_;
//pcl::ConvexHull2D<Point, Point> hull_;
    pcl::ConvexHull<Point> hull_;
    pcl::ExtractPolygonalPrismData<Point> prism_;
    pcl::EuclideanClusterExtraction<Point> cluster_;

    double downsample_leaf_, downsample_leaf_objects_;
    int k_;
    double min_z_bounds_, max_z_bounds_;
    double sac_distance_threshold_;
    double normal_distance_weight_;
    double object_min_height_, object_max_height_;
    double object_cluster_tolerance_, object_cluster_min_size_;

    double img_stamp_;
    double disp_stamp_;
    double caminfo_stamp_;
    double cloud_stamp_;
    ros::Time ct;

    // The raw input image
    IplImage *img_;
    IplImage *disp_;
    // The raw input cameraInfo Msg
	image_geometry::PinholeCameraModel cam_model_;
    // The raw, input point cloud data
    pcl::PointCloud<Point>::ConstPtr cloud_;

    // The filtered and downsampled point cloud data
    pcl::PointCloud<Point>::ConstPtr cloud_filtered_, cloud_downsampled_;
    // The resultant estimated point cloud normals for \a cloud_filtered_
    pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_;
    // The vector of indices from cloud_filtered_ that represent the planar table component
    pcl::PointIndices::ConstPtr table_inliers_;
    // The model coefficients of the planar table component
    pcl::ModelCoefficients::ConstPtr table_coefficients_;
    // The set of point inliers projected on the planar table component from \a cloud_filtered_
    pcl::PointCloud<Point>::ConstPtr table_projected_;
    // The convex hull of \a table_projected_
    pcl::PointCloud<Point>::ConstPtr table_hull_;
    // The remaining of the \a cloud_filtered_ which lies inside the \a table_hull_ polygon
    pcl::PointCloud<Point>::ConstPtr cloud_objects_;
    // the single clusters
    pcl::PointCloud<Point>::ConstPtr cluster_object_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    TableObjectDetector () : nh_ ("~"), it_(nh_), img_stamp_(2), caminfo_stamp_(4), cloud_stamp_(6)
    {
      int maxbuffer = 5;
      this->cloud_sub_ = nh_.subscribe ("input", 1, &TableObjectDetector::cloudCallback, this);
      this->caminfo_sub_ = nh_.subscribe("caminfo", 1, &TableObjectDetector::caminfoCallback, this);
      this->img_sub_ = it_.subscribe("img", 1, &TableObjectDetector::imgCallback, this);
      this->disp_sub_ = it_.subscribe("disp", 1, &TableObjectDetector::dispCallback, this);

      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("clusters", maxbuffer);
      table_pub_ = nh_.advertise<recognition_msgs::Plane> ("table",1);
      oobb3D_pub_ = nh_.advertise<recognition_msgs::OOBB_3D> ("oobb_3d",maxbuffer);
      oobb3D2D_pub_ = nh_.advertise<recognition_msgs::OOBB_3D_2D> ("oobb_3d2d",maxbuffer);
      oobb2D_pub_ = nh_.advertise<recognition_msgs::OOBB_2D> ("oobb_2d",maxbuffer);
      aabb2D_pub_ = nh_.advertise<recognition_msgs::AABB_2D> ("aabb_2d",maxbuffer);
      contour_pub_ = nh_.advertise<recognition_msgs::Contour> ("contour",maxbuffer);
      diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus> ("/diagnostics",1);


      this->img_cluster_pub_ = it_.advertise("img_segment",maxbuffer);


#ifdef DEBUG_PUBLISH
      table_plane_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("table_points", 1);
      voxel_grid_pub_= nh_.advertise<sensor_msgs::PointCloud2> ("voxel_grid", 1);
      table_convex_hull_pub_= nh_.advertise<sensor_msgs::PointCloud2> ("table_outline", 1);
#endif
      // ---[ Create all PCL objects and set their parameters
      // Filtering parameters
      downsample_leaf_ = 0.02;                          // 1cm voxel size by default
      grid_.setLeafSize (downsample_leaf_, downsample_leaf_, downsample_leaf_);
//      grid_.setFilterFieldName ("z");

      min_z_bounds_ = 0.4;                            // restrict the Z dimension between 0.4m
      max_z_bounds_ = 1.6;                            // and 1.6m
      pass_.setFilterFieldName ("z");

      nh_.getParam ("min_z_bounds", min_z_bounds_);
      nh_.getParam ("max_z_bounds", max_z_bounds_);
//      grid_.setFilterLimits (min_z_bounds_, max_z_bounds_);
      pass_.setFilterLimits (min_z_bounds_, max_z_bounds_);
      grid_.setDownsampleAllData (false);
      //grid_objects_.setDownsampleAllData (false);

//      normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
      normals_tree_ = pcl::KdTreeFLANN<Point>().makeShared();

      clusters_tree_ = pcl::KdTreeFLANN<Point>().makeShared();
      clusters_tree_->setEpsilon (1);
      //clusters_tree_->setEpsilon (5);
      //tree_.setSearchWindowAsK (10);
      //tree_.setMaxDistance (0.5);

      // Normal estimation parameters
      k_ = 10;                                // 50 k-neighbors by default
      nh_.getParam ("search_k_closest", k_);
      n3d_.setKSearch (k_);
      //n3d_.setRadiusSearch (0.015);
      n3d_.setSearchMethod (normals_tree_);

      // Table model fitting parameters
      sac_distance_threshold_ = 0.02;               // 5cm
      nh_.getParam ("sac_distance_threshold", sac_distance_threshold_);
      seg_.setDistanceThreshold (sac_distance_threshold_);
      seg_.setMaxIterations (2000);

//      nh_.getParam ("normal_distance_weight", normal_distance_weight_);
      normal_distance_weight_ = 0.1;
      seg_.setNormalDistanceWeight (normal_distance_weight_);
      seg_.setOptimizeCoefficients (true);
      seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);

//      seg_.setMethodType (pcl::SAC_RANSAC);
      seg_.setMethodType(pcl::SAC_MSAC);
      seg_.setProbability (0.95);

      proj_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
      bb_cluster_proj_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
      
      // Consider objects starting at 5mm from the table and ending at 0.xm
      object_min_height_ = 0.007;
      object_max_height_ = 0.7;
      nh_.getParam ("object_min_height", object_min_height_);
      nh_.getParam ("object_max_height", object_max_height_);
      prism_.setHeightLimits (object_min_height_, object_max_height_);

      // Clustering parameters
      object_cluster_tolerance_ = 0.07;        // 5cm between two objects
      object_cluster_min_size_  = 300;         // 100 points per object cluster
      nh_.getParam ("object_cluster_tolerance", object_cluster_tolerance_);
      nh_.getParam ("object_cluster_min_size", object_cluster_min_size_);
      cluster_.setClusterTolerance (object_cluster_tolerance_);
      cluster_.setMinClusterSize (object_cluster_min_size_);
      cluster_.setSearchMethod (clusters_tree_);

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callbacks

    void cloudCallback (const sensor_msgs::PointCloud2ConstPtr &cloud2_in) // get cloud, store it
    {
    ROS_INFO ("[TableObjectDetector::cloudCallback] stamp %f received",  cloud2_in->header.stamp.toSec ());
		//ROS_INFO ("[TableObjectDetector::input_callback] PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.", cloud2_in->width * cloud2_in->height, pcl::getFieldsList (*cloud2_in).c_str (), cloud2_in->header.stamp.toSec (), cloud2_in->header.frame_id.c_str (), nh_.resolveName ("input").c_str ());
		// ---[ Convert the dataset    <-- cloud2_in    --> cloud_
		pcl::PointCloud<Point> cloud;
		pcl::fromROSMsg (*cloud2_in, cloud);
//		this->cloud_ = boost::make_shared<const pcl::PointCloud<Point> > (cloud);
		this->cloud_ = cloud.makeShared();
		this->cloud_stamp_ = cloud2_in->header.stamp.toSec();
		this->ct = cloud2_in->header.stamp;
    }

    void imgCallback(const sensor_msgs::ImageConstPtr& msg) // get img, store it
    {
    ROS_INFO ("[TableObjectDetector::imageCallback] stamp %f received",  msg->header.stamp.toSec ());
        if (bridge_i.fromImage(*msg, "bgr8"))
        {
            this->img_ = bridge_i.toIpl();
            this->img_stamp_ = msg->header.stamp.toSec();
        }
        else
        	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    void dispCallback(const sensor_msgs::ImageConstPtr& msg) // get img, store it
    {
    ROS_INFO ("[TableObjectDetector::disparityCallback] stamp %f received",  msg->header.stamp.toSec ());
        if (bridge_d.fromImage(*msg, "bgr8"))
        {
            this->disp_ = bridge_d.toIpl();
        	this->disp_stamp_ = msg->header.stamp.toSec();
        }
        else
        	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    void caminfoCallback(const sensor_msgs::CameraInfoPtr& msg) // get caminfo, store it
    {
    ROS_INFO ("[TableObjectDetector::caminfoCallback] stamp %f received",  msg->header.stamp.toSec ());
        this->cam_model_.fromCameraInfo(msg);
        this->caminfo_stamp_ = msg->header.stamp.toSec();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // here the real worker function is called if data is synched

    void run()
    {
    	diagnostic_msgs::DiagnosticStatus dia;
    	dia.name = "TOD";
    	dia.level = dia.OK;
    	int diagcntr = 5;
    	ros::Rate r(10);
    	while (ros::ok())
    	{
			if ( this->caminfo_stamp_ ==  this->img_stamp_ && this->img_stamp_ ==  this->cloud_stamp_ && this->img_stamp_ == this->disp_stamp_ )  // synched img, caminfo, cloud
				this->detect();
			else
			{
				ros::spinOnce();
				r.sleep();
	    		diagcntr--;
				if (diagcntr < 0)
				{
					this->diag_pub_.publish(dia);
					diagcntr = 5;
				}
			}
    	}
    }


    void detect ()
    {
      ros::Time t1 = ros::Time::now ();
      ros::Time t2 = ros::Time::now ();
      this->caminfo_stamp_ = 1;
	  this->img_stamp_ = 2;
	  this->cloud_stamp_ = 3;
	  this->disp_stamp_ = 4;

      // ---[ PassThroughFilter       <-- cloud_     --> cloud_filtered_   34ms
	  	pcl::PointCloud<Point> cloud_filtered;
			pass_.setInputCloud (cloud_);
			pass_.filter (cloud_filtered);
			cloud_filtered_.reset (new pcl::PointCloud<Point> (cloud_filtered));
			if ((int)cloud_filtered_->points.size () < k_)
			{
				ROS_WARN ("[TableObjectDetector::input_callback] Filtering returned %d points! Aborting.", (int)cloud_filtered_->points.size ());
				ROS_WARN ("[TableObjectDetector::input_callback] original number of points: %d ", (int)cloud_->points.size ());
				return;
			}
		//ROS_WARN ("Spent %f seconds in passthroughfilter.", (ros::Time::now () - t1).toSec ());
			t1 = ros::Time::now ();

      // ---[ Create the voxel grid    <-- cloud_filtered_     --> cloud_downsampled_  60ms
			pcl::PointCloud<Point> cloud_downsampled;
			grid_.setInputCloud (cloud_filtered_);
			grid_.filter (cloud_downsampled);
			cloud_downsampled_.reset (new pcl::PointCloud<Point> (cloud_downsampled));
			#ifdef DEBUG_PUBLISH   // Publish VOXEL GRID
				  {
					  sensor_msgs::PointCloud2 cloud_out;
					  pcl::toROSMsg (cloud_downsampled, cloud_out);
					  voxel_grid_pub_.publish (cloud_out);
				  }
			#endif
		//ROS_WARN ("Spent %f seconds in voxelizing.", (ros::Time::now () - t1).toSec ());
			t1 = ros::Time::now ();
     
      // ---[ Estimate the point normals
			pcl::PointCloud<pcl::Normal> cloud_normals;
			n3d_.setInputCloud (cloud_downsampled_);
			n3d_.compute (cloud_normals);
			cloud_normals_.reset (new pcl::PointCloud<pcl::Normal> (cloud_normals));
		//ROS_INFO ("[TableObjectDetector::input_callback] %d normals estimated.", (int)cloud_normals.points.size ());
		//ROS_WARN ("Spent %f seconds in normal calculation.", (ros::Time::now () - t1).toSec ());
			t1 = ros::Time::now ();

       // ---[ Perform segmentation
			pcl::PointIndices table_inliers;
			pcl::ModelCoefficients table_coefficients;
			seg_.setInputCloud (cloud_downsampled_);
			seg_.setInputNormals (cloud_normals_);
			seg_.segment (table_inliers, table_coefficients);
			table_inliers_.reset (new pcl::PointIndices (table_inliers));
			table_coefficients_.reset (new pcl::ModelCoefficients (table_coefficients));
//			if (table_coefficients.values.size () > 3)
//				ROS_INFO ("[TableObjectDetector::input_callback] Model found with %d inliers: [%f %f %f %f].", (int)table_inliers.indices.size (),        table_coefficients.values[0], table_coefficients.values[1], table_coefficients.values[2], table_coefficients.values[3]);
			if (table_inliers_->indices.size () == 0)
			{
				ROS_WARN ("[TableObjectDetector::input_callback] No Plane Inliers points! Aborting.");
				return;
			}

		//ROS_WARN ("Spent %f seconds in table plane calculation.", (ros::Time::now () - t1).toSec ());
			t1 = ros::Time::now ();

      // ---[ Extract the table
			pcl::PointCloud<Point> table_projected;
			//proj_.setInputCloud (cloud_filtered_);
			proj_.setInputCloud (cloud_downsampled_);
			proj_.setIndices (table_inliers_);
			proj_.setModelCoefficients (table_coefficients_);
			proj_.filter (table_projected);
			table_projected_.reset (new pcl::PointCloud<Point> (table_projected));
		//ROS_INFO ("[TableObjectDetector::input_callback] Number of projected inliers: %d.", (int)table_projected.points.size ());
			#ifdef DEBUG_PUBLISH   // Publish Table Inliers
				  {
					  sensor_msgs::PointCloud2 cloud_out;
					  pcl::toROSMsg (table_projected, cloud_out);
					  table_plane_pub_.publish (cloud_out);
				  }
			#endif
		//ROS_WARN ("Spent %f seconds in table extraction.", (ros::Time::now () - t1).toSec ());
			t1 = ros::Time::now ();
      
		// ---[ Estimate the convex hull
			pcl::PointCloud<Point> table_hull;
			hull_.setInputCloud (table_projected_);
			hull_.reconstruct (table_hull);
			table_hull_.reset (new pcl::PointCloud<Point> (table_hull));
			#ifdef DEBUG_PUBLISH   // Publish Table Outline
				  {
					  sensor_msgs::PointCloud2 cloud_out;
					  pcl::toROSMsg (table_hull, cloud_out);
					  table_convex_hull_pub_.publish (cloud_out);
				  }
			#endif
		//ROS_WARN ("Spent %f seconds in conv hull.", (ros::Time::now () - t1).toSec ());
			t1 = ros::Time::now ();


			// Compute the plane coefficients
			Eigen::Vector4f model_coefficients;
			EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;

			model_coefficients[0] = table_coefficients.values[0];
			model_coefficients[1] = table_coefficients.values[1];
			model_coefficients[2] = table_coefficients.values[2];
			model_coefficients[3] = table_coefficients.values[3];

			// Need to flip the plane normal towards the viewpoint
			Eigen::Vector4f vp (0, 0, 0, 0);
			// See if we need to flip any plane normals
			vp -= table_hull.points[0].getVector4fMap ();
			vp[3] = 0;
			// Dot product between the (viewpoint - point) and the plane normal
			float cos_theta = vp.dot (model_coefficients);
			// Flip the plane normal
			if (cos_theta < 0)
			{
			model_coefficients *= -1;
			model_coefficients[3] = 0;
			// Hessian form (D = nc . p_plane (centroid here) + p)
			model_coefficients[3] = -1 * (model_coefficients.dot (table_hull.points[0].getVector4fMap ()));
			}

			Eigen::Vector4f table_coeffs;
			//Set table_coeffs
			table_coeffs = model_coefficients;

			// publish the plane coefficients


			recognition_msgs::Plane plane;
			plane.header.stamp = this->ct;
			plane.a = table_coeffs[0];
			plane.b = table_coeffs[1];
			plane.c = table_coeffs[2];
			plane.d = table_coeffs[3];
			table_pub_.publish (plane);





      // ---[ Get the objects on top of the table
			pcl::PointIndices cloud_object_indices;
			prism_.setInputCloud (cloud_filtered_);
			prism_.setInputPlanarHull (table_hull_);
			prism_.segment (cloud_object_indices);
		//ROS_INFO ("[TableObjectDetector::input_callback] Number of object point indices: %d.", (int)cloud_object_indices.indices.size ());
		//ROS_WARN ("Spent %f seconds in extract objects.", (ros::Time::now () - t1).toSec ());
			t1 = ros::Time::now ();

      // ---[ Split the objects into Euclidean clusters
			std::vector<pcl::PointIndices> clusters;
			cluster_.setInputCloud (cloud_filtered_);
			cluster_.setIndices(boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
//			cluster_.setIndices(cloud_object_indices.makeShared());
			cluster_.extract (clusters);
			clusters.push_back(cloud_object_indices);
		ROS_INFO ("[TableObjectDetector::input_callback] Number of clusters found matching the given constraints: %d.", (int)clusters.size ());
		ROS_WARN ("Spent %f seconds in clustering.", (ros::Time::now () - t1).toSec ());
		ROS_WARN ("Spent %f seconds till here.", (ros::Time::now () - t2).toSec ());
			t1 = ros::Time::now ();

	int border = 10;
//	cv::namedWindow("img");
//	cv::namedWindow("mask");

	cv::Point2d uv;
    cv::Mat mask;
	cv::Mat tmpfg,tmpbg;
	cv::Mat imgMat_s(this->img_);
	cv::Mat dispMat_f(this->disp_);
	cv::Mat dispMat_sc = cv::Mat::zeros(imgMat_s.cols , imgMat_s.rows  , CV_8UC1);
	cv::Mat dispMat_s = cv::Mat::zeros(imgMat_s.cols , imgMat_s.rows  , CV_8UC1);
	cv::convertScaleAbs(dispMat_f,dispMat_sc);
	cv::cvtColor(dispMat_sc,dispMat_s,CV_RGB2GRAY);
//	ROS_INFO ("dispMat_s channels %d", dispMat_s.channels());
	cv::Mat imgMat = cv::Mat::zeros(imgMat_s.rows + 2*border, imgMat_s.cols + 2*border, CV_8UC1);
	cv::Mat dispMat= cv::Mat::zeros(imgMat_s.rows + 2*border, imgMat_s.cols + 2*border, CV_8UC1);

	cv::copyMakeBorder(imgMat_s, imgMat, border, border, border, border, cv::BORDER_CONSTANT, 0 );
	cv::copyMakeBorder(dispMat_s, dispMat, border, border, border, border, cv::BORDER_CONSTANT, 1 );
	mask = cv::Mat::zeros(imgMat.size(), CV_8UC1);

	cv::threshold(dispMat,dispMat,0,cv::GC_FGD,cv::THRESH_BINARY_INV);
	// correct for the shift of the disp-image wrt the rgb image, correct non-filled pixels
	cv::Rect rup,rside;
	rup.x = 0; rup.y=0; rup.width = dispMat.cols; rup.height = 40+border;
	rside.x = dispMat.cols - 60-border; rside.y=0; rside.width = 60+border; rside.height = dispMat.rows;
	cv::rectangle(dispMat,rup,0,-1); 	cv::rectangle(dispMat,rside,0,-1);



	// process each cluster and publish
//	for (size_t ii = 0; ii < clusters.size ()-1; ++ii)
	for (size_t ii = 0; ii < 1; ++ii)
	{
		pcl::PointCloud<Point> cloud_object_cluster;
		pcl::copyPointCloud (cloud_filtered, clusters[ii], cloud_object_cluster);
		sensor_msgs::PointCloud2 cloud_out;
		pcl::toROSMsg (cloud_object_cluster, cloud_out);
		cloud_out.header.stamp = this->ct;
		cloud_out.header.stamp += ros::Duration(0,ii);
//		cloud_pub_.publish (cloud_out);

		std:stringstream oss;
		oss << ii << "_clusters.pcd";
		pcl::io::savePCDFile(oss.str(),cloud_out);

		std::vector<cv::Point2i> pts;
		for (size_t i=0; i<cloud_object_cluster.size(); i++)
	    {
	    	cam_model_.project3dToPixel(cv::Point3d(cloud_object_cluster.points[i].x, cloud_object_cluster.points[i].y, cloud_object_cluster.points[i].z), uv);
	    	pts.push_back(cv::Point2i(uv.x+border,uv.y+border));
	    	cv::circle(imgMat_s,cv::Point(uv.x,uv.y),1,cv::Scalar(0,0,255),-1);
	    }
//		cv::imshow("o_img", imgMat_s);

	    dispMat.copyTo(mask);
		for (unsigned int i = 0; i < pts.size(); ++i)
		{
			cv::circle(mask,pts[i],2,cv::GC_FGD,-1);
			mask.at<uchar>(pts[i]) = cv::GC_FGD;
		}

		cv::Mat seedMat;
		mask.copyTo(seedMat);
		cv::Rect rect;
		cv::floodFill(seedMat,pts[0],cv::GC_FGD,&rect,cv::Scalar(0),cv::Scalar(0));  // connect GC_FGD + GC_PR_FGD
		cv::Rect rect_b(rect);	rect_b.x -= border; rect_b.y -= border;	rect_b.width += 2*border;	rect_b.height += 2*border;
		if (rect_b.x < 0) rect_b.x = 0;
		if (rect_b.y < 0) rect_b.y = 0;
		if (rect_b.height+rect_b.y > imgMat.rows ) rect_b.height = imgMat.rows - rect_b.y;
//		for (unsigned int i = 0; i < pts.size(); ++i)
//		{
//			cv::circle(mask,pts[i],1,cv::GC_FGD,-1);
//			mask.at<uchar>(pts[i]) = cv::GC_FGD;
//		}

	//ROS_INFO ("rect size: %d %d %d %d", rect.x, rect.y, rect.width, rect.height);
		cv::Mat mroi = mask(rect);
		cv::Mat image_small_roi = imgMat(rect_b);
		cv::Mat mask_small = cv::Mat::zeros(rect_b.height, rect_b.width, CV_8UC1);
		cv::Mat mask_roi = mask_small(cv::Rect(border,border,rect.width,rect.height));
		cv::Mat seed_roi = mask(rect);

		seed_roi.copyTo(mask_roi);
		cv::medianBlur(mask_roi,mask_roi,5);
	    cv::Mat ctmp = cv::Mat::zeros(mask_small.size(), CV_8UC1);
	    mask_small.copyTo(ctmp);

	    std::vector<std::vector<cv::Point> > contours;
//		ROS_INFO ("eroding");

	    cv::erode(mask_small,mask_small,cv::Mat());
	    cv::erode(mask_small,mask_small,cv::Mat());
	    cv::erode(mask_small,mask_small,cv::Mat());
	    cv::erode(mask_small,mask_small,cv::Mat());
	    cv::findContours(ctmp, contours, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	    mask_small.copyTo(ctmp);
		cv::drawContours(mask_small,contours,-1,cv::GC_PR_BGD,7);   // GC_PR_BGD  broad boada
		cv::drawContours(mask_small,contours,-1,cv::GC_PR_BGD,-1);  // fill interiour, as wrong marked GC_BGD mess up the results
		ctmp.copyTo(mask_small,ctmp);

//		cv::imshow("mask", mask_small*96);
		cv::threshold(ctmp,ctmp,0,1,cv::THRESH_BINARY);
	    std::vector<std::vector<cv::Point> > contours_GC_FGD;
	    cv::findContours(ctmp, contours_GC_FGD, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		cv::drawContours(mask_small,contours_GC_FGD,-1,cv::GC_PR_BGD,2);   // shrink the GC_FGD, kinect tends to broaden real results

	//ROS_INFO ("# of pixels in vector: %d", pts.size());
//		ROS_INFO ("rect size: %d %d %d %d", rect.x, rect.y, rect.width, rect.height);
//		ROS_INFO ("BGD %d FGD %d PR_BGD %d PR_FGB %d", cv::GC_BGD, cv::GC_FGD, cv::GC_PR_BGD, cv::GC_PR_FGD);

		try
		{
			cv::grabCut(image_small_roi, mask_small, rect, tmpbg, tmpfg, 2, cv::GC_INIT_WITH_MASK);
		}
		catch(...)
		{
			ROS_ERROR ("grabcut error");
			continue;
		}
		cv::Mat binMask;
	    binMask.create( mask_small.size(), CV_8UC1 );
	    binMask = mask_small & 1;
		cv::Mat res;

		image_small_roi.copyTo(res, binMask);

		cv::findContours(binMask,contours, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		int maxlen = 0;
		for (int i = 0; i<contours.size(); i++)
			if (contours[i].size() > maxlen)
				maxlen = contours[i].size();
		int i=0;
		while (contours.size() > 1)
		{
			if (contours[i].size() != maxlen)
				contours.erase (contours.begin()+i);
			else
				i++;
		}

		cv::Rect aabb_ = cv::boundingRect(cv::Mat(contours[0]));
		cv::RotatedRect oobb_ = cv::minAreaRect(cv::Mat(contours[0]));
	    cv::Point2f vtx[4];
	    oobb_.points(vtx);

//		cv::imshow("img", res);
//		cv::waitKey();

		// PUBLISHING ROS MESSAGES
//		ROS_INFO ("aabb size: %d %d %d %d", aabb_.x, aabb_.y, aabb_.width, aabb_.height);
//		ROS_INFO ("Publishing Msgs");

		recognition_msgs::AABB_2D aabb2_msg;
	    recognition_msgs::OOBB_2D oobb2_msg;
		recognition_msgs::Contour contour_msg;

	    contour_msg.header.stamp = cloud_out.header.stamp;

	    aabb2_msg.header.stamp = cloud_out.header.stamp;
	    aabb2_msg.xmin = aabb_.x + rect_b.x - border;
	    aabb2_msg.ymin = aabb_.y + rect_b.y - border;
	    aabb2_msg.xmax = aabb2_msg.xmin + aabb_.width;
	    aabb2_msg.ymax = aabb2_msg.ymin + aabb_.height;
	    this->aabb2D_pub_.publish(aabb2_msg);

	    oobb2_msg.header.stamp = cloud_out.header.stamp;
	    for (int i=0; i<4; i++)
	    {
	    	recognition_msgs::Point2D p;
	    	p.x = int(vtx[i].x);
	    	p.y = int(vtx[i].y);
		    oobb2_msg.points.push_back(p);
	    }
	    this->oobb2D_pub_.publish(oobb2_msg);

	    for (int i=0; i<contours[0].size(); i++)
	    {
	    	recognition_msgs::Point2D p;
	    	p.x = contours[0].at(i).x;
	    	p.y = contours[0].at(i).y;
	    	contour_msg.contour.push_back(p);
	    }
	    this->contour_pub_.publish(contour_msg);

		try
		{
			sensor_msgs::CvBridge bridge;
		    IplImage imgc = res(aabb_);
//		    IplImage imgc = binMask(aabb_);


		    sensor_msgs::Image::Ptr img_msg = bridge.cvToImgMsg(&imgc, "bgr8");
			img_msg->header.stamp = cloud_out.header.stamp;
			img_msg->header.frame_id = "/openni_rgb_frame";
			this->img_cluster_pub_.publish(img_msg);
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("error converting img");
		}

		cloud_pub_.publish (cloud_out);



	}
      ROS_WARN ("Spent %f seconds.", (ros::Time::now () - t2).toSec ());
   }





};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "Segmenthor");
  TableObjectDetector p;
  p.run();
  //ros::spin ();
  return (0);
}
