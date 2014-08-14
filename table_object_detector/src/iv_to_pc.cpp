#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <stdlib.h>     /* atof */

//PCL includes
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include "iv_io.h"

//#include <opencv2/highgui.h>

//Ros includes
#include <ros/ros.h>
#include "pcl_ros/publisher.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
//new 30.1.2014
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//#include <tuw_kuka_manipulation_msgs/RecogResult.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

#define BUFSIZE 512

//using namespace pcl;
//using namespace pcl::io;
using namespace std;

ros::Publisher new_pc_pub;
ros::Publisher pub_path_to_ivfilename;

/*
void publishPathToNewIVFile(tuw_kuka_manipulation_msgs::RecogResult recogresult){
	//this function is triggered if object was recognized. it published path to iv-filename on topic (this triggers the default funktion of this program)
	ROS_INFO("--> publishPathToNewIVFile(): Publishing path to iv-file from recognition (triggers publishing of iv-file as point cloud)");
	ROS_INFO("recogresult->xml_iv_model: ");
	cout << recogresult.xml_iv_model << endl;
	size_t start, end;
	string tmp=recogresult.xml_iv_model;
	string path_iv_file;
	string strPattern ("=");	//if this pattern is found in iv-file, it should be a line with point data
	string strPattern2 (">");
	start = tmp.find(strPattern);
	end = tmp.find(strPattern2);
	path_iv_file = tmp.substr(start+2, end-start-3);
	cout << "path_iv_file: " << path_iv_file << endl;

	// convert message to string for ROS node communication
	std_msgs::String msg;
	std::stringstream ss;
	ss << path_iv_file;
	//ROS_INFO("%s", ss.str().c_str());
	msg.data = ss.str();
	pub_path_to_ivfilename.publish(msg);

}
*/


void generateAndPublishIVFile(const std_msgs::String pathtoivfile){

	ROS_INFO("--> generateAndPublshIVFile(): Generating new point cloud from iv-file");
	string path_to_iv_file = pathtoivfile.data ;
	cout << "Generate point cloud from file: " << path_to_iv_file << endl;

	//point cloud initialization
	sensor_msgs::PointCloud2 pc2;
	PointCloud::Ptr msg (new PointCloud);



	ifstream ivfile;	//txt-file with iv-data (points)
	ivfile.open(path_to_iv_file.c_str());
	if (!ivfile){
		cout << "\n PROBLEM opening file: " << path_to_iv_file << endl;
	}

	string line;
	getline(ivfile, line);
	//can be used if files have more than one line
	int lines_to_skip = 8;
	for (unsigned int line_nr = 1; line_nr <= lines_to_skip; ++line_nr){
		getline(ivfile, line);
	}
	string strPattern ("0.");	//if this pattern is found in iv-file, it should be a line with point data
	string strPattern2 (",");
	size_t found, found2, found3;
	string p1,p2;
	string p_x_str, p_y_str, p_z_str;
	double p_x, p_y, p_z;
	while (ivfile.good())
	{
		getline(ivfile, line);
		found = line.find(strPattern);
		if (found!=string::npos){	//<==> if strPattern was detected in line <==> line includes point data
			//print => fill pc
			found2 = line.find(strPattern2);
			found3 = line.rfind(strPattern2);
			p1 = line.substr(found, found2-found);
			p2 = line.substr(found2+2,found3-found2);
			p_x_str = p1.substr(0,p1.find(" "));
			p_y_str = p1.substr(p1.find(" ")+1,p1.rfind(" ")-p1.find(" "));
			p_z_str = p1.substr(p1.rfind(" ")+1);
			p_x = atof(p_x_str.c_str());
			p_y = atof(p_y_str.c_str());
			p_z = atof(p_z_str.c_str());
			//cout << p1 << ' \n david' << p2 << '\n';
			//cout << p_x_str << " " << p_y_str << " " << p_z_str << endl;
			//msg->points.push_back (pcl::PointXYZ(p_x, p_y, p_z));
			pcl::PointXYZ pnt_tmp = pcl::PointXYZ(p_x, p_y, p_z);
			msg->points.push_back (pnt_tmp);
		} else {
			break;
		}
	}

	ivfile.close();

	pc2.header.frame_id = "lwr_";
	pc2.header.stamp = ros::Time::now ();
	msg->header = pcl_conversions::toPCL(pc2.header);

	new_pc_pub.publish(msg);


	return;
}



int main(int argc, char **argv){

  //Subscribe to topic were path to iv file is published
  ros::init (argc, argv, "iv_to_pc");
  ros::NodeHandle nh;
  new_pc_pub = nh.advertise<PointCloud> ("/SS/points2_object_in_rcs", 1);
  //iv_filename_pub = nh.advertise<std_msgs::String>("/pc_to_iv/generated_ivfilename",10);
  //ros::Duration(0.3).sleep(); //needed?
  pub_path_to_ivfilename = nh.advertise<std_msgs::String> ("/path_to_ivfilename", 1);
  ros::Subscriber sub = nh.subscribe("/path_to_ivfilename", 1, generateAndPublishIVFile);
  //ros::Subscriber sub_add_object_from_recognition = nh.subscribe("/segmentation_recognition/add_object", 1, publishPathToNewIVFile); //Aitor recognition
  //ros::Subscriber sub = nh.subscribe("/SS/points2_object_in_rcs", 10, generateInventor);
  ROS_INFO("Starting node: iv_to_pc");
  ros::spin();
  return 0;
}
