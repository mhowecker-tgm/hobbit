// WW
#include "ShapeFunctions.hpp"

int main (int argc, char** argv)
{
	ros::init (argc, argv, "Feature_SF");

//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::io::loadPCDFile("/home/walter/Download/scene_glass_upright.pcd", *cloud);
//	pcl::io::savePCDFileASCII("/home/walter/Download/scene_glass_upright_ASCII.pcd",*cloud);


//    return 0;
	SF sd;
    ros::spin();
    return 0;
}


