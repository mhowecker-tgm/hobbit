//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
//Modified by Paloma de la Puente
// Last update: 30.10.2014
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../include/VirtualLaser2/cVirtualLaser.h"
#include <iostream>
#include <cmath>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace VirtualLaser {
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Opens the file 'Params.txt' or 'Params_xpc2.txt' and configures
// geometry-related parameters of this class.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cVirtualLaser::LoadCfgFile(char *Filename)
{
  char TextLine[32];
  const char *Params[9] = {"SlowDown=", "MinRelHeight=", "MaxRelHeight=", "MinAngle=", "MaxAngle=",
                           "DeltaAngle=", "LinkX=", "LinkY=", "LinkZ="};
  double Data[9];
  FILE *fp;
  size_t StringLength;
  int i;
  bool bState = false;

// Open the file.
  printf("LoadCfgFile(): trying to open file '%s'\n", Filename);
  fp = fopen(Filename, "r");
  if (fp == 0)
  {
    printf(" - failed: '%s'\n\n", strerror(errno));
    return bState;
  }

// Read the file line by line.
  for (i = 0; i < 9; i++)
  {
    printf(" [%i] %s", (i + 1), Params[i]);
    if (fgets(TextLine, 32, fp) == 0)
    {
      printf("\n   - failed to read line #%i - ending\n", (i + 1));
      break;
    }
    else
    {
      StringLength = strlen(Params[i]);
      if (strncmp((const char *)TextLine, Params[i], StringLength) != 0)
      {
        printf("\n   - expected parameter not found - ending\n");
        break;
      }
      Data[i] = atof((const char *)(&TextLine[StringLength]));
      printf("%f\n", Data[i]);
    }
  }
  if (i == 9) bState = true;

// Copy the parameters to the respective variables.
  if (bState)
  {
    SlowDown = (unsigned int)Data[0];
    MinRelHeight = (float)Data[1];
    MaxRelHeight = (float)Data[2];
    MinAngle = (float)(Data[3] * (M_PI / 180.0));
    MaxAngle = (float)(Data[4] * (M_PI / 180.0));
    DeltaAngle = (float)(Data[5] * (M_PI / 180.0));
    loc_trans.transform.translation.x = Data[6];
    loc_trans.transform.translation.y = Data[7];
    loc_trans.transform.translation.z = Data[8];
    printf(" - success\n");
  }

// Close the file.
  fclose(fp);
  return bState;

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Fill the localisation and abstacle avoidance occupancy grids with
// data from the point cloud.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cVirtualLaser::FillGrids(float *pCloud)
{
  float *pNG, MinHN, MaxHN, c_x, c_y, c_z, MinAng, AngFactor;
  unsigned int x, y;
  int Index;
//  double secs = ros::Time::now().toSec();

// Clear the laser scan.  
  pNG = MaxDist;
  for (x = 0; x < SCAN_POINT_COUNT; x++) pNG[x] = 0.0f;

// Minimum and maximum height (y-coordinate, inverted sign!) to be considered.
  MinHN = -MaxRelHeight;
  MaxHN = -MinRelHeight;
  
  MinAng = MinAngle - (0.5f * DeltaAngle);
  AngFactor = 1.0f / DeltaAngle;

  int init_row = DEPTH_DATA_HEIGHT/4;  //We only consider points in the central rows of the point cloud
  int end_row = DEPTH_DATA_HEIGHT*3/4;
  int size_float = sizeof(float);
  int num_elements_row = row_step/size_float;
  //std::cout << "num " << num_elements_row << std::endl;
////////////////////////////////////////////////////////////////////////////////////////
//debugging, the original vesion only included points above the sensor's height
       /*pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	
	for (int x = (init_row-1)*num_elements_row; x < (end_row*num_elements_row-2); x+=4)
  	{
		{
			pcl::PointXYZ point(pCloud[x], pCloud[x+1], pCloud[x+2]);
			pcl_cloud.push_back(point);

		}
	}
        pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

	pcl::PointCloud<pcl::PointXYZ>::Ptr mycloudPtr (new pcl::PointCloud<pcl::PointXYZ> (pcl_cloud));
        viewer.showCloud(mycloudPtr);
	sleep (50);*/
  ////////////////////////////////////////////////////////////////////////////////////////
    for (int x = (init_row-1)*num_elements_row; x < (end_row*num_elements_row-2); x+=4)
    {

// Only process valid points, i.e. with depth > 0.0f and finite value.
      c_z = pCloud[x+2];
      if (c_z != 0.0f && std::isfinite(c_z))
      {

// Check height range.
        c_y = pCloud[x+1];
        if ((c_y >= MinHN) && (c_y <= MaxHN))
        {
          c_x = pCloud[x];
          c_y = atan2f(-c_x, c_z) - MinAng;
          Index = (int)(c_y * AngFactor);
          if ((Index >= 0) && (Index < SCAN_POINT_COUNT))
          {
            c_x = (c_x * c_x) + (c_z * c_z);
            if (pNG[Index] < c_x) pNG[Index] = c_x;
          }
        }
      }
    }
  //}  
//  secs = ros::Time::now().toSec() - secs;
//  secs *= 1000.0;
//  printf("%fms\n", secs);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ROS callback function for the reception and processing of point
// cloud data.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cVirtualLaser::cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  unsigned int u;
  float *pCloud;

// We don't need the 30Hz scan rate - slow down by factor 2 (according to config file).
  ScanSlowdown++;
  if (ScanSlowdown < SlowDown) return;
  else ScanSlowdown = 0;

// Check the resolution of the point cloud data and decide whether we can process it.
  pCloud = (float *)(&(msg->data[0]));


  row_step = msg->row_step;
  //std::cout << "data size " << msg->data.size() << std::endl;

  /*for (int i=0;i<msg->fields.size();i++)
  {
	std::cout << "field name " << msg->fields[i].name << std::endl;
	std::cout << "field count " << msg->fields[i].count << std::endl;
  }*/
  	

  if ((msg->width == DEPTH_DATA_WIDTH) && (msg->height == DEPTH_DATA_HEIGHT))
  {
    FillGrids(pCloud);
  }
  else
  {
    printf("<PointCloud2> must be (640x480) but is (%ux%u)\n", msg->width, msg->height);
    return;
  }

// Compute the virtual laser scan for localisation (farthest sensor data hits).
  for (u = 0; u < SCAN_POINT_COUNT; u++) loc_scan.ranges[u] = sqrtf(MaxDist[u]);

// Update the timestamp and publish the virtual laser scan and associated transform.
  loc_trans.header.stamp = msg->header.stamp;
  loc_scan.header.stamp = msg->header.stamp;
  (*p_loc_broadcaster).sendTransform(loc_trans);
  locScanPub.publish(loc_scan);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor. Initialises member attributes and allocates resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cVirtualLaser::cVirtualLaser(int argc, char **argv) : init_argc(argc), init_argv(argv)
{
  bRun = false;
  ScanSlowdown = 0;

  if (init_argc >= 2) bRun = LoadCfgFile(argv[1]);

  if (bRun)
  {

// Set up the localisation virtual laser scan message.
    loc_scan.header.frame_id = "/loc_link";
    loc_scan.angle_min = MinAngle;
    loc_scan.angle_max = MaxAngle;
    loc_scan.angle_increment = DeltaAngle;
    loc_scan.time_increment = 0.0f;
    loc_scan.scan_time = ((float)SlowDown / 30.0f);
    loc_scan.range_min = 0.45f;
    loc_scan.range_max = 7.5f;
    loc_scan.ranges.resize(SCAN_POINT_COUNT);
    loc_scan.intensities.resize(0);

// Set up transforms for the localisation virtual laser scan.
    loc_trans.header.frame_id = "base_link";
    loc_trans.child_frame_id = "loc_link";
    loc_trans.transform.rotation.x = 0.0;
    loc_trans.transform.rotation.y = 0.0;
    loc_trans.transform.rotation.z = 0.0;
    loc_trans.transform.rotation.w = 1.0;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Destructor. Shuts down ROS, ends the thread and released allocated
// resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cVirtualLaser::~cVirtualLaser()
{
  printf("cVirtualLaser::~cVirtualLaser(): shutting down ROS\n");
  usleep(100000);
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  usleep(100000);
  printf(" - done\n");
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// QThread main loop. Without Qt this would be main(). ROS startup,
// resource allocation and ROS main loop.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cVirtualLaser::Run(void)
{
  if (bRun)
  {

// Initialise ROS and create a node handle.
    ros::init(init_argc, init_argv, "VirtualLaser");
    ros::NodeHandle n;

// Create a subscriber for the point cloud of the depth camera and a publishers for a laser scan.
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/basecam/depth_registered/points", 1,
                                                                &cVirtualLaser::cameraCallback, this);
    locScanPub = n.advertise<sensor_msgs::LaserScan>("loc_scan", 1);

// Create a tf broadcaster for the transform laser to mobile platform.
    tf::TransformBroadcaster loc_broadcaster;
    p_loc_broadcaster = &loc_broadcaster;

// ROS main loop.
    ros::spin();
  }
  else printf("cVirtualLaser::Run(): ignoring run command due to previous error\n");
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

