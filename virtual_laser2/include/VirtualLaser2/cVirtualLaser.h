//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 20.6.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef VIRTUAL_LASER_HPP_
#define VIRTUAL_LASER_HPP_
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <new>
#include <stdint.h>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_broadcaster.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define DEPTH_DATA_WIDTH	640
#define DEPTH_DATA_HEIGHT	480
#define SCAN_POINT_COUNT    121
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace VirtualLaser {
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cVirtualLaser
{

  public:
    cVirtualLaser(int argc, char **argv);
    ~cVirtualLaser();
    void Run(void);

  private:
    float MaxDist[SCAN_POINT_COUNT];
    float MinAngle, MaxAngle, DeltaAngle;
    int init_argc;
    char **init_argv;
    float MinRelHeight, MaxRelHeight;
    unsigned int ScanSlowdown, SlowDown;
    bool bRun;

    sensor_msgs::LaserScan loc_scan;
    ros::Publisher locScanPub;
    geometry_msgs::TransformStamped loc_trans;
    tf::TransformBroadcaster *p_loc_broadcaster;

    bool LoadCfgFile(char *Filename);
    void FillGrids(float *pCloud);
    void cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    int row_step;
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

