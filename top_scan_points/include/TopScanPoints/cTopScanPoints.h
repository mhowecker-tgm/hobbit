
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef TOP_SCAN_POINTS_HPP_
#define TOP_SCAN_POINTS_HPP_
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"


#define DEPTH_DATA_WIDTH	640
#define DEPTH_DATA_HEIGHT	480

class cTopScanPoints
{
public:

  //Constructor
  cTopScanPoints(int argc, char **argv);
  ~cTopScanPoints();
  void Run(void);
  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:

  double min_height_, max_height_;
  std::string baseFrame;  //ground plane referenced by laser
  std::string laserFrame; //pan frame simulating projected laser
  bool result;

  ros::Subscriber cloudSubscriber;

  sensor_msgs::LaserScan output;
  ros::Publisher laserPublisher;

  geometry_msgs::TransformStamped obstacle_trans;
  tf::TransformBroadcaster *p_obstacle_broadcaster;

  int init_argc;
  char **init_argv;

  //tf::TransformListener listener;
  //tf::StampedTransform transform;

};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
