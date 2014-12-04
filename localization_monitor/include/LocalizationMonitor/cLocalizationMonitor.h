
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef LOCALIZATION_MONITOR_HPP_
#define LOCALIZATION_MONITOR_HPP_
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Bool.h"

#include "nav_msgs/OccupancyGrid.h"

#include "hobbit_msgs/GetState.h"

class cLocalizationMonitor
{
public:

  //Constructor
  cLocalizationMonitor(int argc, char **argv);
  ~cLocalizationMonitor();
  void Run(void);
  void loc_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void loc_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void open(ros::NodeHandle & n);

private:

  ros::Subscriber current_loc_sub;

  ros::Subscriber laserSubs;

  ros::Publisher locStatePublisher;

  geometry_msgs::PoseWithCovarianceStamped current_pose;

  int init_argc;
  char **init_argv;

  bool loc_ok;
  bool high_uncertainty;
  bool matching_ok;

  double uncertainty_thres;

  double score_thres;

  nav_msgs::OccupancyGrid static_map;

  nav_msgs::OccupancyGrid static_map_low_res;

  double low_res;

  ros::Publisher mapTestPublisher;

  nav_msgs::OccupancyGrid static_map_modified;
  double thres;

//services
  ros::ServiceServer get_loc_status_service;
  bool getLocStatus(hobbit_msgs::GetState::Request  &req, hobbit_msgs::GetState::Response &res);

  sensor_msgs::LaserScan scan;

  bool checkScan();
  bool checkUncertainty();


};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
