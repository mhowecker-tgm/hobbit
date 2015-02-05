
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

#include <mira_msgs/BatteryState.h>

#include "std_srvs/Empty.h"

#include "hobbit_msgs/GetOccupancyState.h"

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

  /*nav_msgs::OccupancyGrid static_map_low_res;
  double low_res;*/

  ros::Publisher mapTestPublisher;

  nav_msgs::OccupancyGrid static_map_modified;
  double inflation_thres;

//services
  ros::ServiceServer get_loc_status_service;
  bool getLocStatus(hobbit_msgs::GetState::Request  &req, hobbit_msgs::GetState::Response &res);

  sensor_msgs::LaserScan scan;

  bool checkScan();
  bool checkUncertainty();

  double max_lim;

  bool check_cov;
  bool check_scan;

  int min_valid_points;

  bool is_charging;
  void battery_state_callback(const mira_msgs::BatteryState::ConstPtr& msg);
  ros::Subscriber batterySubs;

  ros::ServiceClient reset_loc_client;

  ros::ServiceServer get_occupancy_state_service;
  bool getOccupancyState(hobbit_msgs::GetOccupancyState::Request  &rq, hobbit_msgs::GetOccupancyState::Response &res);

  bool timeout_started;
  int ok_count;
  int not_ok_count;
  clock_t begin;
  double time_limit_secs;
  double rate_thres;

};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
