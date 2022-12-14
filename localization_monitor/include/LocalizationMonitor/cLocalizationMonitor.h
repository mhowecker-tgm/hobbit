
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

#include "std_msgs/Float32.h"
#include <std_msgs/String.h>

#include <fstream>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class cLocalizationMonitor
{
public:

  //Constructor
  cLocalizationMonitor(int argc, char **argv);
  ~cLocalizationMonitor();
  void Run(void);
  void loc_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void loc_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void mileage_callback(const std_msgs::Float32::ConstPtr& msg);
  void open(ros::NodeHandle & n);

private:

  ros::Subscriber current_loc_sub;

  ros::Subscriber laserSubs;

  ros::Subscriber mileage_sub;

  ros::Publisher locStatePublisher;

  geometry_msgs::PoseWithCovarianceStamped current_pose;
  geometry_msgs::PoseWithCovarianceStamped prev_pose;
  geometry_msgs::PoseWithCovarianceStamped prev_pose_check;

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

  int ok_count;
  int not_ok_count;
  double rate_thres;

  double dis_covered_sq;
  double dis_thres;
  double ang_thres;
  double dis_covered_sq_check;
  double dis_thres_check;

  bool initial_current_pose_received;

  float prev_mileage;
  bool initial_mileage_received;
  bool check;

  bool apply_action;
 
  ros::ServiceClient get_last_goal_client;

  std::ofstream log_output;

  ros::ServiceClient cancel_goal_client;

  ros::ServiceServer activate_recovery_service;
  bool activateRecovery(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

  ros::ServiceServer deactivate_recovery_service;
  bool deactivateRecovery(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
  
  ros::ServiceClient check_goal_client;
  ros::ServiceClient cancel_nav_goal_client;
  ros::ServiceClient send_nav_goal_client;

  bool activate_recovery;

  std::vector<bool> check_results;
  int checks_num;
  int method; 

  bool check_updated;

  boost::thread callback_queue_thread_;
  void QueueThread();
  ros::CallbackQueue my_queue;

  std::string fname;
  std::string log_file_name;

};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
