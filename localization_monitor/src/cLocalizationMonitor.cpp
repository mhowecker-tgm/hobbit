#include "ros/ros.h"
#include "../include/LocalizationMonitor/cLocalizationMonitor.h"

#include <Eigen/Eigenvalues> 
#include "tf/transform_datatypes.h"

#include <fstream>
#include<iostream> 

#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"

#include<libgen.h>

#include <occupancy_grid_utils/coordinate_conversions.h>

#include "angles/angles.h"

#include <hobbit_msgs/GeneralHobbitAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor. Initialises member attributes and allocates resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cLocalizationMonitor::cLocalizationMonitor(int argc, char **argv) : init_argc(argc), init_argv(argv)
{
	loc_ok = true;

	check_cov = true;
	check_scan = true;

	is_charging = false;

	initial_current_pose_received = false;
	initial_mileage_received = false;

	ros::NodeHandle nh("~");
	nh.param("uncertainty_thres", uncertainty_thres, 0.15);
	nh.param("score_thres", score_thres, 0.3); //consider lowering this threshold to account for changes in the environment, compromise needed 
	nh.param("inflation_thres", inflation_thres, 0.2); //dilate static map to allow for some error 
	nh.param("max_lim", max_lim, 3.0); //large measurements are more noisy, do not consider
	nh.param("min_valid_points", min_valid_points, 20); 

	nh.param("dis_thres", dis_thres, 0.2);
	nh.param("ang_thres", ang_thres, 15.0);
	nh.param("dis_thres_check", dis_thres_check, 1.5);
	nh.param("rate_thres", rate_thres, 0.6); 

	nh.param("apply_action", apply_action, false); 


}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Destructor. Shuts down ROS, ends the thread and released allocated
// resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cLocalizationMonitor::~cLocalizationMonitor()
{
  printf("cLocalizationMonitor::~cLocalizationMonitor(): shutting down ROS\n");
  usleep(100000);
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  usleep(100000);
  printf(" - done\n");
}

////************************************************************************************
void cLocalizationMonitor::open(ros::NodeHandle & n)
{
        current_loc_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 2, &cLocalizationMonitor::loc_pose_callback, this);
        laserSubs = n.subscribe<sensor_msgs::LaserScan>("loc_scan", 1, &cLocalizationMonitor::loc_scan_callback, this); 
	mileage_sub = n.subscribe<std_msgs::Float32>("/mileage", 2, &cLocalizationMonitor::mileage_callback, this);
	
	locStatePublisher = n.advertise<std_msgs::Bool>("loc_ok", 1); 
	mapTestPublisher = n.advertise<nav_msgs::OccupancyGrid>("map_test", 1); 

	get_loc_status_service = n.advertiseService("/get_loc_status", &cLocalizationMonitor::getLocStatus, this);

	batterySubs = n.subscribe<mira_msgs::BatteryState>("battery_state", 1, &cLocalizationMonitor::battery_state_callback, this);

        reset_loc_client = n.serviceClient<std_srvs::Empty>("/reset_loc"); 

	get_occupancy_state_service = n.advertiseService("/get_occupancy_state", &cLocalizationMonitor::getOccupancyState, this);
	 

/////////////////////////////////////////////////////////////////////////////////////////////////
	//load static global map, we don't want to run the map server
/////////////////////////////////////////////////////////////////////////////////////////////////
  	std::string fname("/opt/ros/hobbit_hydro/src/navigation/share/map.yaml"); //FIXME
	double res;

     	nav_msgs::GetMap::Response map_resp_;

        std::string mapfname = "";   
        double origin[3];
        int negate;
        double occ_th, free_th;
        bool trinary = true;
        std::string frame_id;
        ros::NodeHandle private_nh("~");
        private_nh.param("frame_id", frame_id, std::string("map"));

        std::ifstream fin(fname.c_str());
        if (fin.fail()) 
	{
          ROS_ERROR("Map %s could not be opened.", fname.c_str());
          exit(-1);
        }

        YAML::Parser parser(fin);
        YAML::Node doc;
        parser.GetNextDocument(doc);

        try { 
          doc["resolution"] >> res; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["negate"] >> negate; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a negate tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["occupied_thresh"] >> occ_th; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["free_thresh"] >> free_th; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["trinary"] >> trinary; 
        } catch (YAML::Exception) { 
          ROS_DEBUG("The map does not contain a trinary tag or it is invalid... assuming true");
          trinary = true;
        }
        try { 
          doc["origin"][0] >> origin[0]; 
          doc["origin"][1] >> origin[1]; 
          doc["origin"][2] >> origin[2]; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an origin tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["image"] >> mapfname; 
          // TODO: make this path-handling more robust
          if(mapfname.size() == 0)
          {
            ROS_ERROR("The image tag cannot be an empty string.");
            exit(-1);
          }
          if(mapfname[0] != '/')
          {
            // dirname can modify what you pass it
            char* fname_copy = strdup(fname.c_str());
            mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
            free(fname_copy);
          }
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an image tag or it is invalid.");
          exit(-1); 
        } 


        std::string image_name = mapfname;
        ROS_INFO("Loading map from image \"%s\"",image_name.c_str());
        map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, trinary);
        map_resp_.map.info.map_load_time = ros::Time::now();
        map_resp_.map.header.frame_id = frame_id;
        map_resp_.map.header.stamp = ros::Time::now();
        ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
        map_resp_.map.info.width,
        map_resp_.map.info.height,
        map_resp_.map.info.resolution);

        static_map = map_resp_.map;
        /////////////////////////////////////////////////////////////////////////////////////////////////
        //create dilated map copy to allow for some error
        /////////////////////////////////////////////////////////////////////////////////////////////////
        static_map_modified = static_map;
        std::vector<int> indices;
        for(int ind_i = 0; ind_i < static_map.info.width; ind_i++)
        {
	    for (int ind_j = 0; ind_j < static_map.info.height; ind_j++)
	    {
		 int st_index = occupancy_grid_utils::cellIndex(static_map.info, occupancy_grid_utils::Cell(ind_i,ind_j));
		 if (static_map.data[st_index] == occupancy_grid_utils::OCCUPIED)
		 {
			 int d = inflation_thres/static_map.info.resolution;
			 for (int k=ind_i-d; k<=ind_i+d; k++)
				 for (int l=ind_j-d; l<=ind_j+d; l++)
				 {       
				         if ( (k>= 0) && (k < static_map.info.width) && (l>= 0) && (l < static_map.info.height))
					 { 
					 	int index = occupancy_grid_utils::cellIndex(static_map.info, occupancy_grid_utils::Cell(k,l));
					 	indices.push_back(index);
					 }

				 }

		 }
	   
	    }
        }

	for(int i=0; i<indices.size();i++)
	{
		int index = indices[i];
		if (static_map_modified.data[index] != occupancy_grid_utils::OCCUPIED)
			static_map_modified.data[index] = occupancy_grid_utils::OCCUPIED;

	}

}


////************************************************************************************


void cLocalizationMonitor::loc_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

	scan = (*msg);

}

bool cLocalizationMonitor::checkScan()
{
	
	double current_x = current_pose.pose.pose.position.x;
	double current_y = current_pose.pose.pose.position.y;
	double current_theta = tf::getYaw(current_pose.pose.pose.orientation);

	//std::cout << "current_pose " << current_x << " " << current_y << " " << current_theta*180/M_PI << std::endl;

	double angle_i = scan.angle_min;

	double score = 0;
        int valid_points = 0;

	for (int i=0; i<scan.ranges.size();i++)
	{
		//std::cout << "angle " << angle_i*180/M_PI << std::endl;
	
		double r = scan.ranges[i];
		double ang = angle_i;
		angle_i+= scan.angle_increment;

		if (!std::isfinite(r) || r >= max_lim || r <= scan.range_min)
			continue;
		double global_x = current_x + r*cos(ang)*cos(current_theta) - r*sin(ang)*sin(current_theta);
		double global_y = current_y + r*cos(ang)*sin(current_theta) + r*sin(ang)*cos(current_theta);
		
		geometry_msgs::Point glob_point;
		glob_point.x = global_x;
		glob_point.y = global_y;
		int index = occupancy_grid_utils::pointIndex(static_map_modified.info,glob_point);

 		//int point_ok;

		if (static_map_modified.data[index] == occupancy_grid_utils::OCCUPIED)
		{
			//point_ok = 1;
			score++;
		}	
		if (static_map_modified.data[index] == occupancy_grid_utils::UNOCCUPIED)
		{
			//point_ok = -1;
			//score--;
		}

		/*std::cout << "range " << r << std::endl;
		std::cout << "global point " << global_x << " " << global_y << std::endl;
		std::cout << "point_ok " << point_ok << std::endl;
		std::cout << "********************* " << std::endl;*/

		valid_points++;

	}

	if (valid_points < min_valid_points) return true; //open space, we cannot tell if the robot is lost yet

	double rel_score = score/valid_points;
	//std::cout << "rel_score " << rel_score << std::endl;
	if (rel_score > score_thres)
		return true;
	else 
		return false;

}

bool cLocalizationMonitor::checkUncertainty()
{

	  int cov_n = 2;

   	  Eigen::MatrixXd cov;
	  cov.resize(2,2);
	  cov.setZero(2,2);

  	  cov(0,0) = current_pose.pose.covariance[0]; 
          cov(0,1) = current_pose.pose.covariance[1]; 
          cov(1,0) = current_pose.pose.covariance[6];
          cov(1,1) = current_pose.pose.covariance[7];

          Eigen::VectorXcd eigenvalues = cov.eigenvalues();
          //std::cout << "The eigenvalues of the current covariance are:" << std::endl << eigenvalues << std::endl;

          double sq_area_estimate = std::real(eigenvalues(0))*std::real(eigenvalues(1)); //the eigenvalues of the covariance must be real and non-negative

	  double axes_prod = uncertainty_thres*uncertainty_thres;
	  //std::cout << " sq_area_estimate " << sq_area_estimate << " thres " << axes_prod*axes_prod << std::endl;
          if (sq_area_estimate < axes_prod*axes_prod)
	     return true;
  	  else 
	     return false;

}

bool cLocalizationMonitor::getLocStatus(hobbit_msgs::GetState::Request  &req, hobbit_msgs::GetState::Response &res)
{
	
	std::cout << "********************* " << std::endl;
	ROS_INFO("loc_state request received");

	bool scan_ok = checkScan();
	std::cout << "scan_ok " << scan_ok << std::endl;
	bool uncertainty_ok = checkUncertainty();
	std::cout << "uncertainty_ok " << uncertainty_ok << std::endl;

	res.state = (scan_ok && uncertainty_ok);
	//res.state = scan_ok;

	ROS_INFO("sending back loc_state response");
        std::cout << "********************* " << std::endl;


	return true;

}

bool cLocalizationMonitor::getOccupancyState(hobbit_msgs::GetOccupancyState::Request  &req, hobbit_msgs::GetOccupancyState::Response &res)
{
	
	ROS_INFO("occupancy_state request received");

	geometry_msgs::Point local_point = req.local_point;

	double current_x = current_pose.pose.pose.position.x;
	double current_y = current_pose.pose.pose.position.y;
	double current_theta = tf::getYaw(current_pose.pose.pose.orientation);

	double global_x = current_x + local_point.x*cos(current_theta) - local_point.y*sin(current_theta);
	double global_y = current_y + local_point.x*sin(current_theta) + local_point.y*cos(current_theta);
		
	geometry_msgs::Point glob_point;
	glob_point.x = global_x;
	glob_point.y = global_y;
	int index = occupancy_grid_utils::pointIndex(static_map.info,glob_point);

	if (static_map.data[index] == occupancy_grid_utils::OCCUPIED)
		res.is_occupied = true;
	else 
		res.is_occupied = false;

	ROS_INFO("sending back occupancy_state response");
        std::cout << "********************* " << std::endl;


	return true;

}


//run
void cLocalizationMonitor::Run(void)
{

	 if (!initial_current_pose_received) return;

	dis_covered_sq = (current_pose.pose.pose.position.x-prev_pose.pose.pose.position.x)*(current_pose.pose.pose.position.x-prev_pose.pose.pose.position.x) + (current_pose.pose.pose.position.y-prev_pose.pose.pose.position.y)*(current_pose.pose.pose.position.y-prev_pose.pose.pose.position.y);

	 double current_orientation = tf::getYaw(current_pose.pose.pose.orientation);
	 double prev_orientation = tf::getYaw(prev_pose.pose.pose.orientation);

	 if (dis_covered_sq > dis_thres*dis_thres || fabs(angles::shortest_angular_distance(prev_orientation, current_orientation)) > ang_thres * M_PI/180)
	 {
		bool scan_ok = checkScan();
		bool uncertainty_ok = checkUncertainty();
		if (uncertainty_ok && scan_ok)
			ok_count ++;
		else
			not_ok_count++;
		prev_pose = current_pose;	
	 }

	 if (check) //based on mileage
	 {
		if (!(ok_count+not_ok_count)) return; //should never happen, but just in case
		double rate = (double)ok_count/(ok_count+not_ok_count);
		std::cout << "loc_ok_rate " << rate << std::endl; 
		loc_ok = (rate > rate_thres); //default 50%
		ok_count = 0;
		not_ok_count = 0;
		prev_pose_check = current_pose;
		check = false;

		std_msgs::Bool loc_state;
	 	loc_state.data = loc_ok;
	 	locStatePublisher.publish(loc_state);

		if (!loc_ok && apply_action)
		{
			 // create action client
  			actionlib::SimpleActionClient<hobbit_msgs::GeneralHobbitAction> ac("localization_recovery", true);

  			ROS_INFO("Waiting for action server to start.");
  			// wait for the action server to start
  			ac.waitForServer(); //will wait for infinite time

  			ROS_INFO("Action server started, sending goal.");
  			// send a goal to the action
  			hobbit_msgs::GeneralHobbitGoal goal;
  			goal.command.data = "start";
  			ac.sendGoal(goal); 

  			//wait for the action, rotation should take less than a minute
  			bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

			if (finished_before_timeout)
			{
				actionlib::SimpleClientGoalState state = ac.getState();
				 ROS_INFO("Action finished: %s",state.toString().c_str());
			    	if (state.toString() == "SUCCEEDED")
				{
					std::cout << "recovery action finished " << std::endl;
					//check current localization
					bool scan_ok = checkScan();
					bool uncertainty_ok = checkUncertainty();
					std::cout << "scan_ok " << scan_ok << std::endl;
					std::cout << "uncertainty_ok " << uncertainty_ok << std::endl;

					if (uncertainty_ok && scan_ok)
					{
						std::cout << "localization recovery succeeded " << std::endl;
						//send goal again
						return;
					}
					else
						std::cout << "localization recovery did not succeed " << std::endl;
				}
				
			}
			else
			    ROS_INFO("Rotation did not finish before the time out.");

			std::cout << "The robot is lost!!!!!!! Recovery did not succeed " << std::endl;
			//TODO publish notification
		}
		
	 }

	 //mapTestPublisher.publish(static_map_modified); //for visualization purposes

	 //mapTestPublisher.publish(static_map_low_res); //for visualization purposes

}

////************************************************************************************
  //
void cLocalizationMonitor::loc_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  current_pose = (*msg);
  if (!initial_current_pose_received)
  {
	prev_pose = current_pose;
	prev_pose_check = current_pose;
	ok_count = 0;
	not_ok_count = 0;
  	initial_current_pose_received = true;
  }

        
}

void cLocalizationMonitor::mileage_callback(const std_msgs::Float32::ConstPtr& msg)
{

   float mileage = msg->data;
   if (!initial_mileage_received)
   {
	prev_mileage = mileage;
	check = false;
	initial_mileage_received = true;
   }	
   if (mileage - prev_mileage > dis_thres_check)
   {
	prev_mileage = mileage;
	check = true;
   }

}

void cLocalizationMonitor::battery_state_callback(const mira_msgs::BatteryState::ConstPtr& msg)
{
  bool charging = (*msg).charging;

  if (!is_charging && charging)
  {
	// reset localization
	 std_srvs::Empty srv;
	 if (reset_loc_client.call(srv))
	 {
		  std::cout << "Localization reset at docking station " << std::endl;
	 }
	 else
	 {
	          ROS_DEBUG("Failed to call service get_loc_state");
	 }

  }

  is_charging = charging;
    
}




