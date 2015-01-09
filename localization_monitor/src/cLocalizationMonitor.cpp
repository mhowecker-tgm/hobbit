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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor. Initialises member attributes and allocates resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cLocalizationMonitor::cLocalizationMonitor(int argc, char **argv) : init_argc(argc), init_argv(argv)
{
	loc_ok = true;
	uncertainty_thres = 0.15;

	high_uncertainty = false;
	matching_ok = true;

	score_thres = 0.3;  //consider lowering this threshold to account for changes in the environment, compromise needed 

	thres = 0.2;
        max_lim = 3;
	min_valid_points = 20; //FIXME

	check_cov = true;
	check_scan = true;

	is_charging = false;


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
	
	locStatePublisher = n.advertise<std_msgs::Bool>("loc_ok", 1); 

	mapTestPublisher = n.advertise<nav_msgs::OccupancyGrid>("map_test", 1); 

	get_loc_status_service = n.advertiseService("/get_loc_status", &cLocalizationMonitor::getLocStatus, this);

	batterySubs = n.subscribe<mira_msgs::BatteryState>("battery_state", 1, &cLocalizationMonitor::battery_state_callback, this);

        reset_loc_client = n.serviceClient<std_srvs::Empty>("/reset_loc"); 

/////////////////////////////////////////////////////////////////////////////////////////////////
	//load static global map, we don't want to run the map server
/////////////////////////////////////////////////////////////////////////////////////////////////
  	std::string fname("/opt/ros/hobbit_hydro/src/navigation/share/map.yaml");
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
        //create low resolution map copy
        /////////////////////////////////////////////////////////////////////////////////////////////////
/*        static_map_low_res.info.width = map_resp_.map.info.width*map_resp_.map.info.resolution/low_res;
        static_map_low_res.info.height = map_resp_.map.info.height*map_resp_.map.info.resolution/low_res;
        static_map_low_res.info.resolution = low_res;
        static_map_low_res.info.origin = map_resp_.map.info.origin;
        //Allocate space to hold the data
        static_map_low_res.data.resize(static_map_low_res.info.width * static_map_low_res.info.height);	

        static_map_low_res.info.map_load_time = ros::Time::now();
        static_map_low_res.header.frame_id = frame_id;
        static_map_low_res.header.stamp = ros::Time::now();
        ROS_INFO("Created a %d X %d map @ %.3lf m/cell",
        static_map_low_res.info.width, static_map_low_res.info.height, static_map_low_res.info.resolution);
        //meta_data_message_ = static_map_low_res.info;
        for(int i = 0; i < static_map_low_res.info.width; i++)
        {
	    for (int j = 0; j < static_map_low_res.info.height; j++)
            {
		 int index = occupancy_grid_utils::cellIndex(static_map_low_res.info, occupancy_grid_utils::Cell(i,j));
		 static_map_low_res.data[index] = occupancy_grid_utils::UNKNOWN;
	    }
        }

        for(int ind_i = 0; ind_i < static_map.info.width; ind_i++)
        {
	    for (int ind_j = 0; ind_j < static_map.info.height; ind_j++)
	    {
		 double global_x = static_map.info.origin.position.x + static_map.info.resolution*(ind_i + 0.5);
		 double global_y = static_map.info.origin.position.y + static_map.info.resolution*(ind_j + 0.5);

		 geometry_msgs::Point glob_point;
		 glob_point.x = global_x;
		 glob_point.y = global_y;
		
		 int st_index = occupancy_grid_utils::cellIndex(static_map.info, occupancy_grid_utils::Cell(ind_i,ind_j));
		 if (static_map.data[st_index] == occupancy_grid_utils::OCCUPIED)
		 {
			 int low_res_index = occupancy_grid_utils::pointIndex(static_map_low_res.info,glob_point); 

			 //if (st_index != low_res_index)
			 {
				 //std::cout << "st index " << st_index << std::endl;
				 //std::cout << "low_res_index " << low_res_index << std::endl;

			 }
			
			 if (static_map_low_res.data[low_res_index] != occupancy_grid_utils::OCCUPIED)
				 static_map_low_res.data[low_res_index] = occupancy_grid_utils::OCCUPIED;

		 }

		
	    }
        }*/
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
			 int d = thres/static_map.info.resolution;
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
	std::cout << "rel_score " << rel_score << std::endl;
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
	  std::cout << " sq_area_estimate " << sq_area_estimate << " thres " << axes_prod*axes_prod << std::endl;
          if (sq_area_estimate < axes_prod*axes_prod)
	     return true;
  	  else 
	     return false;

}

bool cLocalizationMonitor::getLocStatus(hobbit_msgs::GetState::Request  &req, hobbit_msgs::GetState::Response &res)
{
	
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


//run
void cLocalizationMonitor::Run(void)
{

	 loc_ok = (!high_uncertainty && matching_ok);
	 std_msgs::Bool loc_state;
	 loc_state.data = loc_ok;
	 locStatePublisher.publish(loc_state);

	 mapTestPublisher.publish(static_map_modified); //for visualization purposes

	 //mapTestPublisher.publish(static_map_low_res); //for visualization purposes

}

////************************************************************************************
  //
void cLocalizationMonitor::loc_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  current_pose = (*msg);

        
}

void cLocalizationMonitor::battery_state_callback(const mira_msgs::BatteryState::ConstPtr& msg)
{
  bool charging = (*msg).charging;

  /*std::cout << "*********************" << std::endl;
  std::cout << "is_charging " << is_charging << std::endl;
  std::cout << "charging " << charging << std::endl;*/

  if (!is_charging && charging)
  {
	// reset localization
	 std_srvs::Empty srv;
	 if (reset_loc_client.call(srv))
	 {
	          //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		  std::cout << "Localization reset at docking station " << std::endl;
	 }
	 else
	 {
	          ROS_DEBUG("Failed to call service get_loc_state");
	 }

  }

  is_charging = charging;

  

        
}

