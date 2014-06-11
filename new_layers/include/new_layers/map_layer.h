/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2008, 2013, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
* David V. Lu!!
*********************************************************************/
#ifndef MAP_LAYER_H_
#define MAP_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>

#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/ObstaclePluginConfig.h>

namespace costmap_2d
{
class MapLayer : public Layer, public Costmap2D
{
public:
  MapLayer();
  virtual ~MapLayer(){delete[] costmap_copy; delete[] static_map_copy;}
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void matchSize();

  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                          const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);
 
  void laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& message, 
                                   const boost::shared_ptr<ObservationBuffer>& buffer);
 
  void pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                           const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);
 
  void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                            const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

  bool getClearingObservations(std::vector<costmap_2d::Observation>& clearing_observations) const;
 
  virtual void raytraceFreespace(const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y,
                            double* max_x, double* max_y);
 
  void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double* min_x, double* min_y,
                             double* max_x, double* max_y);

  laser_geometry::LaserProjection projector_; 

  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > observation_buffers_; 
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > clearing_buffers_; 
  std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;
  std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_;



private:
  /**
* @brief Callback to update the costmap's map from the map_server
* @param new_map The map to put into the costmap. The origin of the new
* map along with its size will determine what parts of the costmap's
* static map are overwritten.
*/
  void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);
  void incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update);
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  unsigned char interpretValue(unsigned char value);

  std::string global_frame_; ///< @brief The global frame for the costmap
  bool subscribe_to_updates_;
  bool map_received_;
  bool has_updated_data_;
  unsigned int x_,y_,width_,height_;
  bool track_unknown_space_;
  bool use_maximum_;
  bool trinary_costmap_;
  ros::Subscriber map_sub_, map_update_sub_;

  unsigned char lethal_threshold_, unknown_cost_value_;

  mutable boost::recursive_mutex lock_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  double min_range;
  bool new_sensor_data;

  unsigned char* static_map_copy;  

  unsigned char* costmap_copy; 
  int min_range_cells;



};
}
#endif
