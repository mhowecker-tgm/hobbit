/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, 2013, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
          David V. Lu!!

* Modified by: Paloma de la Puente
*********************************************************************/
#ifndef NAV_LAYER_H_
#define NAV_LAYER_H_
#include <ros/ros.h>
//#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/ObstaclePluginConfig.h>
#include <costmap_2d/footprint_layer.h>
 
 namespace costmap_2d
 {
 class NavLayerGlobal : public Layer, public Costmap2D
 {
 public:
   NavLayerGlobal()
   {
     costmap_ = NULL; // this is the unsigned char* member of parent class Costmap2D.
   }

   virtual ~NavLayerGlobal(){delete[] costmap_copy;}
 
   virtual void onInitialize();
   virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                              double* max_y);
   virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
 
   virtual void activate();
   virtual void deactivate();
   virtual void reset();
 
   void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                          const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);
 
    void laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& message, 
                                   const boost::shared_ptr<ObservationBuffer>& buffer);
 
   void pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                           const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);
 
   void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                            const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);
 
   void setResetBounds(double mx0, double mx1, double my0, double my1)
   {
     reset_min_x_ = std::min(mx0, reset_min_x_);
     reset_max_x_ = std::max(mx1, reset_max_x_);
     reset_min_y_ = std::min(my0, reset_min_y_);
     reset_max_y_ = std::max(my1, reset_max_y_);
     has_been_reset_ = true;
   }
 
   // for testing purposes
   void addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing);
 
 protected:
 
   virtual void setupDynamicReconfigure(ros::NodeHandle& nh);
 
   bool getMarkingObservations(std::vector<costmap_2d::Observation>& marking_observations) const;
 
   bool getClearingObservations(std::vector<costmap_2d::Observation>& clearing_observations) const;
 
   virtual void raytraceFreespace(const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y,
                                  double* max_x, double* max_y);
 
   void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double* min_x, double* min_y,
                             double* max_x, double* max_y);
 
   virtual void onFootprintChanged();

   void resetMapOutsideWindow(double wx, double wy, double w_size_x, double w_size_y);
 
   std::string global_frame_; 
   double max_obstacle_height_; 
 
   laser_geometry::LaserProjection projector_; 
 
   std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_; 
   std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_; 
   std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > observation_buffers_; 
   std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > marking_buffers_; 
   std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > clearing_buffers_; 
 
   // Used only for testing purposes
   std::vector<costmap_2d::Observation> static_clearing_observations_, static_marking_observations_;
 
   bool rolling_window_;
   dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig> *dsrv_;
 
   bool has_been_reset_;
   double reset_min_x_, reset_max_x_, reset_min_y_, reset_max_y_;
 
   FootprintLayer footprint_layer_; 
   
   int combination_method_;

   double min_range;

   unsigned char* costmap_copy; 
   int min_range_cells;
 
 private:
   void reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level);
 };
}
 #endif
