#include "../include/new_layers/nav_layer_global.h"
 #include<costmap_2d/costmap_math.h>
 
 #include <pluginlib/class_list_macros.h>
 PLUGINLIB_EXPORT_CLASS(costmap_2d::NavLayerGlobal, costmap_2d::Layer)
 
 using costmap_2d::NO_INFORMATION;
 using costmap_2d::LETHAL_OBSTACLE;
 using costmap_2d::FREE_SPACE;
 
 using costmap_2d::ObservationBuffer;
 using costmap_2d::Observation;
 
 namespace costmap_2d
 {
 void NavLayerGlobal::onInitialize()
 {
   ros::NodeHandle nh("~/" + name_), g_nh;
   rolling_window_ = layered_costmap_->isRolling();
 
   bool track_unknown_space;
   nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
   if(track_unknown_space)
     default_value_ = NO_INFORMATION;
   else
     default_value_ = FREE_SPACE;
 
   NavLayerGlobal::matchSize();
   current_ = true;
   has_been_reset_ = false;
 
   global_frame_ = layered_costmap_->getGlobalFrameID();
   double transform_tolerance;
   nh.param("transform_tolerance", transform_tolerance, 0.2);
 
   std::string topics_string;
   //get the topics that we'll subscribe to from the parameter server
   nh.param("observation_sources", topics_string, std::string(""));
   ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());
 
   //now we need to split the topics based on whitespace which we can use a stringstream for
   std::stringstream ss(topics_string);
 
   std::string source;
   while (ss >> source)
   {
     ros::NodeHandle source_node(nh, source);
 
     //get the parameters for the specific topic
     double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
     std::string topic, sensor_frame, data_type;
     bool inf_is_valid, clearing, marking;
 
     source_node.param("topic", topic, source);
     source_node.param("sensor_frame", sensor_frame, std::string(""));
     source_node.param("observation_persistence", observation_keep_time, 1.0);
     source_node.param("expected_update_rate", expected_update_rate, 0.0);
     source_node.param("data_type", data_type, std::string("PointCloud"));
     source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
     source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
     source_node.param("inf_is_valid", inf_is_valid, false);
     //source_node.param("clearing", clearing, false);
     source_node.param("marking", marking, true);

     source_node.param("min_range", min_range, 0.45); //FIXME
 
     if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
     {
       ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
       throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
     }
 
     std::string raytrace_range_param_name, obstacle_range_param_name;
 
     //get the obstacle range for the sensor
     double obstacle_range = 2.5;
     if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
     {
       source_node.getParam(obstacle_range_param_name, obstacle_range);
     }
 
     //get the raytrace range for the sensor
     double raytrace_range = 3.0;
     if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
     {
       source_node.getParam(raytrace_range_param_name, raytrace_range);
     }
 
     ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
               sensor_frame.c_str());
 
     //create an observation buffer
     observation_buffers_.push_back(
         boost::shared_ptr < ObservationBuffer
             > (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                      max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
                                      sensor_frame, transform_tolerance)));
 
     //check if we'll add this buffer to our marking observation buffers
     if (marking)
       marking_buffers_.push_back(observation_buffers_.back());
 
 
     ROS_DEBUG(
         "Created an observation buffer for source %s, topic %s, global frame: %s, expected update rate: %.2f, observation persistence: %.2f",
         source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);
 
     //create a callback for the topic
     if (data_type == "LaserScan")
     {
       boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
           > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));
 
       boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan>
           > filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50));
 
       if (inf_is_valid)
       {
         filter->registerCallback(
             boost::bind(&NavLayerGlobal::laserScanValidInfCallback, this, _1, observation_buffers_.back()));
       }
       else
       {
         filter->registerCallback(
             boost::bind(&NavLayerGlobal::laserScanCallback, this, _1, observation_buffers_.back()));
       }
 
       observation_subscribers_.push_back(sub);
       observation_notifiers_.push_back(filter);
 
       observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
     }
     else if (data_type == "PointCloud")
     {
       boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud>
           > sub(new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));
 
       if( inf_is_valid )
       {
        ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
       }
 
       boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud>
           > filter(new tf::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50));
       filter->registerCallback(
           boost::bind(&NavLayerGlobal::pointCloudCallback, this, _1, observation_buffers_.back()));
 
       observation_subscribers_.push_back(sub);
       observation_notifiers_.push_back(filter);
     }
     else
     {
       boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
           > sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));
 
       if( inf_is_valid )
       {
        ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
       }
 
       boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud2>
           > filter(new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50));
       filter->registerCallback(
           boost::bind(&NavLayerGlobal::pointCloud2Callback, this, _1, observation_buffers_.back()));
 
       observation_subscribers_.push_back(sub);
       observation_notifiers_.push_back(filter);
     }
 
     if (sensor_frame != "")
     {
       std::vector < std::string > target_frames;
       target_frames.push_back(global_frame_);
       target_frames.push_back(sensor_frame);
       observation_notifiers_.back()->setTargetFrames(target_frames);
     }
 
   }
 
   setupDynamicReconfigure(nh);
   footprint_layer_.initialize( layered_costmap_, name_ + "_footprint", tf_);

   min_range_cells = cellDistance(min_range);
   costmap_copy = new unsigned char[4*min_range_cells];

   //std::cout << "min_range " << min_range << std::endl;

   int size_x_cells = getSizeInCellsX();
   int size_y_cells = getSizeInCellsY();
   std::cout << "size_x " << size_x_cells << std::endl;
   std::cout << "size_y " << size_y_cells << std::endl;

 }
 
 void NavLayerGlobal::setupDynamicReconfigure(ros::NodeHandle& nh)
 {
   dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>(nh);
   dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>::CallbackType cb = boost::bind(
       &NavLayerGlobal::reconfigureCB, this, _1, _2);
   dsrv_->setCallback(cb);
 }
 
 void NavLayerGlobal::reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level)
 {
   enabled_ = config.enabled;
   max_obstacle_height_ = config.max_obstacle_height;
   combination_method_ = config.combination_method;
 }
 
 void NavLayerGlobal::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                       const boost::shared_ptr<ObservationBuffer>& buffer)
 {
   //project the laser into a point cloud
   sensor_msgs::PointCloud2 cloud;
   cloud.header = message->header;
 
   //project the scan into a point cloud
   try
   {
     projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
   }
   catch (tf::TransformException &ex)
   {
     ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
              ex.what());
     projector_.projectLaser(*message, cloud);
   }
 
   //buffer the point cloud
   buffer->lock();
   buffer->bufferCloud(cloud);
   buffer->unlock();
 }
 
 void NavLayerGlobal::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message, 
                                               const boost::shared_ptr<ObservationBuffer>& buffer){
   // Filter positive infinities ("Inf"s) to max_range.
   float epsilon = 0.0001; // a tenth of a millimeter
   sensor_msgs::LaserScan message = *raw_message;
   for( size_t i = 0; i < message.ranges.size(); i++ )
   {
     float range = message.ranges[ i ];
     if( !std::isfinite( range ) && range > 0 )
     {
       message.ranges[ i ] = message.range_max - epsilon;
     }
   }
 
   //project the laser into a point cloud
   sensor_msgs::PointCloud2 cloud;
   cloud.header = message.header;
 
   //project the scan into a point cloud
   try
   {
     projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
   }
   catch (tf::TransformException &ex)
   {
     ROS_WARN ("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str (), ex.what ());
     projector_.projectLaser(message, cloud);
   }
 
   //buffer the point cloud
   buffer->lock();
   buffer->bufferCloud(cloud);
   buffer->unlock();
 }
 
 void NavLayerGlobal::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                                                const boost::shared_ptr<ObservationBuffer>& buffer)
 {
   sensor_msgs::PointCloud2 cloud2;
 
   if (!sensor_msgs::convertPointCloudToPointCloud2(*message, cloud2))
   {
     ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
     return;
   }
 
   //buffer the point cloud
   buffer->lock();
   buffer->bufferCloud(cloud2);
   buffer->unlock();
 }
 
 void NavLayerGlobal::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                 const boost::shared_ptr<ObservationBuffer>& buffer)
 {
   //buffer the point cloud
   buffer->lock();
   buffer->bufferCloud(*message);
   buffer->unlock();
 }
 
 void NavLayerGlobal::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
 {
   //std::cout << "updateBounds" << std::endl;
   if (rolling_window_)
   {
     std::cout << "rolling window" << std::endl;
     updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
	
   }
   if (!enabled_)
     return;
   if (has_been_reset_)
   {
     *min_x = std::min(reset_min_x_, *min_x);
     *min_y = std::min(reset_min_y_, *min_y);
     *max_x = std::max(reset_max_x_, *max_x);
     *max_y = std::max(reset_max_y_, *max_y);
     reset_min_x_ = 1e6;
     reset_min_y_ = 1e6;
     reset_max_x_ = -1e6;
     reset_max_y_ = -1e6;
     has_been_reset_ = false;
   }
 
   bool current = true;
   std::vector<Observation> observations, clearing_observations;
 
   //get the marking observations
   current = current && getMarkingObservations(observations);
 
   //update the global current status
   current_ = current;

   //Reset obstacles outside window
   resetMapOutsideWindow(robot_x, robot_y, 2*min_range, 2*min_range);  //FIXME, should be implemented in base class
 
   //place the new obstacles into a priority queue... each with a priority of zero to begin with
   for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
   {
     const Observation& obs = *it;
 
     const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);
 
     double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;
 
     for (unsigned int i = 0; i < cloud.points.size(); ++i)
     {
       double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;
 
       //if the obstacle is too high or too far away from the robot we won't add it
       if (pz > max_obstacle_height_)
       {
         ROS_DEBUG("The point is too high");
         continue;
       }
 
       //compute the squared distance from the hitpoint to the pointcloud's origin
       double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
           + (pz - obs.origin_.z) * (pz - obs.origin_.z);
 
       //if the point is far enough away... we won't consider it
       if (sq_dist >= sq_obstacle_range)
       {
         ROS_DEBUG("The point is too far away");
         continue;
       }
 
       //now we need to compute the map coordinates for the observation
       unsigned int mx, my;
       if (!worldToMap(px, py, mx, my))
       {
         ROS_DEBUG("Computing map coords failed");
         continue;
       }
 
       unsigned int index = getIndex(mx, my);
       costmap_[index] = LETHAL_OBSTACLE;
       touch(px, py, min_x, min_y, max_x, max_y);
      }
    }
 
   footprint_layer_.updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
 }
 
 void NavLayerGlobal::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
 {
   if (!enabled_)
     return;
 
   // The footprint layer clears the footprint in this NavLayerGlobal
   // before we merge this obstacle layer into the master_grid.
   //footprint_layer_.updateCosts(*this, min_i, min_j, max_i, max_j);

   unsigned char* master = master_grid.getCharMap();

   for (int it_i= 0; it_i<size_x_; it_i++)
   {
	for (int it_j= 0; it_j< size_y_; it_j++)
	{
		int ind = getIndex(it_i,it_j);
		if (master[ind]== NO_INFORMATION && costmap_[ind] == LETHAL_OBSTACLE ||master[ind] < costmap_[ind]) 
		{
			master[ind] = costmap_[ind];

		}
	}
   }

   std::cout << "************************************************ update" << std::endl;


 }
 
 void NavLayerGlobal::addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing)
 {
   if(marking)
     static_marking_observations_.push_back(obs);
 }
 
 bool NavLayerGlobal::getMarkingObservations(std::vector<Observation>& marking_observations) const
 {
   bool current = true;
   //get the marking observations
   for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
   {
     marking_buffers_[i]->lock();
     marking_buffers_[i]->getObservations(marking_observations);
     current = marking_buffers_[i]->isCurrent() && current;
     marking_buffers_[i]->unlock();
   }
   marking_observations.insert(marking_observations.end(), 
                               static_marking_observations_.begin(), static_marking_observations_.end());
   return current;
 }
 
 void NavLayerGlobal::activate()
 {
   //if we're stopped we need to re-subscribe to topics
   for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
   {
     if (observation_subscribers_[i] != NULL)
       observation_subscribers_[i]->subscribe();
   }
 
   for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
   {
     if (observation_buffers_[i])
       observation_buffers_[i]->resetLastUpdated();
   }
 }
 void NavLayerGlobal::deactivate()
 {
   for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
   {
     if (observation_subscribers_[i] != NULL)
       observation_subscribers_[i]->unsubscribe();
   }
 }
 
 void NavLayerGlobal::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double* min_x, double* min_y,
                                          double* max_x, double* max_y)
 {
   double dx = wx-ox, dy = wy-oy;
   double full_distance = ::hypot(dx, dy);
   double scale = std::min(1.0, range / full_distance);
   double ex = ox + dx * scale, ey = oy + dy * scale;
   touch(ex, ey, min_x, min_y, max_x, max_y);
 }
 
 void NavLayerGlobal::reset()
 {
     deactivate();
     resetMaps();
     current_ = true;
     has_been_reset_ = false;
     activate();
 }
 
 void NavLayerGlobal::onFootprintChanged()
 {
   footprint_layer_.onFootprintChanged();
 }

 void NavLayerGlobal::resetMapOutsideWindow(double wx, double wy, double w_size_x, double w_size_y) //FIXME, should belong to Costmap2D
 {
     ROS_ASSERT_MSG(w_size_x >= 0 && w_size_y >= 0, "You cannot specify a negative size window");
 
     double start_point_x = wx - w_size_x / 2;
     double start_point_y = wy - w_size_y / 2;
     double end_point_x = start_point_x + w_size_x;
     double end_point_y = start_point_y + w_size_y;
 
     //check start bounds
     start_point_x = std::max(origin_x_, start_point_x);
     start_point_y = std::max(origin_y_, start_point_y);
 
     //check end bounds
     end_point_x = std::min(origin_x_ + getSizeInMetersX(), end_point_x);
     end_point_y = std::min(origin_y_ + getSizeInMetersY(), end_point_y);
 
     unsigned int start_x, start_y, end_x, end_y;
 
     //check for legality just in case
     if(!worldToMap(start_point_x, start_point_y, start_x, start_y) || !worldToMap(end_point_x, end_point_y, end_x, end_y))
       return;
 
     ROS_ASSERT(end_x >= start_x && end_y >= start_y);
     unsigned int cell_size_x = end_x - start_x;
     unsigned int cell_size_y = end_y - start_y;
 
     //we need a map to store the obstacles in the window temporarily
     unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];
 
     //copy the local window in the costmap to the local map
     copyMapRegion(costmap_, start_x, start_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);
 
     //now we'll reset the costmap to free space, the static map is handled in another layer
     memset(costmap_, FREE_SPACE, size_x_ * size_y_ * sizeof(unsigned char));
 
     //now we want to copy the local map back into the costmap
     copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);

 
     //clean up
     delete[] local_map;
 }
 
 } // end namespace costmap_2d

