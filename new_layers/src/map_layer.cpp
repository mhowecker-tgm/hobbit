
#include "../include/new_layers/map_layer.h"
#include<costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::MapLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

MapLayer::MapLayer() : dsrv_(NULL) {}

void MapLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);
  
  nh.param("track_unknown_space", track_unknown_space_, true);
  nh.param("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;
  //we'll subscribe to the latched topic that the map server uses
  ROS_INFO("Requesting the map...");
  map_sub_ = g_nh.subscribe(map_topic, 1, &MapLayer::incomingMap, this);
  map_received_ = false;
  has_updated_data_ = false;

  std::string topic, sensor_frame;
  nh.param("topic", topic, std::string(""));
  nh.param("sensor_frame", sensor_frame, std::string(""));
  std::string data_type;

  bool clearing, inf_is_valid;
  nh.param("clearing", clearing, true);
  nh.param("inf_is_valid", inf_is_valid, false);

  //get the raytrace range for the sensor
  double raytrace_range = 3.0;
  std::string raytrace_range_param_name;
  if (nh.searchParam("raytrace_range", raytrace_range_param_name))
  {
  	nh.getParam(raytrace_range_param_name, raytrace_range);
  }

  //get the min range
  nh.param("min_range", min_range, 0.45);

  if (clearing)
       clearing_buffers_.push_back(observation_buffers_.back());

  if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
  {
  	ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
  	throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
  }

  //create a callback for the topic
  if (data_type == "LaserScan")
  {
  	boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
   	> sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

	boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan>
   	> filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50));

	if (inf_is_valid)
	{
 		filter->registerCallback(boost::bind(&MapLayer::laserScanValidInfCallback, this, _1, observation_buffers_.back()));
	}
	else
	{
 		filter->registerCallback(boost::bind(&MapLayer::laserScanCallback, this, _1, observation_buffers_.back()));
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
		ROS_WARN("map_layer: inf_is_valid option is not applicable to PointCloud observations.");
	}

	boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud>
   	> filter(new tf::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50));
	filter->registerCallback(
   	boost::bind(&MapLayer::pointCloudCallback, this, _1, observation_buffers_.back()));

	observation_subscribers_.push_back(sub);
	observation_notifiers_.push_back(filter);
  }
  else
  {
	boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
	> sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

	if( inf_is_valid )
	{
		ROS_WARN("map_layer: inf_is_valid option is not applicable to PointCloud observations.");
	}

	boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud2>
	> filter(new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50));
	filter->registerCallback(
	boost::bind(&MapLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));

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



  ros::Rate r(10);
  while (!map_received_ && g_nh.ok())
  {
	ros::spinOnce();
	r.sleep();
  }

  ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());

  // make a copy of the original static map!!!!
  static_map_copy = new unsigned char[getSizeInCellsX()*getSizeInCellsY()];
  unsigned int span = getSizeInCellsX();
  for (int j=0; j < getSizeInCellsY(); j++)
  {
     unsigned int it = span*j;
     for (int i =0; i < getSizeInCellsX(); i++)
     {
	 static_map_copy[it] = costmap_[it];
         it++;
     }
  }


  
  if(subscribe_to_updates_)
  {
    ROS_INFO("Subscribing to updates");
    map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &MapLayer::incomingUpdate, this);
  }

  if(dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &MapLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  min_range_cells = cellDistance(min_range);
  costmap_copy = new unsigned char[4*min_range_cells];
  new_sensor_data = false;

}

void MapLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

void MapLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,const boost::shared_ptr<ObservationBuffer>& buffer)
{
   //project the laser into a point cloud
   sensor_msgs::PointCloud2 cloud;
   cloud.header = message->header;
 
   if (!tf_->waitForTransform(global_frame_, message->header.frame_id,
                   message->header.stamp + ros::Duration(message->scan_time), ros::Duration(0.25)))
   {
           ROS_DEBUG("Transform from %s to %s is not available.", message->header.frame_id.c_str(), global_frame_.c_str());
           return;
   }
 
   //project the scan into a point cloud
   try
   {
     projector_.transformLaserScanToPointCloud(global_frame_, *message, cloud, *tf_);
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

   new_sensor_data = true;
}

void MapLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message, const boost::shared_ptr<ObservationBuffer>& buffer)
{
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

   new_sensor_data = true;
}

void MapLayer::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message, const boost::shared_ptr<ObservationBuffer>& buffer)
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

   new_sensor_data = true;
}
 
void MapLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message, const boost::shared_ptr<ObservationBuffer>& buffer)
{
   //buffer the point cloud
   buffer->lock();
   buffer->bufferCloud(*message);
   buffer->unlock();

   new_sensor_data = true;

}

void MapLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

unsigned char MapLayer::interpretValue(unsigned char value)
{
  //check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void MapLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  Costmap2D* master = layered_costmap_->getCostmap();
  if (master->getSizeInCellsX() != size_x ||
      master->getSizeInCellsY() != size_y ||
      master->getResolution() != new_map->info.resolution ||
      master->getOriginX() != new_map->info.origin.position.x ||
      master->getOriginY() != new_map->info.origin.position.y ||
      !layered_costmap_->isSizeLocked())
  {
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y, true);
  }else if(size_x_ != size_x || size_y_ != size_y ||
      resolution_ != new_map->info.resolution ||
      origin_x_ != new_map->info.origin.position.x ||
      origin_y_ != new_map->info.origin.position.y){
    matchSize();
  }

  unsigned int index = 0;

  //initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;
}

void MapLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
    unsigned int di = 0;
    for (unsigned int y = 0; y < update->height ; y++)
    {
        unsigned int index_base = (update->y + y) * update->width;
        for (unsigned int x = 0; x < update->width ; x++)
        {
            unsigned int index = index_base + x + update->x;
            costmap_[index] = interpretValue( update->data[di++] );
        }
    }
    x_ = update->x;
    y_ = update->y;
    width_ = update->width;
    height_ = update->height;
    has_updated_data_ = true;
}

void MapLayer::activate()
{
    onInitialize();
}

void MapLayer::deactivate()
{
    map_sub_.shutdown();
    if (subscribe_to_updates_)
        map_update_sub_.shutdown();
}

void MapLayer::reset()
{
    deactivate();
    activate();
}

void MapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
  if ((!map_received_ || !has_updated_data_) && !new_sensor_data)
    return;

  double mx, my;
  
  mapToWorld(x_, y_, mx, my);
  *min_x = std::min(mx, *min_x);
  *min_y = std::min(my, *min_y);
  
  mapToWorld(x_ + width_, y_ + height_, mx, my);
  *max_x = std::max(mx, *max_x);
  *max_y = std::max(my, *max_y);
  
  has_updated_data_ = false;

  bool current = true;
  std::vector<Observation> clearing_observations;

  //get the clearing observations
  current = current && getClearingObservations(clearing_observations);


  if (new_sensor_data)
  {
  	//raytrace freespace
  	for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  	{
     		raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  	}
	new_sensor_data = false;
  }

}

void MapLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  /*if (!map_received_)
    return;*/
  /*if(!use_maximum_)
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
  else
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);*/

  /*unsigned char* master = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
 
  for (int j = min_j; j < max_j; j++)
  {
     unsigned int it = span*j+min_i;
     for (int i = min_i; i < max_i; i++)
     {
       if (costmap_[it] != NO_INFORMATION)
	 master[it] = costmap_[it];
       it++;
     }
  }*/

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  for (int j = min_j; j < max_j; j++)
  {
	     unsigned int it = j * span + min_i;
	     for (int i = min_i; i < max_i; i++)
	     {
		if (costmap_[it] == NO_INFORMATION)
		{
		 it++;
		 continue;
		}

		unsigned char old_cost = master_array[it];
		if (old_cost == NO_INFORMATION || old_cost < costmap_[it])
		 master_array[it] = costmap_[it];
		it++;
	     }
  }

}



void MapLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y, double* max_x, double* max_y)
{
   double ox = clearing_observation.origin_.x;
   double oy = clearing_observation.origin_.y;
   pcl::PointCloud < pcl::PointXYZ > cloud = *(clearing_observation.cloud_);
 
   //get the map coordinates of the origin of the sensor
   unsigned int x0, y0;
   if (!worldToMap(ox, oy, x0, y0))
   {
     ROS_WARN_THROTTLE(
         1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
         ox, oy);
     return;
   }
 
   //we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
   double origin_x = origin_x_, origin_y = origin_y_;
   double map_end_x = origin_x + size_x_ * resolution_;
   double map_end_y = origin_y + size_y_ * resolution_;
 
 
   //touch(ox, oy, min_x, min_y, max_x, max_y); //FIXME
   *min_x = std::min(ox, *min_x);
   *min_y = std::min(oy, *min_y);
   *max_x = std::max(ox, *max_x);
   *max_y = std::max(oy, *max_y);


   //initialize layer costmap with the static map, by copying the values

    unsigned int span = getSizeInCellsX();
    for (int j=0; j < getSizeInCellsY(); j++)
    {
     	unsigned int it = span*j;
     	for (int i =0; i < getSizeInCellsX(); i++)
     	{
	 	costmap_[it] = static_map_copy[it];
         	it++;
     	}
    }


   //copy costmap blind window before applying raytracing  !!!!!!
   //unsigned char* costmap_copy; 

   int min_x_ind = x0 - min_range_cells;
   int max_x_ind = x0 + min_range_cells;
   int min_y_ind = y0 - min_range_cells;
   int max_y_ind = y0 + min_range_cells;


   int copy_ind = 0;

   for (unsigned int iy = min_y_ind; iy < max_y_ind; iy++)
   {
	     for (unsigned int ix = min_x_ind; ix < max_x_ind; ix++)
	     {
		
	       	    int ind = getIndex(ix, iy);
	       	    costmap_copy[copy_ind] = costmap_[ind];
		    copy_ind++;

	     }
   }

 
   //for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
   for (unsigned int i = 0; i < cloud.points.size(); ++i)
   {
     double wx = cloud.points[i].x;
     double wy = cloud.points[i].y;
 
     //now we also need to make sure that the enpoint we're raytracing
     //to isn't off the costmap and scale if necessary
     double a = wx - ox;
     double b = wy - oy;

 
     //the minimum value to raytrace from is the origin
     if (wx < origin_x)
     {
       double t = (origin_x - ox) / a;
       wx = origin_x;
       wy = oy + b * t;
     }
     if (wy < origin_y)
     {
       double t = (origin_y - oy) / b;
       wx = ox + a * t;
       wy = origin_y;
     }
 
     //the maximum value to raytrace to is the end of the map
     if (wx > map_end_x)
     {
       double t = (map_end_x - ox) / a;
       wx = map_end_x - .001;
       wy = oy + b * t;
     }
     if (wy > map_end_y)
     {
       double t = (map_end_y - oy) / b;
       wx = ox + a * t;
       wy = map_end_y - .001;
     }
 
     //now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
     unsigned int x1, y1;
 
     //check for legality just in case
     if (!worldToMap(wx, wy, x1, y1))
       continue;
 
     unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
     MarkCell marker(costmap_, FREE_SPACE);
     //and finally... we can execute our trace to clear obstacles along that line
     raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);
     updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);

      

   }

   int count_copy_ind = 0;
   // add initial static map inside blind zone  //FIXME, test and check
   for (unsigned int iy = min_y_ind; iy < max_y_ind; iy++)
   {
	     for (unsigned int ix = min_x_ind; ix < max_x_ind; ix++)
	     {
		double xw, yw;
		mapToWorld(ix,iy,xw,yw);
                double a_sq = (xw-ox)*(xw-ox);
		double b_sq = (yw-oy)*(yw-oy);
		if(a_sq+b_sq < min_range*min_range)
		{
	       	    int ind = getIndex(ix, iy);
	       	    costmap_[ind] = costmap_copy[count_copy_ind];
		}
		count_copy_ind++;

	     }
   }

}


 void MapLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double* min_x, double* min_y, double* max_x, double* max_y)
{
   double dx = wx-ox, dy = wy-oy;
   double full_distance = sqrt( dx*dx+dy*dy );
   double scale = std::min(1.0, range / full_distance);
   double ex = ox + dx * scale, ey = oy + dy * scale;
   //touch(ex, ey, min_x, min_y, max_x, max_y);   //FIXME
   *min_x = std::min(ex, *min_x);
   *min_y = std::min(ey, *min_y);
   *max_x = std::max(ex, *max_x);
   *max_y = std::max(ey, *max_y);
}
 




}
