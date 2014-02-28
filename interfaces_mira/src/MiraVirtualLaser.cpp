#include <interfaces_mira/MiraVirtualLaser.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

//#include <interfaces_mira/MiraRobot.h>

MiraVirtualLaser::MiraVirtualLaser() : MiraRobotModule(std::string ("VirtualLasers")) {
}

void MiraVirtualLaser::initialize() {
        
  virtual_laser_loc_subscriber = robot_->getRosNode().subscribe("/loc_scan", 1000, &MiraVirtualLaser::virtual_laser_loc_callback, this);
  virtual_laser_obs_subscriber = robot_->getRosNode().subscribe("/obstacle_scan", 1000, &MiraVirtualLaser::virtual_laser_obs_callback, this);


  virtual_laser_channel_loc = robot_->getMiraAuthority().publish<mira::robot::RangeScan>("/VirtualLaserLoc");
  virtual_laser_channel_obs = robot_->getMiraAuthority().publish<mira::robot::RangeScan>("/VirtualLaserObs");

}

void MiraVirtualLaser::virtual_laser_loc_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
        mira::robot::RangeScan scan;
	scan.range.resize(msg->ranges.size());
	scan.valid.resize(msg->ranges.size());

	for (int i=0; i<msg->ranges.size();i++)
	{
		scan.range[i] = msg->ranges[i];
		scan.valid[i] = 1;
	}

	scan.minimumRange = msg->range_min;
	//scan.maximumRange = msg->range_max;

	scan.startAngle = msg->angle_min;               
	scan.deltaAngle = msg->angle_increment;

	virtual_laser_channel_loc.post(scan, mira::Time::now());
}

void MiraVirtualLaser::virtual_laser_obs_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
	mira::robot::RangeScan scan;
	scan.range.resize(msg->ranges.size());
	scan.valid.resize(msg->ranges.size());

	for (int i=0; i<msg->ranges.size();i++)
	{
		scan.range[i] = msg->ranges[i];
		scan.valid[i] = 1;
	}

	scan.minimumRange = msg->range_min;
	//scan.maximumRange = msg->range_max;

	scan.startAngle = msg->angle_min;               
	scan.deltaAngle = msg->angle_increment;

	virtual_laser_channel_obs.post(scan, mira::Time::now());

}



