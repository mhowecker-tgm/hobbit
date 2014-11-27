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

	scan.minimumRange = msg->range_min;
	//scan.maximumRange = msg->range_max; //ok for localization
	scan.maximumRange = 3.5; //FIXME //value for mapping
	
	//std::cout << "minimumRange " << scan.minimumRange << std::endl;
	//std::cout << "maximumRange " << scan.maximumRange << std::endl;

	for (int i=0; i<msg->ranges.size();i++)
	{
		scan.range[i] = msg->ranges[i];
		//std::cout << "laser loc point range " << scan.range[i] << std::endl;
		if (!std::isfinite(scan.range[i]))
		{
			scan.range[i] = scan.maximumRange;
			scan.valid[i] = mira::robot::RangeScan::Invalid;
			//std::cout << "laser loc point invalid " << std::endl;
			continue;
		}
                if (scan.range[i] < scan.minimumRange)
		{
			scan.valid[i] = mira::robot::RangeScan::Invalid;
		}
		else 
		{
			if (scan.range[i] > scan.maximumRange)
			{
				scan.valid[i] = mira::robot::RangeScan::AboveMaximum;
			}
			else
			{ 
				scan.valid[i] = mira::robot::RangeScan::Valid;
				//std::cout << "loc valid " << std::endl;
			}
		}
	}

	scan.startAngle = msg->angle_min;               
	scan.deltaAngle = msg->angle_increment;

	auto write = virtual_laser_channel_loc.write();
	write->timestamp = mira::Time::now();
	write->frameID = "/robot/LaserFrame";
	write->value() = scan;
}

void MiraVirtualLaser::virtual_laser_obs_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
	mira::robot::RangeScan scan;
	scan.range.resize(msg->ranges.size());
	scan.valid.resize(msg->ranges.size());

	scan.minimumRange = msg->range_min;
	//scan.maximumRange = msg->range_max; //ok for localization
	//scan.maximumRange = 0.99; //FIXME //value for temporary obstacle avoidance
	scan.maximumRange = 1.49;

	for (int i=0; i<msg->ranges.size();i++)
	{
		scan.range[i] = msg->ranges[i];
		//std::cout << "laser obs point range " << scan.range[i] << std::endl;
		if (!std::isfinite(scan.range[i]))
		{
			scan.range[i] = scan.maximumRange;
			scan.valid[i] = mira::robot::RangeScan::Invalid;
			//std::cout << "laser obs point invalid " << std::endl;
			continue;
		}
		if (scan.range[i] < scan.minimumRange)
		{
			scan.valid[i] = mira::robot::RangeScan::Invalid;
		}
		else 
		{
			if (scan.range[i] > scan.maximumRange)
			{
				scan.valid[i] = mira::robot::RangeScan::AboveMaximum;
			}
			else
			{ 
				scan.valid[i] = mira::robot::RangeScan::Valid;
				//std::cout << "obs valid " << std::endl;
			}
		}

	}

	scan.startAngle = msg->angle_min;               
	scan.deltaAngle = msg->angle_increment;

	//virtual_laser_channel_obs.post(scan, mira::Time::now());

        auto write = virtual_laser_channel_obs.write();
	write->frameID="/robot/TopCameraFrame";
	//write->frameID = "/robot/LaserFrame";
	write->timestamp = mira::Time::now();
	write->value() = scan;

}



