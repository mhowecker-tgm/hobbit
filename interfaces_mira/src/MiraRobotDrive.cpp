
#include <interfaces_mira/MiraRobotDrive.h>

#include <transform/RigidTransform.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <interfaces_mira/MiraRobot.h>

MiraRobotDrive::MiraRobotDrive() : MiraRobotModule(std::string ("Drive")) {
}

void MiraRobotDrive::initialize() 
{

  odometry_pub_ = robot_->getRosNode().advertise<nav_msgs::Odometry>("/odom", 20);
  bumper_pub_ = robot_->getRosNode().advertise<std_msgs::Bool>("/bumper", 20);
  mileage_pub_ = robot_->getRosNode().advertise<std_msgs::Float32>("/mileage", 20);
  motorstatus_pub_ = robot_->getRosNode().advertise<mira_msgs::MotorStatus>("/motor_status", 20);
  
  robot_->getMiraAuthority().subscribe<mira::robot::Odometry2>("/robot/Odometry", //&ScitosBase::odometry_cb);
&MiraRobotDrive::odometry_data_callback, this);
  robot_->getMiraAuthority().subscribe<bool>("/robot/Bumper",
&MiraRobotDrive::bumper_data_callback, this);
  robot_->getMiraAuthority().subscribe<float>("/robot/Mileage",
&MiraRobotDrive::mileage_data_callback, this);
  robot_->getMiraAuthority().subscribe<uint8>("/robot/MotorStatus",
&MiraRobotDrive::motor_status_callback, this);

  cmd_vel_subscriber_ = robot_->getRosNode().subscribe("/cmd_vel", 1000, &MiraRobotDrive::velocity_command_callback,this);

  discrete_sub = robot_->getRosNode().subscribe("/DiscreteMotionCmd", 1, &MiraRobotDrive::discrete_motion_cmd_callback, this);
  
  reset_motor_stop_service_ = robot_->getRosNode().advertiseService("/reset_motorstop", &MiraRobotDrive::reset_motor_stop, this);
  reset_odometry_service_ = robot_->getRosNode().advertiseService("/reset_odometry", &MiraRobotDrive::reset_odometry, this);
  emergency_stop_service_ = robot_->getRosNode().advertiseService("/emergency_stop", &MiraRobotDrive::emergency_stop, this);
  enable_motors_service_ = robot_->getRosNode().advertiseService("/enable_motors", &MiraRobotDrive::enable_motors, this);
}

void MiraRobotDrive::velocity_command_callback(const geometry_msgs::Twist::ConstPtr& msg) 
{
	mira::RigidTransform<float, 2> speed(msg->linear.x, 0, msg->angular.z);
	mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot",
	"setVelocity", speed);
	r.wait();
}

void MiraRobotDrive::discrete_motion_cmd_callback(const std_msgs::String::ConstPtr& msg)
{
  char CmdBuf[32];
  char *pAux;
  unsigned int u;
  float MotionValue, a;
  uint16_t MotionType, TickSetPoint;

// Copy the received command to a local buffer for analysis.
  pAux = (char *)msg->data.c_str();
  for (u = 0; u < (32 - 1); u++)
  {
    CmdBuf[u] = pAux[u];
    if (CmdBuf[u] == 0) break;
  }
  if (u == 31) CmdBuf[u] = 0;

// ***** Command "Move" *****
  if (strncmp((const char *)(&CmdBuf[0]), "Move", 4) == 0)
  {
	// Get the desired distance 
        float MotionValue = atof(&CmdBuf[5]);
        
	// send the command
	mira::Pose2 v(MotionValue,0,0);

	float x_speed = 0.5; //FIXME
	float vMax = 0.5; //FIXME
	set_mira_param_("MainControlUnit.DriveMode", "1");
        mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot","driveDistance", v, MotionValue/x_speed, vMax);
	set_mira_param_("MainControlUnit.DriveMode", "0");

  }

// ***** Command "Turn" *****
  else if (strncmp((const char *)(&CmdBuf[0]), "Turn", 4) == 0)
  {
	// Get the desired angle and check if it's within the allowed limits - if not ignore command.
        float MotionValue = atof(&CmdBuf[5]);
	
	// send the command
        mira::Pose2 v(0,0,MotionValue);

	float rot_speed = 0.42; //FIXME
	float vMax = 0.5; //FIXME
	set_mira_param_("MainControlUnit.DriveMode", "1");
        mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot","driveDistance", v, MotionValue/rot_speed, vMax);
	set_mira_param_("MainControlUnit.DriveMode", "0");

  }
 
// ***** Command "Stop" *****
  else if (strncmp((const char *)(&CmdBuf[0]), "Stop", 4) == 0)
  {
    	//send stop command
	mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", std::string("emergencyStop"));
        r.timedWait(mira::Duration::seconds(1));
        r.get();
  }	

// ***** Unsupported command *****
  else return;
}


void MiraRobotDrive::bumper_data_callback(mira::ChannelRead<bool> data) 
{
  std_msgs::Bool out;
  out.data=data->value();	
  bumper_pub_.publish(out);
}

void MiraRobotDrive::mileage_data_callback(mira::ChannelRead<float> data) 
{
  std_msgs::Float32 out;
  out.data = data->value();	
  mileage_pub_.publish(out);
}

void MiraRobotDrive::motor_status_callback(mira::ChannelRead<uint8> data) 
{
  ros::Time time_now = ros::Time::now();
  
  mira_msgs::MotorStatus s;
  s.header.stamp=time_now;
  s.normal = (*data) & 1;
  s.motor_stopped = (*data) & (1 << 1);
  s.free_run = (*data) & (1 << 2);
  s.emergency_button_pressed = (*data) & (1 << 3);
  s.bumper_pressed = (*data) & (1 << 4);
  s.bus_error = (*data) & (1 << 5);
  s.stall_mode_flag = (*data) & (1 << 6);
  s.internal_error_flag = (*data) & (1 << 7);
  
  motorstatus_pub_.publish(s);
}


void MiraRobotDrive::odometry_data_callback(mira::ChannelRead<mira::robot::Odometry2> data ) 
{
	/// new odometry data through mira; put it out in ros
	ros::Time odom_time = ros::Time::now(); // must be something better?
	geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(
	data->value().pose.phi());

	// Publish as a nav_msgs::Odometry
	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = odom_time;
	odom_msg.header.frame_id = "/odom";
	odom_msg.child_frame_id = "/base_footprint";

	// set the position
	odom_msg.pose.pose.position.x = data->value().pose.x();
	odom_msg.pose.pose.position.y = data->value().pose.y();
	odom_msg.pose.pose.orientation = orientation;

	// set the velocity
	odom_msg.twist.twist.linear.x = data->value().velocity.x();
	odom_msg.twist.twist.angular.z = data->value().velocity.phi();

	odometry_pub_.publish(odom_msg);

	// Publish a TF
	geometry_msgs::TransformStamped odom_tf;
	odom_tf.header.stamp = odom_time;
	odom_tf.header.frame_id = "/odom";
	odom_tf.child_frame_id = "/base_footprint";

	odom_tf.transform.translation.x = data->value().pose.x();
	odom_tf.transform.translation.y = data->value().pose.y();
	odom_tf.transform.translation.z = 0.0;
	odom_tf.transform.rotation = orientation;
	// send the transform
	robot_->getTFBroadcaster().sendTransform(odom_tf);
}

bool MiraRobotDrive::reset_motor_stop(mira_msgs::ResetMotorStop::Request &req, mira_msgs::ResetMotorStop::Response &res) 
{
  // call_mira_service
  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", std::string("resetMotorStop"));
  r.timedWait(mira::Duration::seconds(1));
  r.get();

  return true;
}

bool MiraRobotDrive::reset_odometry(mira_msgs::ResetOdometry::Request &req, mira_msgs::ResetOdometry::Response &res) 
{
  // call_mira_service
  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", std::string("resetOdometry"));
  r.timedWait(mira::Duration::seconds(1));
  r.get();

  return true;
}


bool MiraRobotDrive::emergency_stop(mira_msgs::EmergencyStop::Request &req, mira_msgs::EmergencyStop::Response &res) 
{
  // call_mira_service
  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", std::string("emergencyStop"));
  r.timedWait(mira::Duration::seconds(1));
  r.get();

  return true;
}


bool MiraRobotDrive::enable_motors(mira_msgs::EnableMotors::Request &req, mira_msgs::EnableMotors::Response &res) 
{
  // call_mira_service
  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", std::string("enableMotors"),(bool)req.enable);
  r.timedWait(mira::Duration::seconds(1));
  r.get();

  return true;
}
