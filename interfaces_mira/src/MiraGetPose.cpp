#include <interfaces_mira/MiraGetPose.h>

#include <transform/Transformer.h>

#include "pugixml.hpp"

using namespace mira;

MiraGetPose::MiraGetPose() : MiraRobotModule(std::string ("MiraGetPose")) {
}

void MiraGetPose::initialize() {
        

  current_pose_pub = robot_->getRosNode().advertise<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 20);

  robot_->getMiraAuthority().subscribe<mira::Pose2>("/robot/RobotFrame", &MiraGetPose::loc_pose_callback, this);

  reset_loc_service = robot_->getRosNode().advertiseService("/reset_loc", &MiraGetPose::resetLoc, this);

  is_docked_service = robot_->getRosNode().advertiseService("/isDocked", &MiraGetPose::atDockingStation, this);
 
  //get docking pose from stations.xml file

   pugi::xml_document doc;
   std::ifstream nameFile("/opt/ros/hobbit_hydro/src/interfaces_mira/resources/stations.xml"); //FIXME
   pugi::xml_parse_result result = doc.load(nameFile);

   if (!result) 
   {
        std::cout << "Xml stations file missing!! " << std::endl;
	ROS_INFO ("Xml stations file missing!! ");
        //assert(0);
        //return false;

	docking_pose.x() = 0;
        docking_pose.y() = 0;
        docking_pose.phi() = 0; //radians

        return;
    
   }
   
   
   pugi::xml_node root = doc.child("root");
   pugi::xml_node stations = root.child("DockingStations");
   pugi::xml_node item = stations.child("item");
   pugi::xml_node station = item.child("Station");

   docking_pose.x() = atof(station.child_value("X"));
   docking_pose.y() = atof(station.child_value("Y"));
   docking_pose.phi() = atof(station.child_value("Phi"))*M_PI/180; //radians

   /*std::cout << "docking_x " << docking_pose.x() << std::endl;
   std::cout << "docking_y " << docking_pose.y() << std::endl;
   std::cout << "docking_theta " << docking_pose.phi() *180/M_PI << std::endl;*/

   docking_pose.cov(0,0) = 0.05*0.05;
   docking_pose.cov(0,1) = 0;
   docking_pose.cov(0,2) = 0;
   docking_pose.cov(1,0) = 0;
   docking_pose.cov(1,1) = 0.05*0.05;
   docking_pose.cov(1,2) = 0;
   docking_pose.cov(2,0) = 0;
   docking_pose.cov(2,1) = 0;
   docking_pose.cov(2,2) = (5*M_PI/180) * (5*M_PI/180);
}

bool MiraGetPose::resetLoc(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) 
{
 
   std::string resetLocService = robot_->getMiraAuthority().waitForServiceInterface("ILocalization");
   auto result = robot_->getMiraAuthority().callService<void>(resetLocService, "setInitPose", docking_pose);
   result.timedWait(mira::Duration::seconds(1));
   result.get(); // causes exception if something went wrong.   
   return true;
}


void MiraGetPose::loc_pose_callback(mira::ChannelRead<mira::Pose2> data) 
{

  const mira::PoseCov2 robotPose = robot_->getMiraAuthority().getTransform<mira::PoseCov2>("/robot/RobotFrame", "/maps/MapFrame", Time::now());

  //geometry_msgs::PoseWithCovarianceStamped pose_msg;

  // parse pose fields

  //header
  ros::Time pose_time = ros::Time::now();
  pose_msg.header.stamp = pose_time;
  pose_msg.header.frame_id = "map";

  //pose
  pose_msg.pose.pose.position.x = robotPose.x(); 
  pose_msg.pose.pose.position.y = robotPose.y();
  pose_msg.pose.pose.position.z = 0;

  pose_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotPose.phi()); 

  int cov_size = 36;
  for (int i=0; i<cov_size; i++)
  {
	pose_msg.pose.covariance[i] = 0;

  }


  //Provide covariance
  pose_msg.pose.covariance[0] = robotPose.cov(0,0); //robotPose.cov is a public member, the data type is  Eigen::Matrix
  pose_msg.pose.covariance[1] = robotPose.cov(0,1);
  pose_msg.pose.covariance[5] = robotPose.cov(0,2); //may not be needed
  pose_msg.pose.covariance[6] = robotPose.cov(0,1);
  pose_msg.pose.covariance[7] = robotPose.cov(1,1);
  pose_msg.pose.covariance[11] = robotPose.cov(1,2); //may not be needed
  pose_msg.pose.covariance[30] = robotPose.cov(0,2); //may not be needed
  pose_msg.pose.covariance[31] = robotPose.cov(1,2); //may not be needed
  pose_msg.pose.covariance[35] = robotPose.cov(2,2);


  //publish the pose
  current_pose_pub.publish(pose_msg);

  //get odom->base_link
  const mira::PoseCov2 odom = robot_->getMiraAuthority().getTransform<mira::PoseCov2>("/robot/RobotFrame", "/robot/OdometryFrame", Time::now());
  tf::Transform transform1;
  transform1.setOrigin(tf::Vector3(odom.x(), odom.y(), 0));
  transform1.setRotation(tf::createQuaternionFromYaw(odom.phi()));

  //map->base_link
   tf::Transform transform2;
   transform2.setOrigin(tf::Vector3(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, 0));
   transform2.setRotation(tf::createQuaternionFromYaw(robotPose.phi()));

   tf::Transform transform3 = transform2*transform1.inverse();

   //map->odom
   robot_trans.header.frame_id = "map";
   robot_trans.child_frame_id = "odom";
   robot_trans.transform.translation.x = transform3.getOrigin().getX();
   robot_trans.transform.translation.y = transform3.getOrigin().getY();
   robot_trans.transform.translation.z = 0;
   robot_trans.transform.rotation = tf::createQuaternionMsgFromYaw(tf::getYaw(transform3.getRotation()));

   robot_trans.header.stamp = pose_msg.header.stamp;

   broadcaster.sendTransform(robot_trans);

        
}

bool MiraGetPose::atDockingStation(hobbit_msgs::GetState::Request  &req, hobbit_msgs::GetState::Response &res) 
{

   //std::cout << "isDocked service called" << std::endl;
 
   if ( (pose_msg.pose.pose.position.x - docking_pose.x() < 0.05) && (pose_msg.pose.pose.position.y - docking_pose.y() < 0.05) && (tf::getYaw(pose_msg.pose.pose.orientation) - docking_pose.phi() < 5*M_PI/180))
	res.state = true;
   else
	res.state = false;

   return true;
}

