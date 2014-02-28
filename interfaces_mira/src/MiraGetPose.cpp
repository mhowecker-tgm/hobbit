#include <interfaces_mira/MiraGetPose.h>

#include <transform/Transformer.h>

using namespace mira;

MiraGetPose::MiraGetPose() : MiraRobotModule(std::string ("MiraGetPose")) {
}

void MiraGetPose::initialize() {
        

  current_pose_pub = robot_->getRosNode().advertise<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 20);

  robot_->getMiraAuthority().subscribe<mira::PoseCov3>("/robot/PoseCov3", &MiraGetPose::loc_pose_callback, this);

}

void MiraGetPose::loc_pose_callback(mira::ChannelRead<mira::PoseCov3> data) 
{

  const mira::PoseCov3 robotPose = robot_->getMiraAuthority().getTransform<mira::PoseCov3>("/robot/RobotFrame", "/GlobalFrame");

  geometry_msgs::PoseWithCovarianceStamped pose_msg;

  // parse pose fields

  //header
  ros::Time pose_time = ros::Time::now();
  pose_msg.header.stamp = pose_time;
  pose_msg.header.frame_id = "map";

  //FIXME!! What is the structure of PoseCov3?? Do they use quaternions? No documentation found

  //pose
/*  pose_msg.pose.pose.position.x = ; 
  pose_msg.pose.pose.position.y = ;
  pose_msg.pose.pose.position.z = 0;

  //pose_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

  pose_msg.pose.pose.orientation.x = ; 
  pose_msg.pose.pose.orientation.y = ; 
  pose_msg.pose.pose.orientation.z = ; 
  pose_msg.pose.pose.orientation.w = ; */

  //FIXME!! Provide covariance
  //pose_msg.pose.covariance = ;


  //publish the pose
  current_pose_pub.publish(pose_msg);

        
}




