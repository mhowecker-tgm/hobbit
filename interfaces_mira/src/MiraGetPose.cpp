#include <interfaces_mira/MiraGetPose.h>

#include <transform/Transformer.h>

using namespace mira;

MiraGetPose::MiraGetPose() : MiraRobotModule(std::string ("MiraGetPose")) {
}

void MiraGetPose::initialize() {
        

  current_pose_pub = robot_->getRosNode().advertise<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 20);

  robot_->getMiraAuthority().subscribe<mira::PoseCov2>("/robot/PoseCov2", &MiraGetPose::loc_pose_callback, this);

}

void MiraGetPose::loc_pose_callback(mira::ChannelRead<mira::PoseCov2> data) 
{

  const mira::PoseCov2 robotPose = robot_->getMiraAuthority().getTransform<mira::PoseCov2>("/robot/RobotFrame", "/maps/MapFrame", Time::now());

  geometry_msgs::PoseWithCovarianceStamped pose_msg;

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


  //FIXME!! Provide covariance
  pose_msg.pose.covariance[0] = robotPose.cov(0,0); //robotPose.cov is a public member, the data type is  Eigen::Matrix
  pose_msg.pose.covariance[1] = robotPose.cov(0,1);
  pose_msg.pose.covariance[5] = robotPose.cov(0,2);
  pose_msg.pose.covariance[6] = robotPose.cov(0,1);
  pose_msg.pose.covariance[7] = robotPose.cov(1,1);
  pose_msg.pose.covariance[11] = robotPose.cov(1,2);
  pose_msg.pose.covariance[30] = robotPose.cov(0,2);
  pose_msg.pose.covariance[31] = robotPose.cov(1,2);
  pose_msg.pose.covariance[35] = robotPose.cov(2,2);


  //publish the pose
  current_pose_pub.publish(pose_msg);

        
}




