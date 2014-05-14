#include <interfaces_mira/MiraGetPose.h>

#include <transform/Transformer.h>

using namespace mira;

MiraGetPose::MiraGetPose() : MiraRobotModule(std::string ("MiraGetPose")) {
}

void MiraGetPose::initialize() {
        

  current_pose_pub = robot_->getRosNode().advertise<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 20);

  robot_->getMiraAuthority().subscribe<mira::Pose2>("/robot/RobotFrame", &MiraGetPose::loc_pose_callback, this);

  
}

void MiraGetPose::loc_pose_callback(mira::ChannelRead<mira::Pose2> data) 
{

  const mira::PoseCov2 robotPose = robot_->getMiraAuthority().getTransform<mira::PoseCov2>("/robot/RobotFrame", "/maps/MapFrame", Time::now());

  geometry_msgs::PoseWithCovarianceStamped pose_msg;

  // parse pose fields

  //header
  ros::Time pose_time = ros::Time::now();
  pose_msg.header.stamp = pose_time;
  pose_msg.header.frame_id = "map";

  //std::cout << "pose_x " << robotPose.x() << std::endl;
  //std::cout << "pose_y " << robotPose.y() << std::endl;


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


  //FIXME!! Provide covariance
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


   /*robot_trans.header.frame_id = "map";
   robot_trans.child_frame_id = "base_link";
   robot_trans.transform.translation.x = pose_msg.pose.pose.position.x;
   robot_trans.transform.translation.y = pose_msg.pose.pose.position.y;
   robot_trans.transform.translation.z = 0;
   robot_trans.transform.rotation = pose_msg.pose.pose.orientation;

   robot_trans.header.stamp = pose_msg.header.stamp;
   (*p_robot_broadcaster).sendTransform(robot_trans);*/

        
}



