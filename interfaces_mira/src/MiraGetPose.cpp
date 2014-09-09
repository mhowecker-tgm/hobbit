#include <interfaces_mira/MiraGetPose.h>

#include <transform/Transformer.h>



using namespace mira;

MiraGetPose::MiraGetPose() : MiraRobotModule(std::string ("MiraGetPose")) {
}

void MiraGetPose::initialize() {
        

  current_pose_pub = robot_->getRosNode().advertise<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 20);

  robot_->getMiraAuthority().subscribe<mira::Pose2>("/robot/RobotFrame", &MiraGetPose::loc_pose_callback, this);

  //new tf
  last_odom_subscriber = robot_->getRosNode().subscribe("/odom", 1000, & MiraGetPose::last_odom_callback, this);

  
}

void MiraGetPose::last_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
	odom_x = msg->pose.pose.position.x;
	odom_y = msg->pose.pose.position.y;
	odom_th = tf::getYaw(msg->pose.pose.orientation);
        
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
  //std::cout << "mira orientation " << robotPose.phi() << std::endl; 

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

   //old tf msg
/*   robot_trans_msg.header.frame_id = "map";
   robot_trans_msg.child_frame_id = "base_link";
   robot_trans_msg.transform.translation.x = pose_msg.pose.pose.position.x;
   robot_trans_msg.transform.translation.y = pose_msg.pose.pose.position.y;
   robot_trans_msg.transform.translation.z = 0;
   robot_trans_msg.transform.rotation = pose_msg.pose.pose.orientation;
   robot_trans_msg.header.stamp = pose_msg.header.stamp;*/

   //loc tf
   tf::StampedTransform robot_trans;
   robot_trans.setOrigin(tf::Vector3(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, 0));
   robot_trans.setRotation(tf::Quaternion(pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w));

   //odom tf
   tf::StampedTransform odom_trans;
   odom_trans.setOrigin(tf::Vector3(odom_x, odom_y, 0));
   odom_trans.setRotation(tf::createQuaternionFromRPY(0, 0, odom_th));

   //rel tf
   tf::StampedTransform rel_trans;
   rel_trans.setData(robot_trans*odom_trans.inverse());
   rel_trans.stamp_ = pose_msg.header.stamp;
   
   geometry_msgs::TransformStamped rel_trans_msg;
   rel_trans_msg.header.frame_id = "map";
   rel_trans_msg.child_frame_id = "odom";
   double rel_x = rel_trans.getOrigin()[0];
   rel_trans_msg.transform.translation.x = rel_x;
   double rel_y = rel_trans.getOrigin()[1];
   rel_trans_msg.transform.translation.y = rel_y;
   rel_trans_msg.transform.translation.z = 0;
   rel_trans_msg.transform.rotation.x = rel_trans.getRotation().x();
   rel_trans_msg.transform.rotation.y = rel_trans.getRotation().y(); 
   rel_trans_msg.transform.rotation.z = rel_trans.getRotation().z();
   rel_trans_msg.transform.rotation.w = rel_trans.getRotation().w();
   rel_trans_msg.header.stamp = pose_msg.header.stamp;

   tf::TransformBroadcaster broadcaster;
   p_robot_broadcaster = &broadcaster;
   (*p_robot_broadcaster).sendTransform(rel_trans_msg);

        
}



