//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 15.3.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "math.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "../include/hobbit_platform/cException.h"
#include "../include/hobbit_platform/cHobbitMCU.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define NODE_DM_MIN_DIST	0.03f
#define NODE_DM_MAX_DIST	1.51f
#define NODE_DM_MIN_ANG		3.0f
#define NODE_DM_MAX_ANG		180.1f
#define NODE_DM_TICKS_M		15373.0f
#define NODE_DM_ROT_CONST	(4800.0f / 90.0f)
#define NODE_DM_ROT_OFFSET	0.25f
#define NODE_DM_COM_OFFSET	75.0
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define NODE_LAST_CMD_NONE	0
#define NODE_LAST_CMD_DM_MOVE	1
#define NODE_LAST_CMD_DM_TURN	2
#define NODE_LAST_CMD_CM_MOVE	3
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// MotionState
// - 0: idle
// - 1: discrete motion active
// - 2: discrete motion waiting for standstill
// - 3: speed command active
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tf::TransformBroadcaster *p_odom_broadcaster;
ros::Publisher *p_odom_pub;
ros::Publisher *p_state_pub;
ros::Time current_time;
geometry_msgs::Quaternion odom_quat;
geometry_msgs::TransformStamped odom_trans;
nav_msgs::Odometry odom_odom;
std_msgs::String stateString;
cHobbitMCU *pMCU;
bool bMCUready;
uint16_t MotionState;
int LastCommand;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ROS callback function for incoming twist messages.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double v, w;

// If the class 'cHobbitMCU' is instantiated and ready, it directly gets the desired twist.
  v = msg->linear.x;
  w = msg->angular.z;

// The robot must be in idle mode or executing speed commands.
  if (bMCUready && (pMCU != 0) && ((MotionState == 0) || (MotionState == 3)))
  {
    LastCommand = NODE_LAST_CMD_CM_MOVE;
    pMCU->SetSpeed(v, w);
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Callback function for commands sent to this node via the topic
// "/DiscreteMotion" which is of the type "std_msgs/String"
// Valid commands are:
// - "Move <distance>" - [meters]; pos->forward, neg->backwards
// - "Turn <angle>" - [degrees]; pos->ccw, neg->cw
// - "Stop" - sends a motion command with all zero to the platform
// Invalid commands will be ignored.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void discreteCallback(const std_msgs::String::ConstPtr& msg)
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

// ***** Command "Move" - must be idle state *****
  if (strncmp((const char *)(&CmdBuf[0]), "Move", 4) == 0)
  {

// The robot must be in idle mode.
    if (MotionState != 0) return;
    else
    {

// Get the desired distance and check if it's within the allowed limits.
      MotionValue = atof(&CmdBuf[5]);
      if (fabsf(MotionValue) < NODE_DM_MIN_DIST) return;
      else if (fabsf(MotionValue) > NODE_DM_MAX_DIST) return;
      else
      {

// Compute the required encoder tick count and motion type.
        a = (fabsf(MotionValue) * NODE_DM_TICKS_M) - NODE_DM_COM_OFFSET;
        TickSetPoint = (uint16_t)roundf(a);
        if (MotionValue > 0.0f) MotionType = 0;
        else MotionType = 6;

// Send discrete motion command.
        LastCommand = NODE_LAST_CMD_DM_MOVE;
        pMCU->DiscreteMotion(MotionType, TickSetPoint);
      }
    }
  }

// ***** Command "Turn" - must be idle state *****
  else if (strncmp((const char *)(&CmdBuf[0]), "Turn", 4) == 0)
  {

// The robot must be in idle mode.
    if (MotionState != 0) return;
    else
    {

// Get the desired angle and check if it's within the allowed limits.
      MotionValue = atof(&CmdBuf[5]);
      if (fabsf(MotionValue) < NODE_DM_MIN_ANG) return;
      else if (fabsf(MotionValue) > NODE_DM_MAX_ANG) return;
      else
      {

// Compute the required encoder tick count and motion type.
        a = ((fabsf(MotionValue) + NODE_DM_ROT_OFFSET) * NODE_DM_ROT_CONST) - NODE_DM_COM_OFFSET;
        TickSetPoint = (uint16_t)roundf(a);
        if (MotionValue > 0.0f) MotionType = 2;
        else MotionType = 4;

// Send discrete motion command.
        LastCommand = NODE_LAST_CMD_DM_TURN;
        pMCU->DiscreteMotion(MotionType, TickSetPoint);
      }
    }
  }

// ***** Command "Slow" - same as "Turn" - just for backwards compatibility *****
  else if (strncmp((const char *)(&CmdBuf[0]), "Slow", 4) == 0)
  {

// The robot must be in idle mode.
    if (MotionState != 0) return;
    else
    {

// Get the desired angle and check if it's within the allowed limits.
      MotionValue = atof(&CmdBuf[5]);
      if (fabsf(MotionValue) < NODE_DM_MIN_ANG) return;
      else if (fabsf(MotionValue) > NODE_DM_MAX_ANG) return;
      else
      {

// Compute the required encoder tick count and motion type.
        a = ((fabsf(MotionValue) + NODE_DM_ROT_OFFSET) * NODE_DM_ROT_CONST) - NODE_DM_COM_OFFSET;
        TickSetPoint = (uint16_t)roundf(a);
        if (MotionValue > 0.0f) MotionType = 2;
        else MotionType = 4;

// Send discrete motion command.
        LastCommand = NODE_LAST_CMD_DM_TURN;
        pMCU->DiscreteMotion(MotionType, TickSetPoint);
      }
    }
  }

// ***** Command "Stop" - the motion state must be updated by the main loop *****
  else if (strncmp((const char *)(&CmdBuf[0]), "Stop", 4) == 0)
  {
    pMCU->DiscreteMotion(0, 0);
  }

// ***** Unsupported command *****
  else return;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Callback function that is invoked by the instance of the class
// 'cHobbitMCU' whenever a new status message was received from the
// MCU board.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CallBackFunc(cHobbitMCU *pClass)
{

// Get the current time.
  current_time = ros::Time::now();

// Get the current motion state of the mobile platform.
  MotionState = pClass->PlatformStatus.MotionState;

// Update the transform message and send it.
  odom_quat = tf::createQuaternionMsgFromYaw(pClass->PlatformOdometry.theta);
  odom_trans.header.stamp = current_time;
  odom_trans.transform.translation.x = pClass->PlatformOdometry.x;
  odom_trans.transform.translation.y = pClass->PlatformOdometry.y;
  odom_trans.transform.rotation = odom_quat;
  (*p_odom_broadcaster).sendTransform(odom_trans);

// Update the odometry message and send it.
  odom_odom.header.stamp = current_time;
  odom_odom.pose.pose.position.x = pClass->PlatformOdometry.x;
  odom_odom.pose.pose.position.y = pClass->PlatformOdometry.y;
  odom_odom.pose.pose.orientation = odom_quat;
  odom_odom.twist.twist.linear.x = pClass->PlatformOdometry.v;
  odom_odom.twist.twist.angular.z = pClass->PlatformOdometry.w;
  (*p_odom_pub).publish(odom_odom);

// Publish the motion state.
  if (MotionState == 0) stateString.data = "Idle";
  else
  {
    if (LastCommand == NODE_LAST_CMD_DM_MOVE) stateString.data = "Moving";
    else if (LastCommand == NODE_LAST_CMD_DM_TURN) stateString.data = "Turning";
    else if (LastCommand == NODE_LAST_CMD_CM_MOVE) stateString.data = "Moving";
    else stateString.data = "Idle";
  }
  (*p_state_pub).publish(stateString);

/*
  printf("(%f, %f, %f) - %u\n", pClass->PlatformOdometry.x, pClass->PlatformOdometry.y,
         (pClass->PlatformOdometry.theta * (180.0f / M_PI)), pClass->PlatformStatus.MotionState);
*/
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Main routine of the node 'hobbit_platform'. Does only the setup and
// cleanup, but leaves the "productive" part to the callback routine.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char** argv)
{

// There must be an argument handed passed along when starting - the name of the config (XML) file.
  if (argc != 2)
  {
    printf("\nusage: %s name_of_the_xml_file\n\n", argv[0]);
    return 0;
  }

// Initially there are no resources allocated
  pMCU = 0;
  bMCUready = false;
  MotionState = 0;
  LastCommand = NODE_LAST_CMD_NONE;

// Initialise the unused/fixed parts of the transform message.
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.z = 0.0;

// Initialise the unused/fixed parts of the odometry message.
  odom_odom.header.frame_id = "odom";
  odom_odom.pose.pose.position.z = 0.0;
  odom_odom.child_frame_id = "base_link";
  odom_odom.twist.twist.linear.y = 0.0;
  odom_odom.twist.twist.linear.z = 0.0;
  odom_odom.twist.twist.angular.x = 0.0;
  odom_odom.twist.twist.angular.y = 0.0;

// Initialise the ROS node and create subscribers for discrete motion and speed commands.
  ros::init(argc, argv, "hobbit_platform");
  ros::NodeHandle n;
  ros::Subscriber twist_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, twistCallback);
  ros::Subscriber discrete_sub = n.subscribe<std_msgs::String>("/DiscreteMotionCmd", 1, &discreteCallback);

// Create a publisher for the transform and the odometry data.
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
  p_odom_pub = &odom_pub;
  ros::Publisher state_pub = n.advertise<std_msgs::String>("/DiscreteMotionState", 1);
  p_state_pub = &state_pub;
  tf::TransformBroadcaster odom_broadcaster;
  p_odom_broadcaster = &odom_broadcaster;

  try
  {

// Create an instance of the class 'cHobbitMCU' that interfaces with the platform's MCU board.
    pMCU = new (std::nothrow) cHobbitMCU(1, argv[1]);
    if (pMCU == 0)
      throw cSysCallException(0, "main()", "Error.SysCall.new", errno);

// Set the callback function that is invoked whenever a new status message arrives from the MCU board.
    pMCU->SetCallBack(CallBackFunc);

// Open the communication port to the MCU board.
    printf(" - connecting to MCU\n");
    pMCU->Connect();
    usleep(50000);

// Turn on the motor controllers.
    printf(" - turning on key switch\n");
    pMCU->SetKeySwitch(true);
    usleep(50000);

// Enable status message transmission by the MCU board.
    printf(" - turning on status data transmission\n");
    pMCU->SetDataTransmission(true);
    bMCUready = true;

// Enter the ROS main loop to activate the subscribers.
    ros::spin();

// After the main loop was killed, disable status message transmission by the MCU board.
    printf(" - turning off status data transmission\n");
    bMCUready = false;
    pMCU->SetDataTransmission(false);
    usleep(50000);

// Turn off the motor controllers.
    printf(" - turning off key switch\n");
    pMCU->SetKeySwitch(false);
    usleep(50000);

// Close the communication port to the MCU board.
    printf(" - disconnecting from MCU\n");
    pMCU->Disconnect();
  }
  catch (cSysCallException e) {e.Print();}
  catch (cXmlException e) {e.Print();}
  catch (cSystemException e) {e.Print();}
  catch (cException e) {e.Print();}

// Remove the instance of the class 'cHobbitMCU'.
  printf("main(): Performing cleanup...\n");
  if (pMCU != 0)
  {
    printf("main(): Removing instance of type 'cHobbitMCU'.\n");
    delete pMCU;
    pMCU = 0;
  }

  return 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

