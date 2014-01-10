//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 17.4.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef C_MOBILE_PLATFORM_HH
#define C_MOBILE_PLATFORM_HH
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <new>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include "cException.h"
#include "cMySystem.h"
#include "cXml.h"
#include "cThread.h"
#include "cMutex.h"
#include "cCommunicationPort.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The last command state.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define NODE_LAST_CMD_NONE	0
#define NODE_LAST_CMD_DM_MOVE	1
#define NODE_LAST_CMD_DM_TURN	2
#define NODE_LAST_CMD_CM_MOVE	3
#define NODE_LAST_CMD_STOP      4
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Communication buffer and message sizes.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define MY_READ_BUF_SIZE	36
#define MY_STATUS_MSG_SIZE	12
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Status data as received from the platform's MCU board.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
struct StatusData
{
  int16_t EncoderDiff0;
  int16_t EncoderDiff1;
  float BatteryLevel;
  bool BumperSwitch0;
  bool BumperSwitch1;
  bool BumperSwitch2;
  bool BumperSwitch3;
  uint16_t MotionState;
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Odometry data computed from the ecnoder tick difference received
// from the platform's MCU board and using the configured geometry
// data of the platform.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
struct OdometryData
{
  double x;
  double y;
  double theta;
  double v;
  double w;
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cMobilePlatform
{

//-------------------------------------------------------------------
// Static thread function.
//-------------------------------------------------------------------
    static void *ReaderThreadFunc(void *pArg);
    static void *WriterThreadFunc(void *pArg);

  protected:
//-------------------------------------------------------------------
// General private attributes.
//-------------------------------------------------------------------
    uint32_t ID;
    bool bSystemBuilt;
    cMySystem *pSystem;

//-------------------------------------------------------------------
// Private attributes initialised from the configuration file.
//-------------------------------------------------------------------
    double DMMinDist, DMMaxDist;
    double DMMinAng, DMMaxAng;
    double DMRotConst, DMRotOffset, DMComOffset;
    double WheelDistance, WDcorr, TicksPerMeter, OdometryPeriod, ControlPeriod;
    double k_fwd, d_fwd, k_bwd, d_bwd;
    double min_v, max_v, min_w, max_w;
    double ADCtoVoltage;
    uint16_t McuFwdVal0, McuFwdBoost0, McuFwdVal1, McuFwdBoost1;
    uint16_t McuLeftVal0, McuLeftBoost0, McuLeftVal1, McuLeftBoost1;
    uint16_t McuRightVal0, McuRightBoost0, McuRightVal1, McuRightBoost1;
    uint16_t McuBwdVal0, McuBwdBoost0, McuBwdVal1, McuBwdBoost1;
    cThread *pReaderThread;
    cThread *pWriterThread;
    cMutex *pWriterMutex;
    cCommunicationPort *pPort;
/*
    int16_t EncDiffHistory0[16];
    int16_t EncDiffHistory1[16];
    unsigned int EncDiffHistoryIndex;
*/
//-------------------------------------------------------------------
// Private speed command related attributes.
//-------------------------------------------------------------------
    bool bSpeedUpdated;
    double NewTranslationSpeed, NewRotationSpeed;

//-------------------------------------------------------------------
// Private MCU board communication related attributes.
//-------------------------------------------------------------------
    uint8_t ReadBuffer[MY_READ_BUF_SIZE];
    uint32_t ReadBufferContent;
    uint32_t ReadState;
    int LastCommand;
    bool bTransmissionOn;

//-------------------------------------------------------------------
// Private ROS related methods.
//-------------------------------------------------------------------
    tf::TransformBroadcaster *p_odom_broadcaster;
    ros::Publisher *p_odom_pub;
    ros::Publisher *p_state_pub;
    ros::Publisher *p_voltage_pub;
    ros::Time current_time;
    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom_odom;
    std_msgs::String stateString;
    std_msgs::Float64 batteryVoltage;

//-------------------------------------------------------------------
// Private MCU board related methods.
//-------------------------------------------------------------------
    void Connect(void);
    void Disconnect(void);
    void SetKeySwitch(bool bOn);
    void SetDataTransmission(bool bOn);
    void SetParameter(uint16_t ParameterNumber, uint16_t ParameterValue);
    void ResetOdometry(void);
    void DiscreteMotion(uint16_t MotionType, uint16_t TickSetPoint);
    void SetDAC(double Dac0, double Dac1, int16_t Ticks0, int16_t Ticks1);
/*
    void ClearEncDiffHistory(void);
    void GetEncDifferences(int16_t &Enc0, int16_t &Enc1);
*/
//-------------------------------------------------------------------
// Private communication-related methods.
//-------------------------------------------------------------------
    void ReceiveData(void);
    void ComputeOdometry(void);
    void SetSpeed(double v, double w);
    void SpeedHandler(void);

//-------------------------------------------------------------------
// ROS-related private methods.
//-------------------------------------------------------------------
/*
    void dacCallback(const std_msgs::String::ConstPtr& msg);
*/
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void discreteCallback(const std_msgs::String::ConstPtr& msg);

//-------------------------------------------------------------------
// General private methods.
//-------------------------------------------------------------------
    void SetMCUParameters(void);
    void Configuration(const char *ConfigFile);
    void Cleanup(void);

  public:
//-------------------------------------------------------------------
// General public attributes.
//-------------------------------------------------------------------
    struct StatusData PlatformStatus;
    struct OdometryData PlatformOdometry;
    int16_t CummulativeEnc0, CummulativeEnc1;

//-------------------------------------------------------------------
// General public methods.
//-------------------------------------------------------------------
    cMobilePlatform(uint32_t InstanceID, const char *ConfigFile);
    ~cMobilePlatform();
    uint32_t GetID(void);
    void run(int argc, char **argv);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif

