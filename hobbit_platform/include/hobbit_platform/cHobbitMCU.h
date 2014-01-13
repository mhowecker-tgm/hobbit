//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 26.2.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef C_HOBBIT_MCU_HH
#define C_HOBBIT_MCU_HH
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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define MY_READ_BUF_SIZE	32
#define MY_STATUS_MSG_SIZE	12
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
using namespace std;
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
struct OdometryData
{
  double x;
  double y;
  double theta;
  double v;
  double w;
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cHobbitMCU
{
//-------------------------------------------------------------------
// Static thread function.
//-------------------------------------------------------------------
    static void *ReaderThreadFunc(void *pArg);
    static void *WriterThreadFunc(void *pArg);

  protected:
//-------------------------------------------------------------------
// Private attributes.
//-------------------------------------------------------------------
    uint8_t ReadBuffer[MY_READ_BUF_SIZE];
    uint32_t ReadBufferContent;
    uint32_t ReadState;
    uint32_t ID;
    bool bSystemBuilt;
    cMySystem *pSystem;
    cThread *pReaderThread;
    cThread *pWriterThread;
    cMutex *pWriterMutex;
    cCommunicationPort *pPort;
    void (*pCallBack)(cHobbitMCU *);
    double WheelDistance, WDcorr, TicksPerMeter, OdometryPeriod, ControlPeriod;
    double k_fwd, d_fwd, k_bwd, d_bwd;
    double min_v, max_v, min_w, max_w;

    bool bSpeedUpdated;
    double NewTranslationSpeed, NewRotationSpeed;

//-------------------------------------------------------------------
// Communication-related private methods.
//-------------------------------------------------------------------
    void ReceiveData(void);
    void ComputeOdometry(void);
    void SpeedHandler(void);
    void SetSpeedExe(double v, double w);

//-------------------------------------------------------------------
// General private methods.
//-------------------------------------------------------------------
    void Configuration(const char *ConfigFile);
    void Cleanup(void);

//-------------------------------------------------------------------
// General public methods.
//-------------------------------------------------------------------
  public:
    struct StatusData PlatformStatus;
    struct OdometryData PlatformOdometry;
    cHobbitMCU(uint32_t InstanceID, const char *ConfigFile);
    ~cHobbitMCU();
    uint32_t GetID(void);

//-------------------------------------------------------------------
// Public API methods.
//-------------------------------------------------------------------
    void SetCallBack(void (*pFunc)(cHobbitMCU *));
    void Connect(void);
    void Disconnect(void);
    void SetKeySwitch(bool bOn);
    void SetDataTransmission(bool bOn);
    void SetParameter(uint16_t ParameterNumber, uint16_t ParameterValue);
    void ResetOdometry(void);
    void DiscreteMotion(uint16_t MotionType, uint16_t TickSetPoint);
    void SetSpeed(double v, double w);
    void SetDAC(double Dac0, double Dac1, int16_t Ticks0, int16_t Ticks1);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif

