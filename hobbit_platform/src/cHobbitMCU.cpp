//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 26.2.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/hobbit_platform/cHobbitMCU.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// Static thread function.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Reader thread main loop - loops until it is commanded to pause or
// to end.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void *cHobbitMCU::ReaderThreadFunc(void *pArg)
{
  struct ThreadArgument *pTA = (struct ThreadArgument *)pArg;
  cHobbitMCU *pClass = (cHobbitMCU *)pTA->Arg;

  try
  {

// Decides whether to loop, end or just sleep.
    while (pTA->bThreadIsActive(pTA->pClass))
    {

// Periodically read from the data socket.
      pClass->ReceiveData();
    }
  }
  catch (cSysCallException e) {e.Print();}
  catch (cException e) {e.Print();}

// MANDATORY report of the thread's end.
  pTA->ReportThreadEnd(pTA->pClass);
  fprintf(stderr, "Reader thread has ended...\n");
  pthread_exit(0);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Writer thread main loop - loops until it is commanded to pause or
// to end.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void *cHobbitMCU::WriterThreadFunc(void *pArg)
{
  struct ThreadArgument *pTA = (struct ThreadArgument *)pArg;
  cHobbitMCU *pClass = (cHobbitMCU *)pTA->Arg;

  try
  {

// Decides whether to loop, end or just sleep.
    while (pTA->bThreadIsActive(pTA->pClass))
    {

// Periodically check if there is a new speed command.
      pClass->SpeedHandler();
    }
  }
  catch (cSysCallException e) {e.Print();}
  catch (cException e) {e.Print();}

// MANDATORY report of the thread's end.
  pTA->ReportThreadEnd(pTA->pClass);
  fprintf(stderr, "Writer thread has ended...\n");
  pthread_exit(0);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// Communication-related private methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This is the thread's main function. It periodically checks for new
// data received via the communication port and extracts status
// message packets from the incoming data stream. The data items
// are used to update platform state data structure.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::ReceiveData(void)
{
  union
  {
    uint8_t _u8[2];
    uint16_t _u16;
    int16_t _i16;
  } Trans;
  uint32_t u, v;
  float BatteryPeriod, BatteryPulseWidth;

// If there is space in the buffer, read from the communication port.
  if (ReadBufferContent < MY_READ_BUF_SIZE)
  {
    u = pPort->Read(&ReadBuffer[ReadBufferContent], (MY_READ_BUF_SIZE - ReadBufferContent));
    ReadBufferContent += u;
  }

// Main analysis loop - basically, a state machine.
  do
  {

// State #0 - no (valid) data has been received yet.
    while ((ReadState == 0) && (ReadBufferContent > 0))
    {

// A valid status data packet starts with '0xaa'.
      if (ReadBuffer[0] == 0xaa) ReadState++;

// If the expected data was not found, search for it in the rest of the data.
      else
      {
ReadLab00:
        u = 1;
        while (u < ReadBufferContent)
        {
          if (ReadBuffer[u] == 0xaa) break;
          else u++;
        }

// If such data has been found, discard any data in the buffer that came before it.
        ReadBufferContent -= u;
        for (v = 0; v < ReadBufferContent; v++) ReadBuffer[v] = ReadBuffer[u + v];
      }
    }

// State #1 - the second byte of a valid data packet is '0x55'.
    if (ReadState == 1)
    {
      if (ReadBufferContent < 2) return;
      else if (ReadBuffer[1] == 0x55) ReadState++;

// If the expected byte was not found, go back to state #0.
      else
      {
        ReadState = 0;
        goto ReadLab00;
      }
    }

// State #2 - here we just wait for the end of the status message packet.
    if (ReadState == 2)
    {
      if (ReadBufferContent < MY_STATUS_MSG_SIZE) return;
      else
      {

// If there is a complete packet, extract the information and store it in the platform status data structure.
        Trans._u8[0] = ReadBuffer[2];
        Trans._u8[1] = ReadBuffer[3];
        PlatformStatus.EncoderDiff0 = Trans._i16;

        Trans._u8[0] = ReadBuffer[4];
        Trans._u8[1] = ReadBuffer[5];
        PlatformStatus.EncoderDiff1 = Trans._i16;

        Trans._u8[0] = ReadBuffer[6];
        Trans._u8[1] = ReadBuffer[7];
        BatteryPulseWidth = (float)Trans._u16;

        Trans._u8[0] = ReadBuffer[8];
        Trans._u8[1] = ReadBuffer[9];
        BatteryPeriod = (float)Trans._u16;

        if (BatteryPeriod == 0.0f) PlatformStatus.BatteryLevel = 0.0f;
        else PlatformStatus.BatteryLevel = ((BatteryPulseWidth / BatteryPeriod) - 0.05f) * (1000.0f / 9.0f);

        PlatformStatus.BumperSwitch0 = ((ReadBuffer[10] & 0x01) == 0x00);
        PlatformStatus.BumperSwitch1 = ((ReadBuffer[10] & 0x02) == 0x00);
        PlatformStatus.BumperSwitch2 = ((ReadBuffer[10] & 0x10) == 0x00);
        PlatformStatus.BumperSwitch3 = ((ReadBuffer[10] & 0x20) == 0x00);

        PlatformStatus.MotionState = (uint16_t)ReadBuffer[11];

        ComputeOdometry();

// Remove the the packet from the input buffer and update the buffer content counter.
        ReadBufferContent -= MY_STATUS_MSG_SIZE;
        for (u = 0; u < ReadBufferContent; u++) ReadBuffer[u] = ReadBuffer[u + MY_STATUS_MSG_SIZE];

// If a callback function was set, call it.
        if (pCallBack != 0) pCallBack(this);
      }
    }

// If there's still data in the input buffer, keep evaluating it.
  } while (ReadBufferContent > 0);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Method dedicated to computing the odometry from the encoder tick
// difference of the left (encoder #0) and right (encoder #1) wheel
// encoder.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::ComputeOdometry(void)
{
  double d_l, d_r, DeltaTheta, DeltaDistance, dx, dy, a;

// Get the distance that the left wheel (encoder #0) has moved.
  d_l = (double)(PlatformStatus.EncoderDiff0) / TicksPerMeter;

// Get the distance that the right wheel (encoder #1) has moved.
  d_r = (double)(PlatformStatus.EncoderDiff1) / TicksPerMeter;

// Get the change in angle.
  DeltaTheta = (d_r - d_l) / (WheelDistance * WDcorr);

// If the number of right and left encoder ticks are equal (perfect translation)...
  if (d_r == d_l)
  {
    DeltaDistance = d_r;
    dx = DeltaDistance * cos(PlatformOdometry.theta);
    dy = DeltaDistance * sin(PlatformOdometry.theta);
  }

// If the number of ticks is not the same (moving along an arc)...
  else
  {
    DeltaDistance = sin(0.5 * DeltaTheta);
    DeltaDistance /= (d_r - d_l);
    DeltaDistance *= (d_r + d_l);
    DeltaDistance *= (WheelDistance * WDcorr);
    a = PlatformOdometry.theta + (0.5 * DeltaTheta);
    if (a < 0.0) a += (2.0 * M_PI);
    else if (a >= (2.0 * M_PI)) a -= (2.0 * M_PI);
    dx = DeltaDistance * cos(a);
    dy = DeltaDistance * sin(a);
  }

// Update the global position.
  PlatformOdometry.x += dx;
  PlatformOdometry.y += dy;

// Update the global orientation.
  PlatformOdometry.theta += DeltaTheta;
  if (PlatformOdometry.theta < 0.0) PlatformOdometry.theta += (2.0 * M_PI);
  else if (PlatformOdometry.theta >= (2.0 * M_PI)) PlatformOdometry.theta -= (2.0 * M_PI);

// Compute the translation and rotation speed.
  PlatformOdometry.v = (0.5 * (d_r + d_l)) / OdometryPeriod;
  PlatformOdometry.w = DeltaTheta / OdometryPeriod;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Slows down external speed commands to a manageable rate of 10-12Hz.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::SpeedHandler(void)
{
  double v, w;

  pWriterMutex->Lock();
  if (bSpeedUpdated)
  {
    bSpeedUpdated = false;
    v = NewTranslationSpeed;
    w = NewRotationSpeed;
    pWriterMutex->Unlock();
    SetSpeedExe(v, w);
  }
  else pWriterMutex->Unlock();
  usleep(80000);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Executes a speed command.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::SetSpeedExe(double v, double w)
{
  uint8_t CmdBuf[12];
  union
  {
    uint8_t _u8[2];
    uint16_t _u16;
  } Trans;
  double v_r, v_l, a0, a1;

// Limit the desired speeds according to what the robot can physically achieve.
  if (fabs(v) < min_v) v = 0.0;
  else if (fabs(v) > max_v)
  {
    if (v < 0.0) v = -max_v;
    else v = max_v;
  }

  if (fabs(w) < min_w) w = 0.0;
  else if (fabs(w) > max_w)
  {
    if (w < 0.0) w = -max_w;
    else w = max_w;
  }

// Compute the left and right wheel speed required to achieve the desired v and w.
  v_r = v + (0.5 * (w * (WheelDistance * WDcorr)));
  v_l = v - (0.5 * (w * (WheelDistance * WDcorr)));

// Compute the associated DAC values.
  if (v_l == 0.0) a0 = 0.0;
  else if (v_l > 0.0) a0 = (((1000.0 * v_l) - d_fwd) / k_fwd);
  else a0 = ((fabs(1000.0 * v_l) - d_bwd) / k_bwd);

  if (v_r == 0.0) a1 = 0.0;
  else if (v_r > 0.0) a1 = (((1000.0 * v_r) - d_fwd) / k_fwd);
  else a1 = ((fabs(1000.0 * v_r) - d_bwd) / k_bwd);

// Write the command to the buffer.
  CmdBuf[0] = 0x05;
  CmdBuf[1] = 0xfa;

// Compute separate 8bit DAC #0 value and boost value from floating point DAC #0 value.
  CmdBuf[2] = (uint8_t)floor(a0);
  a0 = 4.0 * (a0 - floor(a0));
  if (a0 < 0.5) CmdBuf[3] = 0;
  else if (a0 < 1.5) CmdBuf[3] = 1;
  else if (a0 < 2.5) CmdBuf[3] = 2;
  else if (a0 < 3.5) CmdBuf[3] = 3;
  else
  {
    CmdBuf[2]++;
    CmdBuf[3] = 0;
  }

// Compute separate 8bit DAC #1 value and boost value from floating point DAC #1 value.
  CmdBuf[4] = (uint8_t)floor(a1);
  a1 = 4.0 * (a1 - floor(a1));
  if (a1 < 0.5) CmdBuf[5] = 0;
  else if (a1 < 1.5) CmdBuf[5] = 1;
  else if (a1 < 2.5) CmdBuf[5] = 2;
  else if (a1 < 3.5) CmdBuf[5] = 3;
  else
  {
    CmdBuf[4]++;
    CmdBuf[5] = 0;
  }

// Write the DAC #0 and #1 direction flags to the buffer.
  CmdBuf[6] = 0x00;
  if (v_l < 0.0) CmdBuf[6] |= 0x02;
  if (v_r < 0.0) CmdBuf[6] |= 0x04;
  CmdBuf[7] = 0x00;

// Write the desired tick count #0 to the buffer.
  Trans._u16 = (uint16_t)round(fabs(v_l) * (TicksPerMeter * ControlPeriod));
  CmdBuf[8] = Trans._u8[0];
  CmdBuf[9] = Trans._u8[1];

// Write the desired tick count #1 to the buffer.
  Trans._u16 = (uint16_t)round(fabs(v_r) * (TicksPerMeter * ControlPeriod));
  CmdBuf[10] = Trans._u8[0];
  CmdBuf[11] = Trans._u8[1];

// Send the buffer to the communication port.
  pPort->Write(&CmdBuf[0], 12);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// General private methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method allocates and configures resources used by this instance
// of the class 'cHobbitMCU'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::Configuration(const char *ConfigFile)
{
  struct SimpleElement pAttr[4] = {{"ReaderThread", &pReaderThread, ELEMENT_OBJECT, false, 1},
                                   {"McuPort", &pPort, ELEMENT_OBJECT, false, 1},
                                   {"WriterThread", &pWriterThread, ELEMENT_OBJECT, false, 1},
                                   {"WriterMutex", &pWriterMutex, ELEMENT_OBJECT, false, 1}};
  uint32_t u;

  printf("*** cHobbitMCU [%u]: configuration ***\n", ID);

// Create and configure the system based on the XML file whose name is provided via command line.
  printf(" - creating an instance of the class 'cSystem'\n");
  pSystem = new (std::nothrow) cMySystem(ID + 1);
  if (pSystem == 0)
    throw cSysCallException(ID, "cHobbitMCU::Configuration()", "Error.SysCall.new", errno);
  printf(" - building and configuring a system based on file '%s'\n", ConfigFile);
  pSystem->BuildSystem(ConfigFile);
  bSystemBuilt = true;

// Initialise the instance's attributes from the XML file.
  for (u = 0; u < 4; u++) pAttr[u].bInitialised = false;
  pSystem->ParseElement(pAttr, 4);

// Check if all attributes have been initialised.
  for (u = 0; u < 4; u++)
    if (!pAttr[u].bInitialised)
      throw cException(ID, "cHobbitMCU::Configuration()", "Error.Not.Initialised");

// Create a thread for receiving data from the MCU.
  printf(" - creating MCU reader thread\n");
  if (!pReaderThread->Create(ReaderThreadFunc, (void *)this))
    throw cException(ID, "cHobbitMCU::Configuration()", "Error.ThreadCreate.Failed");

// Create a thread for transmitting data to the MCU.
  printf(" - creating MCU writer thread\n");
  if (!pWriterThread->Create(WriterThreadFunc, (void *)this))
    throw cException(ID, "cHobbitMCU::Configuration()", "Error.ThreadCreate.Failed");
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Removes the resources that had been allocated earlier.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::Cleanup(void)
{
  unsigned int LoopCtr;

  printf("*** cHobbitMCU [%u]: cleaning up ***\n", ID);

// Stop the writer thread.
  if (pWriterThread != 0)
  {
    printf(" - stopping writer thread\n");
    pWriterThread->cmdEnd();
  }

// Wait for the thread to end - at most 20x 50ms i.e. 1s.
  LoopCtr = 20;
  do
  {
    usleep(50000);
    if (pWriterThread->GetThreadState() == THREAD_STATE_ENDED) break;
    LoopCtr--;
  } while (LoopCtr > 0);
  if (LoopCtr > 0) printf("   - thread has ended normally\n");
  else printf("   - thread hasn't answered after 1s - ending anyway (might crash)\n");

// Stop the reader thread.
  if (pReaderThread != 0)
  {
    printf(" - stopping reader thread\n");
    pReaderThread->cmdEnd();
  }

// Wait for the thread to end - at most 20x 50ms i.e. 1s.
  LoopCtr = 20;
  do
  {
    usleep(50000);
    if (pReaderThread->GetThreadState() == THREAD_STATE_ENDED) break;
    LoopCtr--;
  } while (LoopCtr > 0);
  if (LoopCtr > 0) printf("   - thread has ended normally\n");
  else printf("   - thread hasn't answered after 1s - ending anyway (might crash)\n");

// Remove the current system, cleans up all lower level hierarchies that were created.
  if (pSystem != 0)
  {
    if (bSystemBuilt)
    {
      printf(" - destroying built system\n");
      pSystem->DestroySystem();
      bSystemBuilt = false;
    }
    printf(" - removing instance of the class 'cSystem' with ID '%u'\n", pSystem->GetID());
    delete pSystem;
    pSystem = 0;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// General public methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor - initialises some member attributes and invokes the
// configuration method of this class.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cHobbitMCU::cHobbitMCU(uint32_t InstanceID, const char *ConfigFile)
{

// Locally store the ID of this instance of the class 'cHobbitMCU'.
  ID = InstanceID;

// Initially there are no resources allocated.
  pSystem = 0;
  bSystemBuilt = false;
  pReaderThread = 0;
  pWriterThread = 0;
  ReadBufferContent = 0;
  ReadState = 0;

// Initialise member attributes.
  pCallBack = 0;
  PlatformStatus.EncoderDiff0 = 0x0000;
  PlatformStatus.EncoderDiff1 = 0x000;
  PlatformStatus.BatteryLevel = 0.0f;
  PlatformStatus.BumperSwitch0 = false;
  PlatformStatus.BumperSwitch1 = false;
  PlatformStatus.BumperSwitch2 = false;
  PlatformStatus.BumperSwitch3 = false;
  PlatformStatus.MotionState = 0x0000;
  PlatformOdometry.x = 0.0;
  PlatformOdometry.y = 0.0;
  PlatformOdometry.theta = 0.0;

  bSpeedUpdated = false;
  NewTranslationSpeed = 0.0;
  NewRotationSpeed = 0.0;

// FIXME - These values should be set via config file - FIXME
  WheelDistance = 0.397;
  WDcorr = 1.003;
  TicksPerMeter = 15373.0;
  OdometryPeriod = 0.1;
  ControlPeriod = 0.04;
  k_fwd = 4.527;
  d_fwd = -174.83;
  k_bwd = 4.555;
  d_bwd = -170.575;
  min_v = 0.02;
  max_v = 0.4;
  min_w = 0.105;
  max_w = 0.7854;

// Configure this instance of the class 'cHobbitMCU'.
  try
  {
    Configuration(ConfigFile);
  }
  catch (...)
  {
    Cleanup();
    throw;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Destructor - frees previously allocated resources for a clean exit.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cHobbitMCU::~cHobbitMCU()
{
  Cleanup();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the ID of this instance of the class 'cHobbitMCU'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cHobbitMCU::GetID(void)
{
  return ID;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// Public API methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set a callback function. This function is invoked whenever there
// is a new status data message from the MCU.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::SetCallBack(void (*pFunc)(cHobbitMCU *))
{
  pCallBack = pFunc;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Open the communication port and start the reader thread.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::Connect(void)
{

// Open the communication port.
  pPort->Open();

// Start the reader thread.
  pReaderThread->cmdRun();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Suspend the reader thread and close the communication port.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::Disconnect(void)
{

// Suspend the reader thread.
  pReaderThread->cmdSleep();

// Close the communication port.
  pPort->Close();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Command #0 - turn motor controllers off|on
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::SetKeySwitch(bool bOn)
{
  uint8_t CmdBuf[3];

// Write the command to the buffer.
  CmdBuf[0] = 0x00;
  CmdBuf[1] = 0xff;
  if (bOn) CmdBuf[2] = 0x01;
  else CmdBuf[2] = 0x00;

// Suspend the writer thread.
  if (!bOn) pWriterThread->cmdSleep();

// Send the buffer to the communication port.
  pPort->Write(&CmdBuf[0], 3);

// Start the writer thread.
  if (bOn) pWriterThread->cmdRun();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Command #1 - turn on/off data transmission from the MCU to the PC
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::SetDataTransmission(bool bOn)
{
  uint8_t CmdBuf[3];

// Write the command to the buffer.
  CmdBuf[0] = 0x01;
  CmdBuf[1] = 0xfe;
  if (bOn) CmdBuf[2] = 0x01;
  else CmdBuf[2] = 0x00;

// Send the buffer to the communication port.
  pPort->Write(&CmdBuf[0], 3);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Command #2 - set parameter (currently 64 parameters [0..63] are
// supported).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::SetParameter(uint16_t ParameterNumber, uint16_t ParameterValue)
{
  union
  {
    uint8_t _u8[2];
    uint16_t _u16;
  } Trans;
  uint8_t CmdBuf[6];

// Write the command to the buffer.
  CmdBuf[0] = 0x02;
  CmdBuf[1] = 0xfd;
  Trans._u16 = ParameterNumber;
  CmdBuf[2] = Trans._u8[0];
  CmdBuf[3] = Trans._u8[1];
  Trans._u16 = ParameterValue;
  CmdBuf[4] = Trans._u8[0];
  CmdBuf[5] = Trans._u8[1];

// Send the buffer to the communication port.
  pPort->Write(&CmdBuf[0], 6);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Command #3 - reset odometry
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::ResetOdometry(void)
{
  uint8_t CmdBuf[2];

// Write the command to the buffer.
  CmdBuf[0] = 0x03;
  CmdBuf[1] = 0xfc;

// Send the buffer to the communication port.
  pPort->Write(&CmdBuf[0], 2);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Command #4 - discrete motion
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::DiscreteMotion(uint16_t MotionType, uint16_t TickSetPoint)
{
  union
  {
    uint8_t _u8[2];
    uint16_t _u16;
  } Trans;
  uint8_t CmdBuf[6];

// Write the command to the buffer.
  CmdBuf[0] = 0x04;
  CmdBuf[1] = 0xfb;
  Trans._u16 = MotionType;
  CmdBuf[2] = Trans._u8[0];
  CmdBuf[3] = Trans._u8[1];
  Trans._u16 = TickSetPoint;
  CmdBuf[4] = Trans._u8[0];
  CmdBuf[5] = Trans._u8[1];

// Send the buffer to the communication port.
  pPort->Write(&CmdBuf[0], 6);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Command #5 - Speed command
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::SetSpeed(double v, double w)
{
  pWriterMutex->Lock();
  NewTranslationSpeed = v;
  NewRotationSpeed = w;
  bSpeedUpdated = true;
  pWriterMutex->Unlock();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Alternative Command #5 - this is to *manually* set DAC values and
// associated encoder tick setpoints.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cHobbitMCU::SetDAC(double Dac0, double Dac1, int16_t Ticks0, int16_t Ticks1)
{
  uint8_t CmdBuf[12];
  double a, b;
  union
  {
    uint8_t _u8[2];
    int16_t _i16;
  } Trans;

// Check and limit the DAC values.
  if (Dac0 < -255.0) Dac0 = -255.0;
  else if (Dac0 > 255.0) Dac0 = 255.0;
  if (Dac1 < -255.0) Dac1 = -255.0;
  else if (Dac1 > 255.0) Dac1 = 255.0;

// Write the command code to the buffer.
  CmdBuf[0] = 0x05;
  CmdBuf[1] = 0xfa;

// Compute the DAC #0 value and boost value and write them to the buffer.
  a = fabs(Dac0);
  b = floor(a);
  CmdBuf[2] = (uint8_t)b;
  a = 4.0 * (a - b);
  if (a < 0.5) CmdBuf[3] = 0;
  else if (a < 1.5) CmdBuf[3] = 1;
  else if (a < 2.5) CmdBuf[3] = 2;
  else if (a < 3.5) CmdBuf[3] = 3;
  else
  {
    CmdBuf[2]++;
    CmdBuf[3] = 0;
  }

// Compute the DAC #1 value and boost value and write them to the buffer.
  a = fabs(Dac1);
  b = floor(a);
  CmdBuf[4] = (uint8_t)b;
  a = 4.0 * (a - b);
  if (a < 0.5) CmdBuf[5] = 0;
  else if (a < 1.5) CmdBuf[5] = 1;
  else if (a < 2.5) CmdBuf[5] = 2;
  else if (a < 3.5) CmdBuf[5] = 3;
  else
  {
    CmdBuf[4]++;
    CmdBuf[5] = 0;
  }

// Write the DAC #0 and #1 direction flags to the buffer.
  CmdBuf[6] = 0x00;
  if (Dac0 < 0.0) CmdBuf[6] |= 0x02;
  if (Dac1 < 0.0) CmdBuf[6] |= 0x04;
  CmdBuf[7] = 0x00;

// Write the desired tick count #0 to the buffer.
  Trans._i16 = Ticks0;
  CmdBuf[8] = Trans._u8[0];
  CmdBuf[9] = Trans._u8[1];

// Write the desired tick count #1 to the buffer.
  Trans._i16 = Ticks1;
  CmdBuf[10] = Trans._u8[0];
  CmdBuf[11] = Trans._u8[1];

// Send the buffer to the communication port.
  pPort->Write(&CmdBuf[0], 12);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

