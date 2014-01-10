//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 17.4.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/MobilePlatform/cException.h"
#include "../include/MobilePlatform/MobilePlatform.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// Static thread function.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Reader thread main loop - loops until it is commanded to pause or
// to end. Reads data from the socket and extracts messages from the
// MCU board.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void *cMobilePlatform::ReaderThreadFunc(void *pArg)
{
  struct ThreadArgument *pTA = (struct ThreadArgument *)pArg;
  cMobilePlatform *pClass = (cMobilePlatform *)pTA->Arg;

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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Writer thread main loop - loops until it is commanded to pause or
// to end. Checks if there is a new speed command and if so, sends a
// speed command to the MCU board.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void *cMobilePlatform::WriterThreadFunc(void *pArg)
{
  struct ThreadArgument *pTA = (struct ThreadArgument *)pArg;
  cMobilePlatform *pClass = (cMobilePlatform *)pTA->Arg;

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
// Private MCU board related methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Open the communication port and start the reader thread.
// TODO: - when the USB-serial converter specified by the config file
//         is not found or the access rights are not given, there will
//         be an exception.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::Connect(void)
{

// Open the communication port.
  pPort->Open();

// Start the reader thread.
  pReaderThread->cmdRun();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Suspend the reader thread and close the communication port.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::Disconnect(void)
{

// Suspend the reader thread.
  pReaderThread->cmdSleep();

// Close the communication port.
  pPort->Close();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Command #0 - turn motor controllers off|on
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::SetKeySwitch(bool bOn)
{
  uint8_t CmdBuf[3];

// Write the command to the buffer.
  CmdBuf[0] = 0x00;
  CmdBuf[1] = 0xff;
  if (bOn) CmdBuf[2] = 0x01;
  else CmdBuf[2] = 0x00;

// Suspend the writer thread - speed commands will not be accepted anyway.
  if (!bOn) pWriterThread->cmdSleep();

// Send the buffer to the communication port.
  pPort->Write(&CmdBuf[0], 3);

// Start the writer thread - speed commands will be accepted.
  if (bOn) pWriterThread->cmdRun();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Command #1 - turn on/off data transmission from the MCU to the PC
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::SetDataTransmission(bool bOn)
{
  uint8_t CmdBuf[3];

// Write the command to the buffer.
  CmdBuf[0] = 0x01;
  CmdBuf[1] = 0xfe;
  if (bOn) CmdBuf[2] = 0x01;
  else CmdBuf[2] = 0x00;

// If the transmission is turned off, set the flag false.
  if (!bOn) bTransmissionOn = false;

// Send the buffer to the communication port.
  pPort->Write(&CmdBuf[0], 3);

// If the transmission is turned on, set the flag true.
  if (bOn) bTransmissionOn = true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Command #2 - set parameter (currently 64 parameters [0..63] are
// supported).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::SetParameter(uint16_t ParameterNumber, uint16_t ParameterValue)
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

// Write the parameter number to the buffer.
  Trans._u16 = ParameterNumber;
  CmdBuf[2] = Trans._u8[0];
  CmdBuf[3] = Trans._u8[1];

// Write the parameter value to the buffer.
  Trans._u16 = ParameterValue;
  CmdBuf[4] = Trans._u8[0];
  CmdBuf[5] = Trans._u8[1];

// Send the buffer to the communication port.
  pPort->Write(&CmdBuf[0], 6);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Command #3 - reset odometry
// TODO: - this command should only be issued when the platform is
//         not in motion.
//       - the main application of this command is to reset the MCU
//         board's encoder tick counters before a discrete motion
//         command.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::ResetOdometry(void)
{
  uint8_t CmdBuf[2];

// Write the command to the buffer.
  CmdBuf[0] = 0x03;
  CmdBuf[1] = 0xfc;

// Send the buffer to the communication port.
  pPort->Write(&CmdBuf[0], 2);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Command #4 - discrete motion
// TODO: - this command shall only be sent when the mobile platform
//         is not in motion.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::DiscreteMotion(uint16_t MotionType, uint16_t TickSetPoint)
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

// Write the motion type (translation or rotation) to the buffer.
  Trans._u16 = MotionType;
  CmdBuf[2] = Trans._u8[0];
  CmdBuf[3] = Trans._u8[1];

// Write the encoder tick count setpoint to the buffer.
  Trans._u16 = TickSetPoint;
  CmdBuf[4] = Trans._u8[0];
  CmdBuf[5] = Trans._u8[1];

// Set the cummulative encoder #0 and #1 ticks to zero.
  CummulativeEnc0 = 0;
  CummulativeEnc1 = 0;

// Send the buffer to the communication port.
  pPort->Write(&CmdBuf[0], 6);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Command #5 - this is to set DAC values and associated encoder tick
//              setpoints.
// TODO: - is invoked by the method "SetHandler".
//       - currently not in use but might be useful in the future
//         for debugging.
//       - the DAC values are 8bit integers [0..255]. To allow for
//         finer resolution, a 2bit post-comma part was introduced
//         {0.0, 0.25, 0.5, 0.75} - "boost".
//       - the parameters "Ticks0" and "Ticks1" were initially in-
//         tended as setpoints for speed control of the MCU board
//         but are currently not used.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::SetDAC(double Dac0, double Dac1, int16_t Ticks0, int16_t Ticks1)
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

// Write the desired tick count #0 (per period) to the buffer.
  Trans._i16 = Ticks0;
  CmdBuf[8] = Trans._u8[0];
  CmdBuf[9] = Trans._u8[1];

// Write the desired tick count #1 (per period) to the buffer.
  Trans._i16 = Ticks1;
  CmdBuf[10] = Trans._u8[0];
  CmdBuf[11] = Trans._u8[1];

// Send the buffer to the communication port.
  pPort->Write(&CmdBuf[0], 12);
}
/*
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Clears the history of encoder tick differences.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::ClearEncDiffHistory(void)
{
  EncDiffHistoryIndex = 0;
  do
  {
    EncDiffHistory0[EncDiffHistoryIndex] = 0;
    EncDiffHistory1[EncDiffHistoryIndex] = 0;
    EncDiffHistoryIndex++;
    EncDiffHistoryIndex &= 15;
  } while (EncDiffHistoryIndex > 0);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::GetEncDifferences(int16_t &Enc0, int16_t &Enc1)
{
  unsigned int u, v;

  Enc0 = 0;
  Enc1 = 0;

  u = (EncDiffHistoryIndex - 10) & 15;
  for (v = 0; v < 10; v++)
  {
    Enc0 += EncDiffHistory0[u];
    Enc1 += EncDiffHistory1[u];
    u++;
    u &= 15;
  }
}
*/
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// Private communication-related methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// This is the thread's main function. It periodically checks for new
// data received via the communication port and extracts status
// message packets from the incoming data stream. The data items
// are used to update platform state data structure.
// TODO: - if the MCU board sends a motion state other than zero but
//         no motion command is active... can this be?
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::ReceiveData(void)
{
  union
  {
    uint8_t _u8[2];
    uint16_t _u16;
    int16_t _i16;
  } Trans;
  uint32_t u, v;
  float BatteryPeriod, BatteryPulseWidth;
/*
  int16_t Enc0, Enc1;
*/
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

// If too few data is in the buffer, return immediately.
      if (ReadBufferContent < 2) return;

// If the expected byte was found, go to the next state.
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

// If there is not enough data, return immediately.
      if (ReadBufferContent < MY_STATUS_MSG_SIZE) return;

// If there is a complete packet, extract the information and store it in the platform status data structure.
      else
      {

// Get the tick difference of encoder #0.
        Trans._u8[0] = ReadBuffer[2];
        Trans._u8[1] = ReadBuffer[3];
        PlatformStatus.EncoderDiff0 = Trans._i16;
        CummulativeEnc0 += Trans._i16;
/*
        EncDiffHistory0[EncDiffHistoryIndex]= Trans._i16;
*/
// Get the tick difference of encoder #1.
        Trans._u8[0] = ReadBuffer[4];
        Trans._u8[1] = ReadBuffer[5];
        PlatformStatus.EncoderDiff1 = Trans._i16;
        CummulativeEnc1 += Trans._i16;
/*
        EncDiffHistory1[EncDiffHistoryIndex]= Trans._i16;
        EncDiffHistoryIndex++;
        EncDiffHistoryIndex &= 15;
        GetEncDifferences(Enc0, Enc1);
*/
// Get the battery management pulse width.
        Trans._u8[0] = ReadBuffer[6];
        Trans._u8[1] = ReadBuffer[7];
        BatteryPulseWidth = (float)Trans._u16;

// Get the battery management period duration.
        Trans._u8[0] = ReadBuffer[8];
        Trans._u8[1] = ReadBuffer[9];
        BatteryPeriod = (float)Trans._u16;

// Compute the battery charging level from the previous two values.
        if (BatteryPeriod == 0.0f) PlatformStatus.BatteryLevel = (float)((double)BatteryPulseWidth * ADCtoVoltage);
        else PlatformStatus.BatteryLevel = ((BatteryPulseWidth / BatteryPeriod) - 0.05f) * (1000.0f / 9.0f);
        batteryVoltage.data = (double)PlatformStatus.BatteryLevel;

// Get the state of the four bumper switches.
        PlatformStatus.BumperSwitch0 = ((ReadBuffer[10] & 0x01) == 0x00);
        PlatformStatus.BumperSwitch1 = ((ReadBuffer[10] & 0x02) == 0x00);
        PlatformStatus.BumperSwitch2 = ((ReadBuffer[10] & 0x10) == 0x00);
        PlatformStatus.BumperSwitch3 = ((ReadBuffer[10] & 0x20) == 0x00);

// Get the motion state.
        PlatformStatus.MotionState = (uint16_t)ReadBuffer[11];

// Compute the odometry from the encoder tick differences.
        ComputeOdometry();

// Remove the the packet from the input buffer and update the buffer content counter.
        ReadBufferContent -= MY_STATUS_MSG_SIZE;
        for (u = 0; u < ReadBufferContent; u++) ReadBuffer[u] = ReadBuffer[u + MY_STATUS_MSG_SIZE];

// Get the current time.
        current_time = ros::Time::now();

// Update the transform message and send it.
        odom_quat = tf::createQuaternionMsgFromYaw(PlatformOdometry.theta);
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = PlatformOdometry.x;
        odom_trans.transform.translation.y = PlatformOdometry.y;
        odom_trans.transform.rotation = odom_quat;
        (*p_odom_broadcaster).sendTransform(odom_trans);

// Update the odometry message and send it.
        odom_odom.header.stamp = current_time;
        odom_odom.pose.pose.position.x = PlatformOdometry.x;
        odom_odom.pose.pose.position.y = PlatformOdometry.y;
        odom_odom.pose.pose.orientation = odom_quat;
        odom_odom.twist.twist.linear.x = PlatformOdometry.v;
        odom_odom.twist.twist.angular.z = PlatformOdometry.w;
        (*p_odom_pub).publish(odom_odom);

// Publish the motion state.
        if (PlatformStatus.MotionState == 0) stateString.data = "Idle";
        else
        {
          if (LastCommand == NODE_LAST_CMD_DM_MOVE) stateString.data = "Moving";
          else if (LastCommand == NODE_LAST_CMD_DM_TURN) stateString.data = "Turning";
          else if (LastCommand == NODE_LAST_CMD_CM_MOVE) stateString.data = "Moving";
          else if (LastCommand == NODE_LAST_CMD_STOP) stateString.data = "Stopping";
          else stateString.data = "No active motion command but not idle!?";
        }
        (*p_state_pub).publish(stateString);

// Publish the battery voltage.
        (*p_voltage_pub).publish(batteryVoltage);
      }
    }

// If there's still data in the input buffer, keep evaluating it.
  } while (ReadBufferContent > 0);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Method dedicated to computing the odometry from the encoder tick
// difference of the left (encoder #0) and right (encoder #1) wheel
// encoder.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::ComputeOdometry(void)
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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Speed command
// TODO: - this command should NOT be issued when a discrete motion
//         command is active.
//       - the command only tells the writer thread that there is a
//         speed command to be sent to the MCU board but does not
//         send it itself. The idea is to have control over the rate
//         at which speed commands are being sent to the MCU board.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::SetSpeed(double v, double w)
{

// Get exclusive access to the speed command attributes.
  pWriterMutex->Lock();

// Update the speed command attributes.
  NewTranslationSpeed = v;
  NewRotationSpeed = w;
  bSpeedUpdated = true;

// Allow access again.
  pWriterMutex->Unlock();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Slows down external speed commands to a manageable rate of 10-12Hz.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::SpeedHandler(void)
{
  double v, w, v_r, v_l, a0, a1;
  int16_t Ticks0, Ticks1;

// Get exclusive access to the speed command attributes.
  pWriterMutex->Lock();

// Only do something if a speed command had been issued.
  if (bSpeedUpdated)
  {

// Get the speed attributes and clear the speed command flag.
    v = NewTranslationSpeed;
    w = NewRotationSpeed;
    bSpeedUpdated = false;

// Allow access again.
    pWriterMutex->Unlock();

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

// Compute the required left and right encoder ticks per second.
    v_r *= TicksPerMeter;
    v_l *= TicksPerMeter;

// Compute the required tick count #0 and count #1 per control period.
    Ticks0 = (int16_t)round(v_l * ControlPeriod);
    Ticks1 = (int16_t)round(v_r * ControlPeriod);

// Compute the associated DAC values.
    if (v_l == 0.0) a0 = 0.0;
    else if (v_l > 0.0) a0 = ((v_l - d_fwd) / k_fwd);
    else a0 = -((fabs(v_l) - d_bwd) / k_bwd);

    if (v_r == 0.0) a1 = 0.0;
    else if (v_r > 0.0) a1 = ((v_r - d_fwd) / k_fwd);
    else a1 = -((fabs(v_r) - d_bwd) / k_bwd);

/**
// Compute the associated DAC values.
    if (v_l == 0.0) a0 = 0.0;
    else if (v_l > 0.0) a0 = (((1000.0 * v_l) - d_fwd) / k_fwd);
    else a0 = -(((1000.0 * fabs(v_l)) - d_bwd) / k_bwd);

    if (v_r == 0.0) a1 = 0.0;
    else if (v_r > 0.0) a1 = (((1000.0 * v_r) - d_fwd) / k_fwd);
    else a1 = -(((1000.0 * fabs(v_r)) - d_bwd) / k_bwd);

// Compute the required tick count #0 and count #1.
    Ticks0 = (int16_t)round(v_l * (TicksPerMeter * ControlPeriod));
    Ticks1 = (int16_t)round(v_r * (TicksPerMeter * ControlPeriod));
**/
// Send the command to the MCU board.
    SetDAC(a0, a1, Ticks0, Ticks1);
  }

// Allow access again.
  else pWriterMutex->Unlock();

// Just sleep some time that we limit the speed command rate.
  usleep(80000);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// ROS-related private methods.
//************************************************************************
/*
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::dacCallback(const std_msgs::String::ConstPtr& msg)
{
  char CmdBuf[64];
  double Dac0, Dac1;
  char *pAux;
  unsigned int u;

// Copy the received command to a local buffer for analysis.
  pAux = (char *)msg->data.c_str();
  for (u = 0; u < (64 - 1); u++)
  {
    CmdBuf[u] = pAux[u];
    if (CmdBuf[u] == 0) break;
  }
  if (u == (64 - 1)) CmdBuf[u] = 0;

// Find the *single space character* after the first value.
  for (u = 0; u < (64 - 1); u++) if (CmdBuf[u] == ' ') break;
  if (u == (64 - 1)) return;
  CmdBuf[u] = 0;
  Dac0 = atof(&CmdBuf[0]);
  Dac1 = atof(&CmdBuf[u + 1]);

  printf("cMobilePlatform::dacCallback(): %f | %f\n", Dac0, Dac1);
  SetDAC(Dac0, Dac1, 0, 0);
}
*/
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// ROS callback function for incoming twist messages.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double v, w;

// If the class 'cMobilePlatform' is instantiated and ready, it directly gets the desired twist.
  v = msg->linear.x;
  w = msg->angular.z;

// The robot must be in idle mode or executing speed commands.
  if (bTransmissionOn && ((PlatformStatus.MotionState == 0) || (PlatformStatus.MotionState == 3)))
  {
    LastCommand = NODE_LAST_CMD_CM_MOVE;
    SetSpeed(v, w);
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Callback function for commands sent to this node via the topic
// "/DiscreteMotion" which is of the type "std_msgs/String"
// Valid commands are:
// - "Move <distance>" - [meters]; pos->forward, neg->backwards
// - "Turn <angle>" - [degrees]; pos->ccw, neg->cw
// - "Stop" - sends a motion command with all zero to the platform
// Invalid commands will be ignored.
// TODO: - it is assumed that the argument of motion commands 'Move'
//         and 'Turn' is a valid floating point value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::discreteCallback(const std_msgs::String::ConstPtr& msg)
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

// The robot must be in idle mode - otherwise ignore.
    if (PlatformStatus.MotionState != 0) return;
    else
    {

// Get the desired distance and check if it's within the allowed limits - if not ignore command.
      MotionValue = atof(&CmdBuf[5]);
      if (fabsf(MotionValue) < DMMinDist) return;
      else if (fabsf(MotionValue) > DMMaxDist) return;
      else
      {

// Compute the required encoder tick count and motion type.
        a = (fabsf(MotionValue) * TicksPerMeter) - DMComOffset;
        TickSetPoint = (uint16_t)roundf(a);
        if (MotionValue > 0.0f) MotionType = 0;
        else MotionType = 6;

// Send discrete motion command.
        LastCommand = NODE_LAST_CMD_DM_MOVE;
        DiscreteMotion(MotionType, TickSetPoint);
      }
    }
  }

// ***** Command "Turn" - must be idle state *****
  else if (strncmp((const char *)(&CmdBuf[0]), "Turn", 4) == 0)
  {

// The robot must be in idle mode.
    if (PlatformStatus.MotionState != 0) return;
    else
    {

// Get the desired angle and check if it's within the allowed limits - if not ignore command.
      MotionValue = atof(&CmdBuf[5]);
      if (fabsf(MotionValue) < DMMinAng) return;
      else if (fabsf(MotionValue) > DMMaxAng) return;
      else
      {

// Compute the required encoder tick count and motion type.
        a = ((fabsf(MotionValue) + DMRotOffset) * DMRotConst) - DMComOffset;
        TickSetPoint = (uint16_t)roundf(a);
        if (MotionValue > 0.0f) MotionType = 2;
        else MotionType = 4;

// Send discrete motion command.
        LastCommand = NODE_LAST_CMD_DM_TURN;
        DiscreteMotion(MotionType, TickSetPoint);
      }
    }
  }

// ***** Command "Slow" - same as "Turn" - just for backwards compatibility *****
  else if (strncmp((const char *)(&CmdBuf[0]), "Slow", 4) == 0)
  {

// The robot must be in idle mode.
    if (PlatformStatus.MotionState != 0) return;
    else
    {

// Get the desired angle and check if it's within the allowed limits - atherwise ignore command.
      MotionValue = atof(&CmdBuf[5]);
      if (fabsf(MotionValue) < DMMinAng) return;
      else if (fabsf(MotionValue) > DMMaxAng) return;
      else
      {

// Compute the required encoder tick count and motion type.
        a = ((fabsf(MotionValue) + DMRotOffset) * DMRotConst) - DMComOffset;
        TickSetPoint = (uint16_t)roundf(a);
        if (MotionValue > 0.0f) MotionType = 2;
        else MotionType = 4;

// Send discrete motion command.
        LastCommand = NODE_LAST_CMD_DM_TURN;
        DiscreteMotion(MotionType, TickSetPoint);
      }
    }
  }

// ***** Command "Stop" - the motion state must be updated by the main loop *****
  else if (strncmp((const char *)(&CmdBuf[0]), "Stop", 4) == 0)
  {
    LastCommand = NODE_LAST_CMD_STOP;
    DiscreteMotion(0, 0);
  }

// ***** Unsupported command *****
  else return;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// General private methods.
//************************************************************************
void cMobilePlatform::SetMCUParameters(void)
{
  printf(" - Setting MCU parameters\n");

  usleep(20000);
  SetParameter(0, McuFwdVal0);
  usleep(20000);
  SetParameter(1, McuFwdBoost0);
  usleep(20000);
  SetParameter(2, McuFwdVal1);
  usleep(20000);
  SetParameter(3, McuFwdBoost1);

  usleep(20000);
  SetParameter(4, McuLeftVal0);
  usleep(20000);
  SetParameter(5, McuLeftBoost0);
  usleep(20000);
  SetParameter(6, McuLeftVal1);
  usleep(20000);
  SetParameter(7, McuLeftBoost1);

  usleep(20000);
  SetParameter(8, McuRightVal0);
  usleep(20000);
  SetParameter(9, McuRightBoost0);
  usleep(20000);
  SetParameter(10, McuRightVal1);
  usleep(20000);
  SetParameter(11, McuRightBoost1);

  usleep(20000);
  SetParameter(12, McuBwdVal0);
  usleep(20000);
  SetParameter(13, McuBwdBoost0);
  usleep(20000);
  SetParameter(14, McuBwdVal1);
  usleep(20000);
  SetParameter(15, McuBwdBoost1);

  usleep(20000);
  printf("   - done\n");
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// This method allocates and configures resources used by this instance
// of the class 'cMobilePlatform'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::Configuration(const char *ConfigFile)
{
  struct SimpleElement pAttr[41] = {{"DMMinDist", &DMMinDist, ELEMENT_DOUBLE, false, 1},
                                    {"DMMaxDist", &DMMaxDist, ELEMENT_DOUBLE, false, 1},
                                    {"DMMinAng", &DMMinAng, ELEMENT_DOUBLE, false, 1},
                                    {"DMMaxAng", &DMMaxAng, ELEMENT_DOUBLE, false, 1},
                                    {"DMRotConst", &DMRotConst, ELEMENT_DOUBLE, false, 1},
                                    {"DMRotOffset", &DMRotOffset, ELEMENT_DOUBLE, false, 1},
                                    {"DMComOffset", &DMComOffset, ELEMENT_DOUBLE, false, 1},
                                    {"WheelDistance", &WheelDistance, ELEMENT_DOUBLE, false, 1},
                                    {"WDcorr", &WDcorr, ELEMENT_DOUBLE, false, 1},
                                    {"TicksPerMeter", &TicksPerMeter, ELEMENT_DOUBLE, false, 1},
                                    {"OdometryPeriod", &OdometryPeriod, ELEMENT_DOUBLE, false, 1},
                                    {"ControlPeriod", &ControlPeriod, ELEMENT_DOUBLE, false, 1},
                                    {"k_fwd", &k_fwd, ELEMENT_DOUBLE, false, 1},
                                    {"d_fwd", &d_fwd, ELEMENT_DOUBLE, false, 1},
                                    {"k_bwd", &k_bwd, ELEMENT_DOUBLE, false, 1},
                                    {"d_bwd", &d_bwd, ELEMENT_DOUBLE, false, 1},
                                    {"min_v", &min_v, ELEMENT_DOUBLE, false, 1},
                                    {"max_v", &max_v, ELEMENT_DOUBLE, false, 1},
                                    {"min_w", &min_w, ELEMENT_DOUBLE, false, 1},
                                    {"max_w", &max_w, ELEMENT_DOUBLE, false, 1},
                                    {"ADCtoVoltage", &ADCtoVoltage, ELEMENT_DOUBLE, false, 1},
                                    {"McuFwdVal0", &McuFwdVal0, ELEMENT_UINT16, false, 1},
                                    {"McuFwdBoost0", &McuFwdBoost0, ELEMENT_UINT16, false, 1},
                                    {"McuFwdVal1", &McuFwdVal1, ELEMENT_UINT16, false, 1},
                                    {"McuFwdBoost1", &McuFwdBoost1, ELEMENT_UINT16, false, 1},
                                    {"McuLeftVal0", &McuLeftVal0, ELEMENT_UINT16, false, 1},
                                    {"McuLeftBoost0", &McuLeftBoost0, ELEMENT_UINT16, false, 1},
                                    {"McuLeftVal1", &McuLeftVal1, ELEMENT_UINT16, false, 1},
                                    {"McuLeftBoost1", &McuLeftBoost1, ELEMENT_UINT16, false, 1},
                                    {"McuRightVal0", &McuRightVal0, ELEMENT_UINT16, false, 1},
                                    {"McuRightBoost0", &McuRightBoost0, ELEMENT_UINT16, false, 1},
                                    {"McuRightVal1", &McuRightVal1, ELEMENT_UINT16, false, 1},
                                    {"McuRightBoost1", &McuRightBoost1, ELEMENT_UINT16, false, 1},
                                    {"McuBwdVal0", &McuBwdVal0, ELEMENT_UINT16, false, 1},
                                    {"McuBwdBoost0", &McuBwdBoost0, ELEMENT_UINT16, false, 1},
                                    {"McuBwdVal1", &McuBwdVal1, ELEMENT_UINT16, false, 1},
                                    {"McuBwdBoost1", &McuBwdBoost1, ELEMENT_UINT16, false, 1},
                                    {"ReaderThread", &pReaderThread, ELEMENT_OBJECT, false, 1},
                                    {"McuPort", &pPort, ELEMENT_OBJECT, false, 1},
                                    {"WriterThread", &pWriterThread, ELEMENT_OBJECT, false, 1},
                                    {"WriterMutex", &pWriterMutex, ELEMENT_OBJECT, false, 1}};
  uint32_t u;

  printf("*** cMobilePlatform [%u]: configuration ***\n", ID);

// Create and configure the system based on the XML file whose name is provided via command line.
  printf(" - creating an instance of the class 'cSystem'\n");
  pSystem = new (std::nothrow) cMySystem(ID + 1);
  if (pSystem == 0)
    throw cSysCallException(ID, "cMobilePlatform::Configuration()", "Error.SysCall.new", errno);
  printf(" - building and configuring a system based on file '%s'\n", ConfigFile);
  pSystem->BuildSystem(ConfigFile);
  bSystemBuilt = true;

// Initialise the instance's attributes from the XML file.
  for (u = 0; u < 41; u++) pAttr[u].bInitialised = false;
  pSystem->ParseElement(pAttr, 41);

// Check if all attributes have been initialised.
  for (u = 0; u < 41; u++)
    if (!pAttr[u].bInitialised)
      throw cException(ID, "cMobilePlatform::Configuration()", "Error.Not.Initialised");

// Create a thread for receiving data from the MCU.
  printf(" - creating MCU reader thread\n");
  if (!pReaderThread->Create(ReaderThreadFunc, (void *)this))
    throw cException(ID, "cMobilePlatform::Configuration()", "Error.ThreadCreate.Failed");

// Create a thread for transmitting data to the MCU.
  printf(" - creating MCU writer thread\n");
  if (!pWriterThread->Create(WriterThreadFunc, (void *)this))
    throw cException(ID, "cMobilePlatform::Configuration()", "Error.ThreadCreate.Failed");
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Removes the resources that had been allocated earlier.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::Cleanup(void)
{
  unsigned int LoopCtr;

  printf("*** cMobilePlatform [%u]: cleaning up ***\n", ID);

// Stop the writer thread.
  if (pWriterThread != 0)
  {
    printf(" - stopping writer thread\n");
    pWriterThread->cmdEnd();

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
  }

// Stop the reader thread.
  if (pReaderThread != 0)
  {
    printf(" - stopping reader thread\n");
    pReaderThread->cmdEnd();

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
  }

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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Constructor - initialises some member attributes and invokes the
// configuration method of this class.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cMobilePlatform::cMobilePlatform(uint32_t InstanceID, const char *ConfigFile)
{

// Locally store the ID of this instance of the class 'cMobilePlatform'.
  ID = InstanceID;

// Initially there are no resources allocated and everything is idle.
  pSystem = 0;
  bSystemBuilt = false;
  pReaderThread = 0;
  pWriterThread = 0;
  ReadBufferContent = 0;
  ReadState = 0;
  LastCommand = NODE_LAST_CMD_NONE;
  bTransmissionOn = false;

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

// Configure this instance of the class 'cMobilePlatform'.
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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Destructor - frees previously allocated resources for a clean exit.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cMobilePlatform::~cMobilePlatform()
{
  Cleanup();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Returns the ID of this instance of the class 'cMobilePlatform'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cMobilePlatform::GetID(void)
{
  return ID;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// This opens a communication port to the MCU board and tells the 
// latter to switch on the motor controllers and the status data
// transmission.
// After that the method stays in the ROS spin loop until the latter
// receives a Ctrl-C.
// Finally, the MCU board is told to switch off status data trans-
// mission and the motor controllers, and the communication port is
// closed.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMobilePlatform::run(int argc, char **argv)
{

// Initialise the ROS node and create subscribers for discrete motion and speed commands.
  ros::init(argc, argv, "MobilePlatform");
  ros::NodeHandle n;
/*
  ros::Subscriber dac_sub = n.subscribe<std_msgs::String>("/DacCmd", 1, &cMobilePlatform::dacCallback, this);
*/
  ros::Subscriber twist_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &cMobilePlatform::twistCallback, this);
  ros::Subscriber discrete_sub = n.subscribe<std_msgs::String>("/DiscreteMotionCmd", 1, &cMobilePlatform::discreteCallback, this);

// Create a publisher for the transform and the odometry data.
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
  p_odom_pub = &odom_pub;
  ros::Publisher state_pub = n.advertise<std_msgs::String>("/DiscreteMotionState", 1);
  p_state_pub = &state_pub;
  ros::Publisher voltage_pub = n.advertise<std_msgs::Float64>("/BatteryVoltage", 1);
  p_voltage_pub = &voltage_pub;
  tf::TransformBroadcaster odom_broadcaster;
  p_odom_broadcaster = &odom_broadcaster;

// Open the communication port to the MCU board.
    printf(" - connecting to MCU\n");
    Connect();
    usleep(50000);

// Send parameters to the MCU board.
    SetMCUParameters();
/*
// Clear the encoder tick history.
    ClearEncDiffHistory();
*/
// Turn on the motor controllers.
    printf(" - turning on key switch\n");
    SetKeySwitch(true);
    usleep(50000);

// Enable status message transmission by the MCU board.
    printf(" - turning on status data transmission\n");
    SetDataTransmission(true);

// Enter the ROS main loop to activate the subscribers.
    ros::spin();

// After the main loop was killed, disable status message transmission by the MCU board.
    printf(" - turning off status data transmission\n");
    SetDataTransmission(false);
    usleep(50000);

// Turn off the motor controllers.
    printf(" - turning off key switch\n");
    SetKeySwitch(false);
    usleep(50000);

// Close the communication port to the MCU board.
    printf(" - disconnecting from MCU\n");
    Disconnect();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// Entry and exit point of the program.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <O.K.>
// Main routine of the node 'MobilePlatform'. Does only the setup and
// cleanup, but leaves the rest to ROS.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char** argv)
{
  char Filename[128], c;
  unsigned int u;
  cMobilePlatform *pMCU;

// Initially there are no resources allocated
  pMCU = 0;

// Get the own filename and create the name for the config file out of it.
  u = 0;
  while (1)
  {
    c = argv[0][u];
    Filename[u] = c;
    if (c == 0) break;
    else u++;
  }
  Filename[u++] = '.';
  Filename[u++] = 'x';
  Filename[u++] = 'm';
  Filename[u++] = 'l';
  Filename[u++] = 0;

  try
  {

// Create an instance of the class 'cMobilePlatform' that interfaces with the platform's MCU board.
    pMCU = new (std::nothrow) cMobilePlatform(1, Filename);
    if (pMCU == 0)
      throw cSysCallException(0, "main()", "Error.SysCall.new", errno);

// Enter the main loop - between starting and stopping the MCU board, this will stay at ROS spin. 
    pMCU->run(argc, argv);
  }
  catch (cSysCallException e) {e.Print();}
  catch (cXmlException e) {e.Print();}
  catch (cSystemException e) {e.Print();}
  catch (cException e) {e.Print();}

// Remove the instance of the class 'cMobilePlatform'.
  printf("main(): Performing cleanup...\n");
  if (pMCU != 0)
  {
    printf("main(): Removing instance of type 'cMobilePlatform'.\n");
    delete pMCU;
    pMCU = 0;
  }

// Leave the program.
  return 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

