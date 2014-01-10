//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 22.3.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/MobilePlatform/cRs232Port.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The literal baud rate is converted into a LINUX-specific constant
// that is stored in the attribute 'NativeBaudRate'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cRs232Port::ConvertBaudRate(void)
{
  if (BaudRate == 0) NativeBaudRate = B0;
  else if (BaudRate == 50) NativeBaudRate = B50;
  else if (BaudRate == 75) NativeBaudRate = B75;
  else if (BaudRate == 110) NativeBaudRate = B110;
  else if (BaudRate == 134) NativeBaudRate = B134;
  else if (BaudRate == 150) NativeBaudRate = B150;
  else if (BaudRate == 200) NativeBaudRate = B200;
  else if (BaudRate == 300) NativeBaudRate = B300;
  else if (BaudRate == 600) NativeBaudRate = B600;
  else if (BaudRate == 1200) NativeBaudRate = B1200;
  else if (BaudRate == 1800) NativeBaudRate = B1800;
  else if (BaudRate == 2400) NativeBaudRate = B2400;
  else if (BaudRate == 4800) NativeBaudRate = B4800;
  else if (BaudRate == 9600) NativeBaudRate = B9600;
  else if (BaudRate == 19200) NativeBaudRate = B19200;
  else if (BaudRate == 38400) NativeBaudRate = B38400;
  else if (BaudRate == 57600) NativeBaudRate = B57600;
  else if (BaudRate == 115200) NativeBaudRate = B115200;
#ifdef B230400
  else if (BaudRate == 230400) NativeBaudRate = B230400;
#endif
  else throw cException(ID, "cRs232Port::ConvertBaudRate()", "Error.Invalid.BaudRate");
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The literal number of data bits is converted into a LINUX-specific
// constant and stored in the attribute 'DataBitCount'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cRs232Port::ConvertDataBitCount(void)
{
  if (DataBits == 5) DataBitCount = CS5;
  else if (DataBits == 6) DataBitCount = CS6;
  else if (DataBits == 7) DataBitCount = CS7;
  else if (DataBits == 8) DataBitCount = CS8;
  else throw cException(ID, "cRs232Port::ConvertDataBitCount()", "Error.Invalid.DataBitCount");
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the baud rate for the RS232 port.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cRs232Port::SetBaudRate(void)
{

// Write the output baud rate to the parameter block.
  if (cfsetospeed(&tio, NativeBaudRate) == -1)
    throw cSysCallException(ID, "cRs232Port::SetBaudRate()", "Error.SysCall.cfsetospeed()", errno);

// Write the input baud rate to the parameter block.
  if (cfsetispeed(&tio, NativeBaudRate) == -1)
    throw cSysCallException(ID, "cRs232Port::SetBaudRate()", "Error.SysCall.cfsetispeed()", errno);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the number of bits in a data word.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cRs232Port::SetDataBitCount(void)
{
// Set the number of data bits.
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= DataBitCount;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the number of stop bits.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cRs232Port::SetStopBitCount(void)
{
  if (HalfStopBits == 2) tio.c_cflag &= ~CSTOPB;
  else if (HalfStopBits == 4) tio.c_cflag |= CSTOPB;
  else throw cException(ID, "cRs232Port::SetStopBitCount()", "Error.Invalid.StopBitCount");
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Turns parity generation on or off and sets even or odd parity.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cRs232Port::SetParity(void)
{
// If parity is used...
  if (bUseParity)
  {
    tio.c_cflag |= PARENB;

// Set even or odd parity.
    if (bOddParity) tio.c_cflag |= PARODD;
    else tio.c_cflag &= ~PARODD;
  }

// If parity is not used...
  else tio.c_cflag &= ~PARENB;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enables or disables timeout and sets the timeout's value, that has
// a resolution of deciseconds (100ms).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cRs232Port::SetTimeout(void)
{
  tio.c_cc[VTIME] = (unsigned int)(ReadDefaultTimeout / 100);
  if ((ReadDefaultTimeout % 100) >= 50) tio.c_cc[VTIME]++;
  tio.c_cc[VMIN] = 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// If 'bUseHardwareControl' is 'true' then the hardware flow control
// for the RS232 port will be enabled, otherwise disabled.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cRs232Port::SetHardwareControl(bool bUseHardwareControl)
{
// Conditionally set the hardware control bit.
  if (bUseHardwareControl) tio.c_cflag |= CRTSCTS;
  else tio.c_cflag &= ~CRTSCTS;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets some details of the RS232 port's mode of operation.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cRs232Port::SetMode(void)
{
// Turn off echo, canonical mode, extended processing, signals.
  tio.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

// Turn off break sig, cr->nl, 8 bit strip, flow control.
  tio.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

// Turn output processing off.
  tio.c_oflag &= ~OPOST;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Frees all resources that have been allocated.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cRs232Port::Cleanup(void)
{

// Only do something if there's a valid communication device file descriptor.
  if (hRS232 != -1)
  {

// If not, wait until all data is written out...
    (void)tcdrain(hRS232);
    (void)Close();
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method reads the configuration data from an XML-file by
// making use of the instance of the class 'cXML' that is provided
// as argument.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cRs232Port::Configuration(void)
{
  struct SimpleElement pAttr[9] = {{"PortName", &PortName, ELEMENT_STRING, false, 15},
                                   {"BaudRate", &BaudRate, ELEMENT_UINT32, false, 1},
                                   {"DataBits", &DataBits, ELEMENT_UINT32, false, 1},
                                   {"HalfStopBits", &HalfStopBits, ELEMENT_UINT32, false, 1},
                                   {"bUseParity", &bUseParity, ELEMENT_BOOLEAN, false, 1},
                                   {"bOddParity", &bOddParity, ELEMENT_BOOLEAN, false, 1},
                                   {"ConnectDefaultTimeout", &ConnectDefaultTimeout, ELEMENT_UINT32, false, 1},
                                   {"ReadDefaultTimeout", &ReadDefaultTimeout, ELEMENT_UINT32, false, 1},
                                   {"WriteDefaultTimeout", &WriteDefaultTimeout, ELEMENT_UINT32, false, 1}};
  uint32_t u;

// Initialise the instance's attributes.
  for (u = 0; u < 9; u++) pAttr[u].bInitialised = false;
  pSystem->ParseElement(pAttr, 9);

// Set up the RS232 port parameters.
  ConvertBaudRate();
  ConvertDataBitCount();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The constructor of the class 'cRs232Port' prepares everything for
// opening an RS232 port.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cRs232Port::cRs232Port(uint32_t InstanceID, cSystem *pConfig) : cCommunicationPort(InstanceID, pConfig)
{

// Initially there are no resources allocated.
  hRS232 = -1;

// Read the configuration data from the XML-file.
  Configuration();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The destructor of the class 'cRs232Port' flushes the RS232 port's
// output buffer and closes the port's descriptor, but only, if it
// was successfully opened before.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cRs232Port::~cRs232Port()
{

// Do the standard cleanup but ignore any errors that might have occured.
  Cleanup();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// After an error has occured and a higher instance wants to restart,
// this method is to be called in order to get rid of leftovers.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cRs232Port::Reset(void)
{
// If there is still an open port, try to close it, but ignore the result.
  if (hRS232 != -1) Close();

// Clear the handle of the previously open port.
  hRS232 = -1;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Opens the specified RS232 port and does the setup.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cRs232Port::Open(uint32_t Timeout)
{

// Check if the port is already open.
  if (hRS232 != -1)
    throw cException(ID, "cRs232Port::Open", "Error.Already.Open");

// Open the specified RS232 port.
  hRS232 = open(PortName, O_RDWR | O_NDELAY);
  if (hRS232 == -1)
    throw cSysCallException(ID, "cRs232Port::Open", "Error.SysCall.open()", errno);

// FNDELAY...non-blocking, 0...blocking (used).
  if (fcntl(hRS232, F_SETFL, 0) == -1)
    throw cSysCallException(ID, "cRs232Port::Open", "Error.SysCall.fcntl()", errno);

// Get the open port's parameter block.
  if (tcgetattr(hRS232, &tio) == -1)
    throw cSysCallException(ID, "cRs232Port::Open", "Error.SysCall.tcgetattr()", errno);

// Set up some details concerning the port's mode of operation.
  SetMode();

// Turn on or off parity and set odd or even parity.
  SetParity();

// Set the number of data bits.
  SetDataBitCount();

// Set one or two stop bits.
  SetStopBitCount();

// Set timeout.
  SetTimeout();

// Set the baud rate.
  SetBaudRate();

// Disable hardware flow control.
  SetHardwareControl(false);

// Set the open port's parameter block.
  if (tcsetattr(hRS232, TCSAFLUSH, &tio) == -1)
    throw cSysCallException(ID, "cRs232Port::Open", "Error.SysCall.tcsetattr()", errno);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Closes the RS232 port, in case it is open.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cRs232Port::Close(void)
{

// Check if the RS232 port is open.
  if (hRS232 == -1)
    throw cException(ID, "cRs232Port::Close", "Error.Not.Open");

// Wait until all data has been written out.
  if (tcdrain(hRS232) == -1)
    throw cSysCallException(ID, "cRs232Port::Close", "Error.SysCall.tcdrain()", errno);

// If so, close it.
  if (close(hRS232) == -1)
    throw cSysCallException(ID, "cRs232Port::Close", "Error.SysCall.close()", errno);
  hRS232 = -1;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Reads data from the RS232 port and stores it in the buffer 'Data'.
// 'ReadCount' specifies the maximum number of bytes to be read.
// The number of bytes read is returned. In case this number is '0',
// a timeout has occured.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cRs232Port::Read(uint8_t *Data, uint32_t ReadCount, uint32_t Timeout)
{

// Check if the port is open at all.
  if (hRS232 == -1)
    throw cException(ID, "cRs232Port::Read", "Error.Not.Open");

// Check the validity of the pointer to the destination buffer.
  if (Data == 0)
    throw cException(ID, "cRs232Port::Read", "Error.Zero.Pointer");

// Check if 'ReadCount' is valid.
  if (ReadCount == 0)
    throw cException(ID, "cRs232Port::Read", "Error.Zero.Count");

// Read data from the input port's queue.
  int RetVal = read(hRS232, (void *)Data, (size_t)ReadCount);
  if (RetVal == -1)
    throw cSysCallException(ID, "cRs232Port::Read", "Error.SysCall.read()", errno);
  return (uint32_t)RetVal;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Reads exactly N bytes from the RS232 port and stores them in the
// buffer 'Data'. 'ReadCount' specifies the number of bytes to be
// read, and 'Timeout', after how many milliseconds all N bytes must
// have been read at latest. ReadCount will contain the number of
// bytes successfully read.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cRs232Port::ReadNBytes(uint8_t *Data, uint32_t ReadCount, uint32_t Timeout)
{

// Check the timeout.
  if (Timeout == 0xffffffff) Timeout = ReadDefaultTimeout;

// Check if 'ReadCount' is valid.
  if (ReadCount == 0)
    throw cException(ID, "cRs232Port::ReadNBytes", "Error.Zero.Count");

// Check if the port is open at all.
  if (hRS232 == -1)
    throw cException(ID, "cRs232Port::ReadNBytes", "Error.Not.Open");

// Check the validity of the pointer to the destination buffer.
  if (Data == 0)
    throw cException(ID, "cRs232Port::ReadNBytes", "Error.Zero.Pointer");

// Calculate the deadline for reading from the port.
  int64_t Deadline = GetTimestampMs();
  Deadline = Deadline + (int64_t)Timeout;

// Read from the port until all data is there or the until the deadline.
  int RetVal;
  uint32_t BytesRead = 0;
  while (GetTimestampMs() < Deadline)
  {

// Read data from the port.
    RetVal = read(hRS232, (void *)(&Data[BytesRead]), (size_t)(ReadCount - BytesRead));
    if (RetVal == -1)
      throw cSysCallException(ID, "cRs232Port::ReadNBytes", "Error.SysCall.read()", errno);
    BytesRead = BytesRead + (uint32_t)RetVal;

// Check if the desired number of bytes has already been read.
    if (BytesRead == ReadCount) break;
  }

// Return the number of bytes successfully read.
  return BytesRead;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Writes the data contained in the buffer 'Data' to the RS232 port.
// This method tries to write all data to the port or the port's buf-
// fer, respectively. This means that the method loops writing for
// as long as there's unwritten data. The return value is the number
// of bytes successfully written.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cRs232Port::Write(uint8_t *Data, uint32_t WriteCount, uint32_t Timeout)
{

// Check if the port is open at all.
  if (hRS232 == -1)
    throw cException(ID, "cRs232Port::Write", "Error.Not.Open");

// Check the validity of the pointer to the source buffer'.
  if (Data == 0)
    throw cException(ID, "cRs232Port::Write", "Error.Zero.Pointer");

// Check if 'WriteCount' is valid.
  if (WriteCount == 0)
    throw cException(ID, "cRs232Port::Write", "Error.Zero.Count");

// Loop for as long as there is unwritten data.
  int RetVal;
  uint32_t BytesWritten = 0;
  do
  {

// 'write()' returns the number of bytes written or '-1'.
    RetVal = write(hRS232, (const void *)(&Data[BytesWritten]), (size_t)(WriteCount - BytesWritten));
    if (RetVal == -1)
      throw cSysCallException(ID, "cRs232Port::Write", "Error.SysCall.write()", errno);
    BytesWritten = BytesWritten + (uint32_t)RetVal;
  } while (BytesWritten < WriteCount);

// Return the number of bytes successfully written.
  return BytesWritten;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the number of bytes available for reading.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cRs232Port::NumberOfBytesToRead(void)
{

// Check if the port is open at all.
  if (hRS232 == -1)
    throw cException(ID, "cRs232Port::NumberOfBytesToRead", "Error.Not.Open");

// Get the number of bytes available for reading.
  unsigned int BytesAvailable;
  if (ioctl(hRS232, FIONREAD, &BytesAvailable) == -1)
    throw cSysCallException(ID, "cRs232Port::NumberOfBytesToRead", "Error.SysCall.ioctl()", errno);

// Return the number of bytes available for reading.
  return BytesAvailable;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method executes a port-specific command such as setting and
// clearing RS-232 control lines. The command is specified in the form
// of an ASCII string. So is the result.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const char *cRs232Port::PortCommand(const char *Command)
{

// Check if the port is open at all.
  if (hRS232 == -1)
    throw cException(ID, "cRs232Port::PortCommand", "Error.Not.Open");

// Check the validity of the argument.
  if (Command == 0)
    throw cException(ID, "cRs232Port::PortCommand", "Error.Zero.Pointer");

// Command 'clear control line'.
  int Flags;
  if (SubstringCompare(Command, "Clr ") == 0)
  {
    if (StringCompare((Command + 4), "DTR") == 0) Flags = ~TIOCM_DTR;
    else if (StringCompare((Command + 4), "RTS") == 0) Flags = ~TIOCM_RTS;
    else throw cException(ID, "cRs232Port::PortCommand", "Error.Invalid.Command");

// Clear the control line 'DTR' or 'RTS'.
    if (ioctl(hRS232, TIOCMBIC, &Flags) == -1)
      throw cSysCallException(ID, "cRs232Port::PortCommand", "Error.SysCall.ioctl()", errno);
  }

// Command 'get control line status'.
  else if (SubstringCompare(Command, "Get ") == 0)
  {
    int i;
    if (StringCompare((Command + 4), "CD") == 0) i = TIOCM_CAR;
    else if (StringCompare((Command + 4), "DSR") == 0) i = TIOCM_DSR;
    else if (StringCompare((Command + 4), "CTS") == 0) i = TIOCM_CTS;
    else if (StringCompare((Command + 4), "RI") == 0) i = TIOCM_RNG;
    else throw cException(ID, "cRs232Port::PortCommand", "Error.Invalid.Command");

// Get the status of the control line 'CD', 'DSR', 'CTS' or 'RI'.
    if (ioctl(hRS232, TIOCMGET, &Flags) == -1)
      throw cSysCallException(ID, "cRs232Port::PortCommand", "Error.SysCall.ioctl()", errno);
    Flags = Flags & i;
    if (Flags == 0) return "0";
    else return "1";
  }

// Command 'set control line'.
  else if (SubstringCompare(Command, "Set ") == 0)
  {
    if (StringCompare((Command + 4), "DTR") == 0) Flags = TIOCM_DTR;
    else if (StringCompare((Command + 4), "RTS") == 0) Flags = TIOCM_RTS;
    else throw cException(ID, "cRs232Port::PortCommand", "Error.Invalid.Command");

// Set the control line 'DTR' or 'RTS'.
    if (ioctl(hRS232, TIOCMBIS, &Flags) == -1)
      throw cSysCallException(ID, "cRs232Port::PortCommand", "Error.SysCall.ioctl()", errno);
  }
  else throw cException(ID, "cRs232Port::PortCommand", "Error.Invalid.Command");

// Just to suppress the compiler's warning...
  return 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

