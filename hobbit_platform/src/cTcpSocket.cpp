//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 27.10.2010
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/hobbit_platform/cTcpSocket.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Frees all resources that have been allocated.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cTcpSocket::Cleanup(void)
{

// Only do something if there is a socket.
  if (s != -1) Close();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method reads the configuration data from an XML-file by
// making use of the instance of the class 'cXML' that is provided
// as argument.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cTcpSocket::Configuration(void)
{
  struct SimpleElement pAttr[5] = {{"HostIP", &HostIP, ELEMENT_STRING, false, 15},
                                   {"HostPort", &HostPort, ELEMENT_UINT16, false, 1},
                                   {"ConnectDefaultTimeout", &ConnectDefaultTimeout, ELEMENT_UINT32, false, 1},
                                   {"ReadDefaultTimeout", &ReadDefaultTimeout, ELEMENT_UINT32, false, 1},
                                   {"WriteDefaultTimeout", &WriteDefaultTimeout, ELEMENT_UINT32, false, 1}};

// Initialise the instance's attributes.
  for (uint32_t u = 0; u < 5; u++) pAttr[u].bInitialised = false;
  pSystem->ParseElement(pAttr, 5);

// Set up the host address.
  host_addr.sin_family = AF_INET;
  host_addr.sin_port = htons(HostPort);
  if (inet_pton(AF_INET, HostIP, &host_addr.sin_addr) <= 0)
    throw cSysCallException(ID, "cTcpSocket::Configuration()", "Error.SysCall.inet_pton", errno);
  bzero(&(host_addr.sin_zero), 8);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The constructor of the class 'cTcpSocket' allocates and configures
// the required resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cTcpSocket::cTcpSocket(uint32_t InstanceID, cSystem *pConfig) : cCommunicationPort(InstanceID, pConfig)
{

// Initially there are no resources allocated.
  s = -1;

// Read the configuration data from the XML-file.
  Configuration();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The destructor of the class 'cTcpSocket' frees all resources that
// have been allocated.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cTcpSocket::~cTcpSocket()
{

// Do the standard cleanup but ignore any errors that might have occured.
  Cleanup();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// After an error has occured and a higher instance wants to restart,
// this method is to be called in order to get rid of leftovers.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cTcpSocket::Reset(void)
{

// If there is still an open socket, try to close it, but ignore the result.
  if (s != -1) Close();

// Clear the handle of the previously open socket.
  s = -1;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Requests a TCP socket and connects to a host. If this method is
// invoked without specifying a timeout (in ms), the default timeout
// (as defined in the configuration file) is being used instead.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cTcpSocket::Open(uint32_t Timeout)
{
  socklen_t Length;
  uint32_t u;
  int Temp, RetVal, SockFlags;

// Check if there is already a socket.
  if (s != -1)
    throw cException(ID, "cTcpSocket::Open", "Error.Already.Open");

// Creation of a stream socket. If something happens here, no cleanup's needed.
  s = socket(AF_INET, SOCK_STREAM, 0);
  if (s == -1)
    throw cSysCallException(ID, "cTcpSocket::Open", "Error.SysCall.socket()", errno);

// Make the socket non-blocking. If something happens here, do complete cleanup.
  SockFlags = fcntl(s, F_GETFL);
  if (SockFlags == -1)
  {
    Temp = errno;
    Reset();
    throw cSysCallException(ID, "cTcpSocket::Open", "Error.SysCall.fcntl()", Temp);
  }
  if (fcntl(s, F_SETFL, (SockFlags | O_NONBLOCK)) == -1)
  {
    Temp = errno;
    Reset();
    throw cSysCallException(ID, "cTcpSocket::Open", "Error.SysCall.fcntl()", Temp);
  }

// Connect to the host.
  if (connect(s, (sockaddr *)&host_addr, sizeof(host_addr)) == -1)
  {

// In case of an error, do a complete cleanup.
    if (errno != EINPROGRESS)
    {
      Temp = errno;
      Reset();
      throw cSysCallException(ID, "cTcpSocket::Open", "Error.SysCall.connect()", Temp);
    }

 // If not immediately connected, wait (at latest until timeout).
    else
    {
      FD_ZERO(&myfds);
      FD_SET(s, &myfds);

// Calculate the timeout.
      if (Timeout == 0xffffffff) Timeout = ConnectDefaultTimeout;
      u = Timeout / 1000;
      tv.tv_sec = (time_t)u;
      u = (Timeout % 1000) * 1000;
      tv.tv_usec = (suseconds_t)u;

// Wait for the connection to be established.
      RetVal = select((s + 1), NULL, &myfds, NULL, &tv);
      if (RetVal == -1)
      {

// In case of an error, do a complete cleanup.
        Temp = errno;
        Reset();
        throw cSysCallException(ID, "cTcpSocket::Open", "Error.SysCall.select()", Temp);
      }

// At this point we have run into a timeout - do a complete cleanup.
      else if (RetVal == 0)
      {
        Temp = errno;
        Reset();
        throw cSysCallException(ID, "cTcpSocket::Open", "Warning.Timeout.Connect", Temp);
      }

// Check if there has been an error at socket level.
      Length = sizeof(Temp);
      if (getsockopt(s, SOL_SOCKET, SO_ERROR, (void *)(&Temp), &Length) == -1)
      {

// In case of an error, do a complete cleanup.
        Temp = errno;
        Reset();
        throw cSysCallException(ID, "cTcpSocket::Open", "Error.SysCall.getsockopt()", Temp);
      }

// In case of an error at socket level, do a complete cleanup.
      if (Temp != 0)
      {
        Temp = errno;
        Reset();
        throw cSysCallException(ID, "cTcpSocket::Open", "Error.TCPlevel", Temp);
      }
    }
  }

// Make the socket blocking again.
  if (fcntl(s, F_SETFL, (SockFlags & ~O_NONBLOCK)) == -1)
  {

// In case of an error, do a complete cleanup.
    Temp = errno;
    Reset();
    throw cSysCallException(ID, "cTcpSocket::Open", "Error.SysCall.fcntl()", Temp);
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Closes the connection.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cTcpSocket::Close(void)
{

// If there is no socket, just issue a warning.
  if (s == -1)
    throw cException(ID, "cTcpSocket::Close", "Error.Not.Open");

// Otherwise, close the socket.
  if (close(s) == -1)
    throw cSysCallException(ID, "cTcpSocket::Close", "Error.SysCall.close()", errno);
  s = -1;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Checks via select() if there is data to be read on the socket.
// This is done using a read timeout. If data is available, the data
// is stored in the buffer and the number of bytes read is returned.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cTcpSocket::Read(uint8_t *Data, uint32_t ReadCount, uint32_t Timeout)
{

// Check if the port is open at all.
  if (s == -1)
    throw cException(ID, "cTcpSocket::Read", "Error.Not.Open");

// Check the validity of the pointer to the destination buffer.
  if (Data == (uint8_t *)NULL)
    throw cException(ID, "cTcpSocket::Read", "Error.Zero.Pointer");

// Check if the read count is zero.
  if (ReadCount == 0)
    throw cException(ID, "cTcpSocket::Read", "Error.Zero.Count");

// Check if there is data to read (with timeout).
  FD_ZERO(&myfds);
  FD_SET(s, &myfds);

// Calculate the timeout.
  if (Timeout == 0xffffffff) Timeout = ReadDefaultTimeout;
  tv.tv_sec = (time_t)(Timeout / 1000);
  tv.tv_usec = (suseconds_t)((Timeout % 1000) * 1000);

// Wait for the readability of the socket.
  int RetVal = select((s + 1), &myfds, NULL, NULL, &tv);
  if (RetVal == -1)
    throw cSysCallException(ID, "cTcpSocket::Read", "Error.SysCall.select()", errno);

// Check if there has been a timeout.
  else if (RetVal == 0) return 0;

// Check if there has been an error at TCP level.
  int Temp;
  socklen_t Length = sizeof(Temp);
  if (getsockopt(s, SOL_SOCKET, SO_ERROR, (void *)(&Temp), &Length) == -1)
    throw cSysCallException(ID, "cTcpSocket::Read", "Error.SysCall.getsockopt()", errno);
  if (Temp != 0)
    throw cSysCallException(ID, "cTcpSocket::Read", "Error.TCPlevel", Temp);

// If neither of both is true, there must be data for reading.
  RetVal = recv(s, (void *)Data, ReadCount, 0);
  if (RetVal == -1)
    throw cSysCallException(ID, "cTcpSocket::Read", "Error.SysCall.recv()", errno);

// Check if the connection has been closed.
  if (RetVal == 0)
    throw cSysCallException(ID, "cTcpSocket::Read", "Warning.Connection.Closed", errno);
  return (uint32_t)RetVal;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Reads exactly N bytes from the TCP socket and stores them in the
// buffer 'Data'. 'ReadCount' specifies the number of bytes to be
// read, and 'Timeout' after how many milliseconds all N bytes must
// have been read at latest. ReadCount will contain the number of
// bytes successfully read.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cTcpSocket::ReadNBytes(uint8_t *Data, uint32_t ReadCount, uint32_t Timeout)
{

// Check if the port is open at all.
  if (s == -1)
    throw cException(ID, "cTcpSocket::ReadNBytes", "Error.Not.Open");

// Check the validity of the pointer to the destination buffer.
  if (Data == (uint8_t *)NULL)
    throw cException(ID, "cTcpSocket::ReadNBytes", "Error.Zero.Pointer");

// Check if 'ReadCount' is valid.
  if (ReadCount == 0)
    throw cException(ID, "cTcpSocket::ReadNBytes", "Error.Zero.Count");

// Calculate the timeout and deadline for reading from the port.
  uint32_t TOs = ReadDefaultTimeout / 1000;
  uint32_t TOus = (ReadDefaultTimeout % 1000) * 1000;
  if (Timeout == 0xffffffff) Timeout = ReadDefaultTimeout;
  int64_t Deadline = GetTimestampMs();
  Deadline = Deadline + (int64_t)Timeout;

// Read from the port until the all data is there or the until the deadline.
  socklen_t Length;
  int Temp, RetVal;
  uint32_t BytesRead = 0;
  while (GetTimestampMs() < Deadline)
  {

// Check if there is data to read (with timeout).
    FD_ZERO(&myfds);
    FD_SET(s, &myfds);

// Set the timeout.
    tv.tv_sec = (time_t)TOs;
    tv.tv_usec = (suseconds_t)TOus;

// Wait for the readability of the socket.
    RetVal = select((s + 1), &myfds, NULL, NULL, &tv);
    if (RetVal == -1)
      throw cSysCallException(ID, "cTcpSocket::ReadNBytes", "Error.SysCall.select()", errno);

//  If there has been a timeout, loop.
    else if (RetVal == 0) continue;

// Check if there has been an error at TCP level.
    Length = sizeof(Temp);
    if (getsockopt(s, SOL_SOCKET, SO_ERROR, (void *)(&Temp), &Length) == -1)
      throw cSysCallException(ID, "cTcpSocket::ReadNBytes", "Error.SysCall.getsockopt()", errno);
    if (Temp != 0)
      throw cSysCallException(ID, "cTcpSocket::ReadNBytes", "Error.TCPlevel", Temp);

// If neither of both is true, there must be data for reading.
    RetVal = recv(s, (void *)(&Data[BytesRead]), (size_t)(ReadCount - BytesRead), 0);
    if (RetVal == -1)
      throw cSysCallException(ID, "cTcpSocket::ReadNBytes", "Error.SysCall.recv()", errno);

// Check if the connection has been closed.
    else if (RetVal == 0)
      throw cSysCallException(ID, "cTcpSocket::ReadNBytes", "Warning.Connection.Closed", errno);
    BytesRead = BytesRead + (uint32_t)RetVal;

// Check if the desired number of bytes has already been read.
    if (BytesRead == ReadCount) break;
  }

// Return the number of bytes that have been read in total.
  return BytesRead;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Writes data to the socket. If the message can't be sent immediately
// the method blocks. The return value is the number of bytes that
// has been sent.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cTcpSocket::Write(uint8_t *Data, uint32_t WriteCount, uint32_t Timeout)
{

// Check if the port is open at all.
  if (s == -1)
    throw cException(ID, "cTcpSocket::Write", "Error.Not.Open");

// Check the validity of the pointer to the source buffer'.
  if (Data == (uint8_t *)NULL)
    throw cException(ID, "cTcpSocket::Write", "Error.Zero.Pointer");

// Check if 'WriteCount' is valid.
  if (WriteCount == 0)
    throw cException(ID, "cTcpSocket::Write", "Error.Zero.Count");

// Check the writeability of the socket.
  FD_ZERO(&myfds);
  FD_SET(s, &myfds);

// Calculate the timeout.
  if (Timeout == 0xffffffff) Timeout = WriteDefaultTimeout;
  tv.tv_sec = (time_t)(Timeout / 1000);
  tv.tv_usec = (suseconds_t)((Timeout % 1000) * 1000);

// Wait for the writeability of the socket.
  int RetVal = select((s + 1), NULL, &myfds, NULL, &tv);
  if (RetVal == -1)
    throw cSysCallException(ID, "cTcpSocket::Write", "Error.SysCall.select()", errno);
  else if (RetVal == 0)
    throw cException(ID, "cTcpSocket::Write", "Warning.Timeout.Write");

// Check if there has been an error at TCP level.
  int Temp;
  socklen_t Length = sizeof(Temp);
  if (getsockopt(s, SOL_SOCKET, SO_ERROR, (void *)(&Temp), &Length) == -1)
    throw cSysCallException(ID, "cTcpSocket::Write", "Error.SysCall.getsockopt()", errno);
  if (Temp != 0)
    throw cSysCallException(ID, "cTcpSocket::Write", "Error.TCPlevel", Temp);

// Write the data to the socket (writing to a socket is atomical).
  RetVal = send(s, (void *)Data, WriteCount, 0);
  if (RetVal == -1)
    throw cSysCallException(ID, "cTcpSocket::Write", "Error.SysCall.send()", errno);
  return (uint32_t)RetVal;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the number of bytes available for reading.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cTcpSocket::NumberOfBytesToRead(void)
{

// Check if the port is open at all.
  if (s == -1)
    throw cException(ID, "cTcpSocket::NumberOfBytesToRead", "Error.Not.Open");

// Get the number of bytes available for reading.
  unsigned int BytesAvailable;
  if (ioctl(s, FIONREAD, &BytesAvailable) == -1)
    throw cSysCallException(ID, "cTcpSocket::NumberOfBytesToRead", "Error.SysCall.ioctl()", errno);

// Return the number of bytes available for reading.
  return (uint32_t)BytesAvailable;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method executes a port-specific command. The command is spe-
// cified in the form of an ASCII string. So is the result.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const char *cTcpSocket::PortCommand(const char *Command)
{

// Check the validity of the argument.
  if (Command == (const char *)NULL)
    throw cException(ID, "cTcpSocket::PortCommand", "Error.Zero.Pointer");

// Check if it's the command 'GetHostIP'.
  if (StringCompare(Command, "GetHostIP") == 0)
    return (const char *)HostIP;

// Check if it's the command 'GetHostPort'.
  else if (StringCompare(Command, "GetHostPort") == 0)
  {
    sprintf(PortCommandResult, "%hu", HostPort);
    return (const char *)PortCommandResult;
  }

// Here it's sure that it was an invalid command.
  else throw cException(ID, "cTcpSocket::PortCommand", "Error.Invalid.Command");
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
