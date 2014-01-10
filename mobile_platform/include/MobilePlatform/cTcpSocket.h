//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 23.1.2009
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef C_TCP_SOCKET_HH
#define C_TCP_SOCKET_HH
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <strings.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include "cCommunicationPort.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef NULL
#define NULL (void *)0
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The class 'cTcpSocket' provides the funtionality that is required
// for initalising, reading from and writing to a TCP socket.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cTcpSocket : public cCommunicationPort
{
  protected:
//-------------------------------------------------------------------
// Private attributes.
//-------------------------------------------------------------------
    char HostIP[16];
    char PortCommandResult[8];
    fd_set myfds;
    struct sockaddr_in host_addr;
    struct timeval tv;
    int s;
    uint16_t HostPort;
//-------------------------------------------------------------------
// Private methods.
//-------------------------------------------------------------------
    virtual void Cleanup(void);
    virtual void Configuration(void);
public:
//-------------------------------------------------------------------
// Public methods.
//-------------------------------------------------------------------
    cTcpSocket(uint32_t InstanceID, cSystem *pConfig);
    ~cTcpSocket();
    virtual void Reset(void);
    virtual void Open(uint32_t Timeout = 0xffffffff);
    virtual void Close(void);
    virtual uint32_t Read(uint8_t *Data, uint32_t ReadCount, uint32_t Timeout = 0xffffffff);
    virtual uint32_t ReadNBytes(uint8_t *Data, uint32_t ReadCount, uint32_t Timeout = 0xffffffff);
    virtual uint32_t Write(uint8_t *Data, uint32_t WriteCount, uint32_t Timeout = 0xffffffff);
    virtual uint32_t NumberOfBytesToRead(void);
    virtual const char *PortCommand(const char *Command);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
