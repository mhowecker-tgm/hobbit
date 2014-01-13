//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 31.1.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef C_RS232_PORT_HH
#define C_RS232_PORT_HH
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "cCommunicationPort.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The class 'cRs232Port' provides the funtionality that is required
// for initalising, reading from and writing to an RS232 port.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cRs232Port : public cCommunicationPort
{
  protected:
//-------------------------------------------------------------------
// Private attributes.
//-------------------------------------------------------------------
    char PortName[16];
    uint32_t BaudRate;
    uint32_t DataBits;
    uint32_t HalfStopBits;
    bool bUseParity;
    bool bOddParity;
    int hRS232;
    speed_t NativeBaudRate;
    tcflag_t DataBitCount;
    struct termios tio;
//-------------------------------------------------------------------
// Private methods.
//-------------------------------------------------------------------
    void ConvertBaudRate(void);
    void ConvertDataBitCount(void);
    void SetBaudRate(void);
    void SetDataBitCount(void);
    void SetStopBitCount(void);
    void SetParity(void);
    void SetTimeout(void);
    void SetHardwareControl(bool bUseHardwareControl);
    void SetMode(void);
    virtual void Cleanup(void);
    virtual void Configuration(void);
  public:
//-------------------------------------------------------------------
// Public methods.
//-------------------------------------------------------------------
    cRs232Port(uint32_t InstanceID, cSystem *pConfig);
    ~cRs232Port();
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

