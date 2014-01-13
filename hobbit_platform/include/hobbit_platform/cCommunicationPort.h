//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 23.1.2009
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef C_COMMUNICATION_PORT_HH
#define C_COMMUNICATION_PORT_HH
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <new>
#include <inttypes.h>
#include <errno.h>
#include "cException.h"
#include "cTimestamping.h"
#include "cSystem.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef NULL
#define NULL (void *)0
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The class 'cCommunicationPort' is an abstract class that defines
// the interface of general communication hardware. Special implemen-
// tations derived from this class are e.g. for an RS232 port or for
// an TCP socket.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cCommunicationPort
{
  protected:
//-------------------------------------------------------------------
// Private attributes.
//-------------------------------------------------------------------
    uint32_t ID;
    cSystem *pSystem;
    uint32_t ConnectDefaultTimeout;
    uint32_t ReadDefaultTimeout;
    uint32_t WriteDefaultTimeout;
//-------------------------------------------------------------------
// Private methods.
//-------------------------------------------------------------------
    int32_t StringCompare(const char *pString1, const char *pString2);
    int32_t SubstringCompare(const char *pString1, const char *pString2);
    virtual void Cleanup(void) = 0;
    virtual void Configuration(void) = 0;
  public:
//-------------------------------------------------------------------
// Public methods.
//-------------------------------------------------------------------
    cCommunicationPort(uint32_t InstanceID, cSystem *pConfig);
    virtual ~cCommunicationPort() {};
    uint32_t GetID(void);
    virtual void Reset(void) = 0;
    virtual void Open(uint32_t Timeout = 0xffffffff) = 0;
    virtual void Close(void) = 0;
    virtual uint32_t Read(uint8_t *Data, uint32_t ReadCount, uint32_t Timeout = 0xffffffff) = 0;
    virtual uint32_t ReadNBytes(uint8_t *Data, uint32_t ReadCount, uint32_t Timeout = 0xffffffff) = 0;
    virtual uint32_t Write(uint8_t *Data, uint32_t WriteCount, uint32_t Timeout = 0xffffffff) = 0;
    virtual uint32_t NumberOfBytesToRead(void) = 0;
    virtual const char *PortCommand(const char *Command) = 0;
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
