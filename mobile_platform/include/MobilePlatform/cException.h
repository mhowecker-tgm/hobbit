//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 25.12.2009
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef C_EXCEPTION_HH
#define C_EXCEPTION_HH
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <string>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <stdio.h>
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef NULL
#define NULL (void *)0
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// The class 'cException' is the base class of all more specialised
// exception classes.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cException
{
  protected:
//-------------------------------------------------------------------
// Private attributes.
//-------------------------------------------------------------------
    uint32_t InstanceID;
    string MethodName;
    string ErrorName;

  public:
//-------------------------------------------------------------------
// Public methods.
//-------------------------------------------------------------------
    cException(uint32_t ID, const char *Method, const char *Error);
    uint32_t GetInstanceID(void);
    const char *GetMethodName(void);
    const char *GetErrorName(void);
    void Print(void);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// The class 'cSysCallException' is derived from the base class
// 'cException'. Such an exception is to be thrown if a system call
// reported an error and a retry makes no sense.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cSysCallException : public cException
{
  protected:
//-------------------------------------------------------------------
// Private attributes.
//-------------------------------------------------------------------
    int ErrorNumber;

  public:
//-------------------------------------------------------------------
// Public methods.
//-------------------------------------------------------------------
    cSysCallException(uint32_t ID, const char *Method, const char *Error, int ErrNo);
    int GetErrorNumber(void);
    void Print(void);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// The class 'cLibCallException' is derived from the base class
// 'cException'. Such an exception is to be thrown if a library call
// reported an error and a retry makes no sense.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cLibCallException : public cException
{
  protected:
//-------------------------------------------------------------------
// Private attributes.
//-------------------------------------------------------------------
    int ErrorNumber;

  public:
//-------------------------------------------------------------------
// Public methods.
//-------------------------------------------------------------------
    cLibCallException(uint32_t ID, const char *Method, const char *Error, int ErrNo);
    int GetErrorNumber(void);
    void Print(void);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// The class 'cConfigFileException' is derived from the base class
// 'cException'. Such an exception is to be thrown if an error was
// encountered when parsing the contents of a configuration file and a
// retry makes no sense.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cConfigFileException : public cException
{
  protected:
//-------------------------------------------------------------------
// Private attributes.
//-------------------------------------------------------------------
    uint32_t FileLineNumber;

  public:
//-------------------------------------------------------------------
// Public methods.
//-------------------------------------------------------------------
    cConfigFileException(uint32_t ID, const char *Method, const char *Error, uint32_t LineNumber);
    uint32_t GetFileLineNumber(void);
    void Print(void);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
