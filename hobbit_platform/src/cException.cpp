//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 25.12.2009
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/hobbit_platform/cException.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// The class 'cException'.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The constructor of the class 'cException' has these arguments:
// (1) The ID of the instance that has generated the exception.
// (2) A string with the name of the method that wants to report an
//     error, format 'ClassName::MethodName()'.
// (3) A 'structured string' that describes the error, for example
//     'Error.Port.Open'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cException::cException(uint32_t ID, const char *Method, const char *Error)
{
  const char *Default = "not available";

// Locally store the ID of the instance that has generated this exception.
  InstanceID = ID;

// If the NULL pointer was provided as pointer to the method name, use the default.
  if (Method == (const char *)NULL) MethodName = Default;
  else MethodName = Method;

// If the NULL pointer was provided as pointer to the error name, use a default.
  if (Error == (const char *)NULL) ErrorName = Default;
  else ErrorName = Error;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the ID of the instance that has generated the exception.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cException::GetInstanceID(void)
{
  return InstanceID;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns a pointer to the ASCIIZ string with the name of the method
// that has generated the exception.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const char *cException::GetMethodName(void)
{
  return MethodName.c_str();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns a pointer to the structured ASCIIZ string that describes
// the error to be reported.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const char *cException::GetErrorName(void)
{
  return ErrorName.c_str();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Prints the contents of the exception's attributes to the screen.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cException::Print(void)
{
  printf("\n*** cException ***\n");
  printf("%s [%u]: %s\n\n", MethodName.c_str(), InstanceID, ErrorName.c_str());
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// The class 'cSysCallException'.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The constructor of the class 'cSysCallException' has these arguments:
// (1) The ID of the instance that has generated the exception.
// (2) A string with the name of the method that wants to report an
//     error, format 'ClassName::MethodName()'.
// (3) A 'structured string' that describes the error, for example
//     'Error.Port.Open'.
// (4) A copy of errno's current value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cSysCallException::cSysCallException(uint32_t ID, const char *Method, const char *Error, int ErrNo) :
                   cException(ID, Method, Error)
{

// Locally store the current value of 'errno'.
  ErrorNumber = ErrNo;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// If the error to be reported is the failure of a system call, then
// this method returns the associated value of 'errno'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int cSysCallException::GetErrorNumber(void)
{
  return ErrorNumber;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Prints the contents of the exception's attributes to the screen.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cSysCallException::Print(void)
{
  printf("\n*** cSysCallException ***\n");
  printf("%s [%u]: %s\n", MethodName.c_str(), InstanceID, ErrorName.c_str());
  printf("errno: %i (%s)\n\n", ErrorNumber, strerror(ErrorNumber));
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// The class 'cLibCallException'.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The constructor of the class 'cLibCallException' has these arguments:
// (1) The ID of the instance that has generated the exception.
// (2) A string with the name of the method that wants to report an
//     error, format 'ClassName::MethodName()'.
// (3) A 'structured string' that describes the error, for example
//     'Error.Port.Open'.
// (4) The error code reported by the library call.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cLibCallException::cLibCallException(uint32_t ID, const char *Method, const char *Error, int ErrNo) :
                   cException(ID, Method, Error)
{

// Locally store the error code reported by the library call.
  ErrorNumber = ErrNo;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the error code reported by the library call.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int cLibCallException::GetErrorNumber(void)
{
  return ErrorNumber;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Prints the contents of the exception's attributes to the screen.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cLibCallException::Print(void)
{
  printf("\n*** cLibCallException ***\n");
  printf("%s [%u]: %s\n", MethodName.c_str(), InstanceID, ErrorName.c_str());
  printf("error code: %i\n\n", ErrorNumber);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// The class 'cConfigFileException'.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The constructor of the class 'cConfigFileException' has these arguments:
// (1) The ID of the instance that has generated the exception.
// (2) A string with the name of the method that wants to report an
//     error, format 'ClassName::MethodName()'.
// (3) A 'structured string' that describes the error, for example
//     'Error.Port.Open'.
// (4) The file line number that was last read.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cConfigFileException::cConfigFileException(uint32_t ID, const char *Method, const char *Error,
                      uint32_t LineNumber) : cException(ID, Method, Error)
{

// Locally store the file line number.
  FileLineNumber = LineNumber;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the file line number.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cConfigFileException::GetFileLineNumber(void)
{
  return FileLineNumber;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Prints the contents of the exception's attributes to the screen.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cConfigFileException::Print(void)
{
  printf("\n*** cConfigFileException ***\n");
  printf("%s [%u]: %s\n", MethodName.c_str(), InstanceID, ErrorName.c_str());
  printf("file line number: %i\n\n", FileLineNumber);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
