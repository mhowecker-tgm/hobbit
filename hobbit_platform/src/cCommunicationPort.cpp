//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 23.1.2009
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/hobbit_platform/cCommunicationPort.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// Private methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Compares the two strings referenced by <pString1> and <pString2>.
// The return value is '-1' if <pString1> lies lexically before
// <pString2>, '1' if <pString1> lies lexically after <pString2>,
// and '0' if both strings are equal. Two strings can only be equal
// if they have the same length.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int32_t cCommunicationPort::StringCompare(const char *pString1, const char *pString2)
{

// Check the validity of the pointers to the input strings.
  if ((pString1 == (const char *)NULL) || (pString2 == (const char *)NULL))
    throw cException(ID, "cCommunicationPort::StringCompare()", "Error.Zero.Pointer");

// Compare both strings...
  uint32_t u = 0;
  char c;
  while (1)
  {
    c = pString2[u];
    if (pString1[u] == c)
    {

// ...taking into account that both must have the same length.
      if (c == 0x00) break;
      else u++;
    }
    else if (pString1[u] < c) return -1;
    else return 1;
  }
  return 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Compares the string referenced by <pString2> to the first part of
// the string <pString1>. The return value is '-1' if <pString1> lies
// lexically before <pString2>, '1' if <pString1> lies lexically after
// <pString2>, and '0' if both strings are equal.
// This method is intended for searching for a keyword (<pString2>)
// within a longer string (<pString1>).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int32_t cCommunicationPort::SubstringCompare(const char *pString1, const char *pString2)
{

// Check the validity of the pointers to the input strings.
  if ((pString1 == (const char *)NULL) || (pString2 == (const char *)NULL))
    throw cException(ID, "cCommunicationPort::SubstringCompare()", "Error.Zero.Pointer");

// Compare both strings.
  uint32_t u = 0;
  char c;
  while (1)
  {
    c = pString2[u];
    if (c == 0x00) break;
    else if (pString1[u] == c) u++;
    else if (pString1[u] < c) return -1;
    else return 1;
  }
  return 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// Public methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The constructor of the class 'cCommunicationPort' just stores the
// ID of this instance of the class and clears the error number for
// system calls.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cCommunicationPort::cCommunicationPort(uint32_t InstanceID, cSystem *pConfig)
{
// Locally store the ID of this instance of the class 'cCommunicationPort'.
  ID = InstanceID;
  pSystem = pConfig;

// Set all default timeouts to '0'.
  ConnectDefaultTimeout = 0;
  ReadDefaultTimeout = 0;
  WriteDefaultTimeout = 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the ID of the instance of the class 'cCommunicationPort'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cCommunicationPort::GetID(void)
{
  return ID;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
