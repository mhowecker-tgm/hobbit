//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 5.8.2009
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/hobbit_platform/cTimestamping.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Definition of a function that returns a timestamp (milliseconds
// that have elapsed since 1.1.1970).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int64_t GetTimestampMs(void)
{
  struct timeval tv;
  int64_t TheTime;

  gettimeofday(&tv, (struct timezone *)NULL);
  TheTime = ((int64_t)tv.tv_sec * 1000) + (int64_t)(tv.tv_usec / 1000);
  return TheTime;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Definition of a function that returns a timestamp (microseconds
// that have elapsed since 1.1.1970).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int64_t GetTimestampUs(void)
{
  struct timeval tv;
  int64_t TheTime;

  gettimeofday(&tv, (struct timezone *)NULL);
  TheTime = ((int64_t)tv.tv_sec * 1000000) + (int64_t)tv.tv_usec;
  return TheTime;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
