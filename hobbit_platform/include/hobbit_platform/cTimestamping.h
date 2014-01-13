//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 5.8.2009
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef C_TIMESTAMPING_HH
#define C_TIMESTAMPING_HH
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <inttypes.h>
#include <sys/time.h>
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef NULL
#define NULL (void *)0
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Declaration of functions that return a timestamp. The timestamp
// is the number of milliseconds (GetTimestampMs) or the number of
// microseconds (GetTimestampUs) elapsed since 1.1.1970.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int64_t GetTimestampMs(void);
int64_t GetTimestampUs(void);
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
