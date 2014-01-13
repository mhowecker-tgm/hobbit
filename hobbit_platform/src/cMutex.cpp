//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 14.1.2011
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/hobbit_platform/cMutex.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//********************************************************************************
// General private methods.
//********************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Release all resources that were previously allocated.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMutex::Cleanup(void)
{

// Destroy the thread condition variable.
  if (bCondVarInitialised)
  {
    pthread_cond_destroy(&PThreadCondVar);
    bCondVarInitialised = false;
  }

// Destroy the thread mutex.
  if (bMutexInitialised)
  {
    pthread_mutex_destroy(&PThreadMutex);
    bMutexInitialised = false;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//********************************************************************************
// General public methods.
//********************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The constructor of the class 'cMutex'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cMutex::cMutex(uint32_t InstanceID)
{
  int i;

// Locally store the ID of this instance of the class 'cMutex'.
  ID = InstanceID;

// Initially there are no resources allocated.
  bMutexInitialised = false;

// Initialise the mutex.
  i = pthread_mutex_init(&PThreadMutex, 0);
  if (i != 0)
  {
    if (i != EBUSY)
      throw cSysCallException(ID, "cMutex::Mutex()", "Error.SysCall.pthread_mutex_init()", i);
    else printf("*** WARNING: Re-initialising mutex!\n");
  }
  bMutexInitialised = true;

// Initialise the thread condition variable.
  i = pthread_cond_init(&PThreadCondVar, 0);
  if (i != 0)
  {
    if (i != EBUSY)
    {
      Cleanup();
      throw cSysCallException(ID, "cMutex::cMutex()", "Error.SysCall.pthread_cond_init()", i);
    }
    else fprintf(stderr, "*** WARNING: Re-initialising condition variable!\n");
  }
  bCondVarInitialised = true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The destructor of the class 'cMutex'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cMutex::~cMutex()
{
  Cleanup();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the ID of this instance of the class 'cMutex'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cMutex::GetID(void)
{
  return ID;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//********************************************************************************
// API-related public methods.
//********************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Locks the mutex. In case the mutex was not already locked, the
// method returns with the mutex being locked. If the mutex was
// already locked, this method blocks until the mutex is unlocked by
// the thread that had locked it.
// WARNING: If a thread that has already locked the mutex tries to
// lock the mutex again, this will (most likely) result in a deadlock.
// A return code of '0' means success, every other value represents
// a LINUX error code.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int cMutex::Lock(void)
{
  return pthread_mutex_lock(&PThreadMutex);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Tries to lock the mutex. If it wasn't locked before, it is locked
// and '0' is returned. If the mutex is already locked, the method
// returns immediately with the code 'EBUSY'. Every other return
// value represents a LINUX error code.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int cMutex::TryLock(void)
{
  return pthread_mutex_trylock(&PThreadMutex);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Unlocks the mutex. In case the mutex is either not locked or had
// been locked by another thread, a LINUX error code is returned. A
// return value of '0' means success.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int cMutex::Unlock(void)
{
  return pthread_mutex_unlock(&PThreadMutex);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Signals the condition and unlocks the mutex. Make sure the mutex
// is locked befor invoking this method!
// A return code of '0' means success, every other value represents
// a LINUX error code.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int cMutex::SignalCondition(void)
{
  pthread_cond_signal(&PThreadCondVar);
  return pthread_mutex_unlock(&PThreadMutex);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int cMutex::WaitForSignal(long int MaxWaitTimeInMs)
{
  struct timespec TimeoutTime;
  long int TimeNanoseconds;
  __time_t TimeSeconds;
  int i;

// Calculate the absolute time for timeout.
  TimeNanoseconds = 1000000 * (MaxWaitTimeInMs % 1000);
  TimeSeconds = MaxWaitTimeInMs / 1000;
  clock_gettime(CLOCK_REALTIME, &TimeoutTime);
  TimeoutTime.tv_nsec += TimeNanoseconds;
  if (TimeoutTime.tv_nsec >= 1000000000)
  {
    TimeoutTime.tv_nsec -= 1000000000;
    TimeSeconds++;
  }
  TimeoutTime.tv_sec += TimeSeconds;
  i =  pthread_cond_timedwait(&PThreadCondVar, &PThreadMutex, &TimeoutTime);
  return i;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
