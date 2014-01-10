//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 11.11.2010
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef C_MUTEX_HH
#define C_MUTEX_HH
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <new>
#include <string>
#include <inttypes.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include "cException.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cMutex
{
  protected:
//-------------------------------------------------------------------
// Mutex-related private attributes.
//-------------------------------------------------------------------
    bool bMutexInitialised;
    pthread_mutex_t PThreadMutex;
    bool bCondVarInitialised;
    pthread_cond_t PThreadCondVar;
//-------------------------------------------------------------------
// General private attributes.
//-------------------------------------------------------------------
    uint32_t ID;
//-------------------------------------------------------------------
// General private attributes.
//-------------------------------------------------------------------
    void Cleanup(void);

  public:
//-------------------------------------------------------------------
// General public methods.
//-------------------------------------------------------------------
    cMutex(uint32_t InstanceID);
    ~cMutex();
    uint32_t GetID(void);
//-------------------------------------------------------------------
// API-related public methods.
//-------------------------------------------------------------------
    int Lock(void);
    int TryLock(void);
    int Unlock(void);
    int SignalCondition(void);
    int WaitForSignal(long int MaxWaitTimeInMs = 2500);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
