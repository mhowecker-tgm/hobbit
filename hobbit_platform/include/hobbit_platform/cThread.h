//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 10.11.2010
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef C_THREAD_HH
#define C_THREAD_HH
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <new>
#include <string>
#include <inttypes.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include "cException.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Commands to control the behaviour of a thread.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define THREAD_COMMAND_SLEEP    0
#define THREAD_COMMAND_RUN      1
#define THREAD_COMMAND_END      2
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// States of a thread.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define THREAD_STATE_SLEEPING   0
#define THREAD_STATE_RUNNING    1
#define THREAD_STATE_ENDED      2
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// An instance of the structure 'ThreadArgument' is provided to the
// thread function upon creation of the thread. It comprises a pointer
// to its instance of the class 'cThread', a pointer to an optional
// argument as well as two function pointers. One function is used
// to inform the thread about its state (running, sleeping, ended) and
// the other function is called by the thread at its end to report its
// termination.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cThread;
struct ThreadArgument
{
  void *Arg;
  cThread *pClass;
  bool (*bThreadIsActive)(cThread *pClass);
  void (*ReportThreadEnd)(cThread *pClass);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The class 'cThread' provides the functionality to create and use a
// thread. The thread can be running or sleeping (waiting for a signal).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cThread
{
//-------------------------------------------------------------------
// Static member functions.
//-------------------------------------------------------------------
  static bool bThreadIsActive(cThread *pClass);
  static void ReportThreadEnd(cThread *pClass);

  protected:
//-------------------------------------------------------------------
// Private thread resources.
//-------------------------------------------------------------------
    bool bThreadCreated;
    bool bMutexInitialised;
    bool bCondVarInitialised;
    int ThreadCommand;
    int ThreadState;
    pthread_t ThreadID;
    pthread_mutex_t ThreadMutex;
    pthread_cond_t ThreadCondVar;
    struct ThreadArgument *pTA;
//-------------------------------------------------------------------
// General private attributes.
//-------------------------------------------------------------------
    uint32_t ID;
    uint32_t ResponseTimeout;
//-------------------------------------------------------------------
// General private methods.
//-------------------------------------------------------------------
    void Cleanup(void);
    void Configuration(void);

  public:
//-------------------------------------------------------------------
// General public methods.
//-------------------------------------------------------------------
    cThread(uint32_t InstanceID, uint32_t WaitForResponseTimeout = 4000);
    ~cThread();
    uint32_t GetID(void);
//-------------------------------------------------------------------
// API-related public methods.
//-------------------------------------------------------------------
    int GetThreadState(void);
    bool Create(void *(*ThreadFunc)(void *), void *Arg);
    bool cmdRun(void);
    bool cmdSleep(void);
    bool cmdEnd(void);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
