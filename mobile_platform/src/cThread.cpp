//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 22.3.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/MobilePlatform/cThread.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//********************************************************************************
// Static member functions.
//********************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method is to be called before every new loop of the thread
// function. It will be suspended in case of a thread sleep command,
// it will return 'true' in case of a thread run command and 'false'
// in case of a thread end command.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cThread::bThreadIsActive(cThread *pClass)
{

// Enter the critical section.
  pthread_mutex_lock(&pClass->ThreadMutex);

  while (1)
  {

// Run - normal operation.
    if (pClass->ThreadCommand == THREAD_COMMAND_RUN)
    {
      pClass->ThreadState = THREAD_STATE_RUNNING;
      pthread_mutex_unlock(&pClass->ThreadMutex);
      return true;
    }

// End - this is the exit point of the thread's main loop.
    else if (pClass->ThreadCommand == THREAD_COMMAND_END)
    {
      pClass->ThreadState = THREAD_STATE_ENDED;
      pthread_mutex_unlock(&pClass->ThreadMutex);
      return false;
    }

// Sleep - the thread is suspended until it receives a signal.
    pClass->ThreadState = THREAD_STATE_SLEEPING;
    while (pClass->ThreadCommand == THREAD_COMMAND_SLEEP)
      pthread_cond_wait(&pClass->ThreadCondVar, &pClass->ThreadMutex);
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// It is *compulsory* that the thread calls this method immediately
// before exiting - be it in case of a normal exit or after a (caught)
// exception.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cThread::ReportThreadEnd(cThread *pClass)
{
  pthread_mutex_lock(&pClass->ThreadMutex);
  pClass->ThreadState = THREAD_STATE_ENDED;
  pClass->bThreadCreated = false;
  pthread_mutex_unlock(&pClass->ThreadMutex);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//********************************************************************************
// General private methods.
//********************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Frees all resources that have been allocated.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cThread::Cleanup(void)
{

// Tell the thred to end.
  try
  {
    cmdEnd();
  }
  catch (cException e) {e.Print();}

// Destroy the thread mutex.
  if (bMutexInitialised)
  {
    pthread_mutex_destroy(&ThreadMutex);
    bMutexInitialised = false;
  }

// Destroy the thread condition variable.
  if (bCondVarInitialised)
  {
    pthread_cond_destroy(&ThreadCondVar);
    bCondVarInitialised = false;
  }

// Delete the thread argument data structure.
  if (pTA != 0)
  {
    delete pTA;
    pTA = 0;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialisation of the thread mutex and thread condition variable,
// creation and initialisation of a thread argument data structure.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cThread::Configuration(void)
{

// Initialise the thread mutex.
  int i = pthread_mutex_init(&ThreadMutex, 0);
  if (i != 0)
  {
    if (i != EBUSY)
    {
      Cleanup();
      throw cSysCallException(ID, "cThread::Configuration()", "Error.SysCall.pthread_mutex_init()", i);
    }
    else fprintf(stderr, "*** WARNING: Re-initialising mutex!\n");
  }
  bMutexInitialised = true;

// Initialise the thread condition variable.
  i = pthread_cond_init(&ThreadCondVar, 0);
  if (i != 0)
  {
    if (i != EBUSY)
    {
      Cleanup();
      throw cSysCallException(ID, "cThread::Configuration()", "Error.SysCall.pthread_cond_init()", i);
    }
    else fprintf(stderr, "*** WARNING: Re-initialising condition variable!\n");
  }
  bCondVarInitialised = true;

// Create and initialise a thread argument data structure.
  pTA = new (std::nothrow) struct ThreadArgument;
  if (pTA == 0)
  {
    i = errno;
    Cleanup();
    throw cSysCallException(ID, "cThread::Configuration()", "Error.SysCall.new", i);
  }
  pTA->pClass = this;
  pTA->bThreadIsActive = bThreadIsActive;
  pTA->ReportThreadEnd = ReportThreadEnd;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//********************************************************************************
// General public methods.
//********************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The constructor of the class 'cThread'. The ID of this instance of
// the class 'cThread' as well as the timeout (in milliseconds) when
// waiting for the thread's response to commands are locally stored,
// member attributes are initialised and the instance is configured.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cThread::cThread(uint32_t InstanceID, uint32_t WaitForResponseTimeout)
{

// Locally store the ID of this instance of the class 'cThread' and the wait-for-end timeout.
  ID = InstanceID;
  ResponseTimeout = WaitForResponseTimeout;

// Initially there are no resources allocated.
  pTA = 0;
  bThreadCreated = false;
  bMutexInitialised = false;
  bCondVarInitialised = false;
  ThreadState = THREAD_STATE_ENDED;

// Configure this instance of the class 'cThread'.
  Configuration();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The destructor of the class 'cThread'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cThread::~cThread()
{
  Cleanup();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the ID of this instance of the class 'cThread'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t cThread::GetID(void)
{
  return ID;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//********************************************************************************
// API-related public methods.
//********************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Returns the state of the thread: 'THREAD_STATE_ENDED',
// 'THREAD_STATE_SLEEPING' or 'THREAD_STATE_RUNNING'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int cThread::GetThreadState(void)
{
  int Aux;

// Enter the critical section.
  pthread_mutex_lock(&ThreadMutex);

// Get the thread state and leave the critical section.
  Aux = ThreadState;
  pthread_mutex_unlock(&ThreadMutex);
  return Aux;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Creates a thread using the function 'ThreadFunc' and providing
// 'Arg' to the thread as argument.
// If thread creation fails, there will be *NO* cleanup of the
// previously allocated resources - would prevent a retry.
// In case of success 'true' is returned, if there already exists a
// thread 'false' is returned and in case of failing to create a
// thread, an exception will be thrown.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cThread::Create(void *(*ThreadFunc)(void *), void *Arg)
{
  int i;

// Check if there is already a thread.
  if (bThreadCreated) return false;

// Initialize the thread argument structure.
  pTA->Arg = Arg;

  ThreadCommand = THREAD_COMMAND_SLEEP;
  ThreadState = THREAD_STATE_SLEEPING;
  i = pthread_create(&ThreadID, 0, ThreadFunc, (void *)pTA);
  if (i != 0)
  {
    ThreadState = THREAD_STATE_ENDED;
    bThreadCreated = false;
    throw cSysCallException(ID, "cThread::Create()", "Error.SysCall.pthread_create()", i);
  }
  else bThreadCreated = true;
  return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the thread to the running state. In case of success (that is,
// a thread had been created as is sleeping) 'true' is returned,
// 'false' otherwise.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cThread::cmdRun(void)
{

// Enter the critical section.
  pthread_mutex_lock(&ThreadMutex);

// Check if there is a thread and if it's sleeping.
  if (bThreadCreated && (ThreadState == THREAD_STATE_SLEEPING))
  {
    ThreadCommand = THREAD_COMMAND_RUN;
    pthread_cond_signal(&ThreadCondVar);
    pthread_mutex_unlock(&ThreadMutex);
    return true;
  }
  else
  {
    pthread_mutex_unlock(&ThreadMutex);
    return false;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the thread to sleeping state and waits for the thread to be
// asleep. In case of success (that is, a thread existed, was running
// and put to sleep) 'true' is returned, otherwise 'false' is returned.
// It is possible that the thread's main function gets stuck, e.g. when
// waiting for I/O without timeout. This means that the caller of this
// method also gets stuck - in order to prevent this, after a time
// <ResponseTimeout> without getting the desired response, an exception
// is thrown.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cThread::cmdSleep(void)
{
  uint32_t TimeWaited;
  int i;

// Enter the critical section.
  pthread_mutex_lock(&ThreadMutex);

// Check if there is a thread and if it's running.
  if (bThreadCreated && (ThreadState == THREAD_STATE_RUNNING))
  {
    ThreadCommand = THREAD_COMMAND_SLEEP;
    pthread_cond_signal(&ThreadCondVar);
    pthread_mutex_unlock(&ThreadMutex);

// Wait for the thread to sleep (or end).
    TimeWaited = 0;
    while (1)
    {
      i = GetThreadState();
      if (i == THREAD_STATE_SLEEPING) return true;
      else if (i == THREAD_STATE_ENDED) return false;
      else
      {
        if (TimeWaited < ResponseTimeout)
        {
          usleep(5000);
          TimeWaited += 5;
        }
        else throw cException(ID, "cThread::cmdSleep()", "Error.Timeout.Response");
      }
    }
  }
  else
  {
    pthread_mutex_unlock(&ThreadMutex);
    return false;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Ends the thread. In case of success (that is, the thread existed,
// was either running or asleep, and ended) 'true' is returned,
// otherwise 'false' is returned.
// TODO: 'pthread_join()' waits until the thread has ended, however,
// this might never happen if the thread's main function is stuck,
// e.g., waiting for I/O without timeoout.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cThread::cmdEnd(void)
{
  uint32_t TimeWaited;
  int i;

// Enter the critical section.
  pthread_mutex_lock(&ThreadMutex);

// Check if there is a thread and if it's running or sleeping.
  if (bThreadCreated && ((ThreadState == THREAD_STATE_RUNNING) || (ThreadState == THREAD_STATE_SLEEPING)))
  {
    ThreadCommand = THREAD_COMMAND_END;
    pthread_cond_signal(&ThreadCondVar);
    pthread_mutex_unlock(&ThreadMutex);

// Wait for the thread to end).
    TimeWaited = 0;
    while (1)
    {
      i = GetThreadState();
      if (i == THREAD_STATE_SLEEPING) return true;
      else if (i == THREAD_STATE_ENDED) return false;
      else
      {
        if (TimeWaited < ResponseTimeout)
        {
          usleep(5000);
          TimeWaited += 5;
        }
        else throw cException(ID, "cThread::cmdEnd()", "Error.Timeout.Response");
      }
    }

    pthread_join(ThreadID, 0);
    return true;
  }
  else
  {
    pthread_mutex_unlock(&ThreadMutex);
    return false;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
