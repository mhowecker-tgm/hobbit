//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 22.3.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/MobilePlatform/cMySystem.h"
#include "../include/MobilePlatform/cThread.h"
#include "../include/MobilePlatform/cMutex.h"
#include "../include/MobilePlatform/cRs232Port.h"
#include "../include/MobilePlatform/cTcpSocket.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// Private factory methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Factory for implementations (derived classes) of various inter-
// faces (base clases).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMySystem::Factory(void)
{

// Get the interface and implementation type of the current node.
  const char *Interface = pXml->GetAttributeValue(pXml->HasAttributeWithName("interface"));
  const char *Implementation = pXml->GetAttributeValue(pXml->HasAttributeWithName("implementation"));
  uint32_t InstanceID = strtoul(pXml->GetAttributeValue(pXml->HasAttributeWithName("id")), 0, 10);
  printf("<%03u> Creating '%s : %s' [%u]\n", ObjectPoolIndex, Implementation, Interface, InstanceID);

// Check if the interface type is supported.
  void *Object = 0;

  if (StringCompare(Interface, "System") == 0)
  {
    if (StringCompare(Implementation, "Thread") == 0)
      Object = (void *)(new (std::nothrow) cThread(InstanceID));
    else if (StringCompare(Implementation, "Mutex") == 0)
      Object = (void *)(new (std::nothrow) cMutex(InstanceID));
    else throw cSystemException(ID, "cMySystem::Factory()", "Error.Unknown.Implementation", Implementation);
  }

  else if (StringCompare(Interface, "CommunicationPort") == 0)
  {
    if (StringCompare(Implementation, "Rs232Port") == 0)
      Object = (void *)(new (std::nothrow) cRs232Port(InstanceID, this));
    else if (StringCompare(Implementation, "TcpSocket") == 0)
      Object = (void *)(new (std::nothrow) cTcpSocket(InstanceID, this));
    else throw cSystemException(ID, "cMySystem::Factory()", "Error.Unknown.Implementation", Implementation);
  }
  else throw cSystemException(ID, "cMySystem::Factory()", "Error.Unknown.Interface", Interface);

  if (Object == 0)
    throw cSysCallException(ID, "cMySystem::Factory()", "Error.SysCall.new", errno);

// Add the object to the object pool.
  pObjectPool[ObjectPoolIndex].pObject = Object;
  pObjectPool[ObjectPoolIndex].pInterface = Interface;
  pObjectPool[ObjectPoolIndex].pImplementation = Implementation;
  pObjectPool[ObjectPoolIndex].ID = InstanceID;
  ObjectPoolIndex++;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cMySystem::Scrap(void)
{

// Remove all objects of the object pool.
  while (ObjectPoolIndex > 0)
  {
    ObjectPoolIndex--;
    printf("<%03u> Removing '%s : %s' [%u]\n", ObjectPoolIndex, pObjectPool[ObjectPoolIndex].pImplementation,
           pObjectPool[ObjectPoolIndex].pInterface, pObjectPool[ObjectPoolIndex].ID);
    if (StringCompare(pObjectPool[ObjectPoolIndex].pInterface, "CommunicationPort") == 0)
    {
      if (StringCompare(pObjectPool[ObjectPoolIndex].pImplementation, "Rs232Port") == 0)
        delete (cRs232Port *)(pObjectPool[ObjectPoolIndex].pObject);
      else if (StringCompare(pObjectPool[ObjectPoolIndex].pImplementation, "TcpSocket") == 0)
        delete (cTcpSocket *)(pObjectPool[ObjectPoolIndex].pObject);
    }
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// General public methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The constructor of the class 'cMySystem' initialises the class'
// attributes and allocates the required resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cMySystem::cMySystem(uint32_t InstanceID) : cSystem(InstanceID)
{
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The destructor of the class 'cMySystem' only removes the data
// specific to this derived class.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cMySystem::~cMySystem()
{
  Scrap();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

