//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 12.2.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <new>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <inttypes.h>
#include "cException.hh"
#include "cHobbitMCU.hh"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CallBackFunc(cHobbitMCU *pClass)
{

  printf("x=%f [m] | y=%f [m] | theta=%f [°] | v=%f [m/s] | w=%f [°/s]\n", pClass->PlatformOdometry.x,
         pClass->PlatformOdometry.y, ((180.0 / M_PI) * pClass->PlatformOdometry.theta),
         pClass->PlatformOdometry.v, ((180.0 / M_PI) * pClass->PlatformOdometry.w));

/*
  printf("Callback - [%u | %u] - %.1fpercent - [%i | %i | %i | %i] - [%u]\n",
         pClass->PlatformStatus.CurrentEncoder0, pClass->PlatformStatus.CurrentEncoder1,
         pClass->PlatformStatus.BatteryLevel, pClass->PlatformStatus.BumperSwitch0,
         pClass->PlatformStatus.BumperSwitch1, pClass->PlatformStatus.BumperSwitch2,
         pClass->PlatformStatus.BumperSwitch3, pClass->PlatformStatus.MotionState);
*/
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char* argv[])
{
  cHobbitMCU *pMCU;

// Check for the correct usage of the program.
  if (argc != 2)
  {
    printf("\nusage: %s name_of_the_xml_file\n\n", argv[0]);
    return 0;
  }

// Initially there are no resources allocated.
  pMCU = 0;

  try
  {

// Create and configure the system based on the XML file whose name is provided via command line.
    pMCU = new (std::nothrow) cHobbitMCU(1, argv[1]);
    if (pMCU == 0)
      throw cSysCallException(0, "main()", "Error.SysCall.new", errno);
    pMCU->SetCallBack(CallBackFunc);

    printf("Connecting to MCU\n");
    pMCU->Connect();
    getchar();


    pMCU->SetKeySwitch(true);
    getchar();

    pMCU->SetDataTransmission(true);
    getchar();

    pMCU->ResetOdometry();
    getchar();

    pMCU->DiscreteMotion(0, 1000);
    getchar();

    pMCU->ResetOdometry();
    getchar();

    pMCU->DiscreteMotion(2, 1000);
    getchar();

    pMCU->ResetOdometry();
    getchar();

    pMCU->DiscreteMotion(4, 1000);
    getchar();

    pMCU->ResetOdometry();
    getchar();

    pMCU->DiscreteMotion(6, 1000);
    getchar();

    pMCU->SetDataTransmission(false);
    getchar();

    pMCU->SetKeySwitch(false);
    getchar();

/*
    printf("Turning on key switch\n");
    pMCU->SetKeySwitch(true);
    getchar();

    printf("Enabling data transmission\n");
    pMCU->SetDataTransmission(true);
    getchar();

    printf("Disabling data transmission\n");
    pMCU->SetDataTransmission(false);
    getchar();

    printf("Turning off key switch\n");
    pMCU->SetKeySwitch(false);
    getchar();
*/
// Close the communication port;
    printf("Closing the port\n");
    pMCU->Disconnect();
  }
  catch (cSysCallException e) {e.Print();}
  catch (cXmlException e) {e.Print();}
  catch (cSystemException e) {e.Print();}
  catch (cException e) {e.Print();}

  printf("main(): Performing cleanup...\n");

// Conditionally remove the instance of the class 'cHobbitMCU'.
  if (pMCU != 0)
  {
    printf("main(): Removing instance of type 'cHobbitMCU'.\n");
    delete pMCU;
    pMCU = 0;
  }

  return 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

