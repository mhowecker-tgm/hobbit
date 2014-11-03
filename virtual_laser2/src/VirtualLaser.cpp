//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 20.2.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/VirtualLaser2/cVirtualLaser.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{
  VirtualLaser::cVirtualLaser vl(argc, argv);

  vl.Run();

  return 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

