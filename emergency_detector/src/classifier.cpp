#include "classifier.h"


struct classifierData minimums={0};
struct classifierData lastState={0};
struct classifierData maximums={0};

unsigned int emergencyDetected=0;
unsigned int overHeight=120;

//-----------------------------------------------------------------------
 //The following values are set by the launch file , so change them there..
 // Synchronization variables
 int maximumFrameDifferenceForTemperatureToBeRelevant=20;
 unsigned int doCalculationsCooldown=20;
 int doCalculations=0;
unsigned int receivedBaseImages=0;
 // --------------------------------------------------------------------


int initializeClassifier()
{
  minimums.holesPercentOverTop=60;
  maximums.holesPercentOverTop=100;
  lastState.useHolesOverTop=1;

  minimums.holesPercentTop=15;
  maximums.holesPercentTop=75;
  lastState.useHolesTop=0;

  minimums.holesPercentBase=5;
  maximums.holesPercentBase=55;
  lastState.useHolesBase=1;

  //These are initialized by the main.cpp
  //minimums.objectTemperature = 31.5;
  //maximums.objectTemperature = 37.0;


  minimums.scoreTop = 1300;
  maximums.scoreTop = 1600;

  minimums.scoreBase = 550;
  maximums.scoreBase = 1000;
  lastState.useScoreBase=1;

  lastState.badContrastTop=0;

  lastState.baseWidth=440;
  lastState.baseHeight=160;
  lastState.baseX1=200;
  lastState.baseY1=200;

  lastState.topWidth=300;
  lastState.topHeight=200;

  return 0;
}