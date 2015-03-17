#include "classifier.h"


struct classifierData minimums={0};
struct classifierData lastState={0};
struct classifierData maximums={0};





int initializeClassifier()
{
  minimums.holesPercentTop=15;
  maximums.holesPercentTop=75;

  minimums.holesPercentBase=5;
  maximums.holesPercentBase=60;

  minimums.objectTemperature = 31.5;
  maximums.objectTemperature = 37.0;

  minimums.scoreTop = 1350;
  maximums.scoreTop = 2000;

  minimums.scoreBase = 550;
  maximums.scoreBase = 1000;
  lastState.useScoreBase=1;



   lastState.baseWidth=440;
   lastState.baseHeight=160;
   lastState.baseX1=200;
   lastState.baseY1=200;

   lastState.topWidth=300;
   lastState.topHeight=200;

  return 0;
}
