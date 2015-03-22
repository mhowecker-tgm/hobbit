#include "classifier.h"


struct classifierData minimums={0};
struct classifierData lastState={0};
struct classifierData maximums={0};

unsigned int headIsMoving=0;

unsigned int emergencyDetected=0;

//-----------------------------------------------------------------------
 //The following values are set by the launch file , so change them there..
 // Synchronization variables
 int maximumFrameDifferenceForTemperatureToBeRelevant=20;
 unsigned int doCalculationsCooldown=10;
 int doCalculations=0;
unsigned int receivedBaseImages=0;
 // --------------------------------------------------------------------



int appendClassifierData(const char * filename)
{
  FILE * fp = fopen(filename,"a");

  if (fp!=0)
  {

    struct classifierData * what2print = &lastState;

    unsigned int i=0;

    for (i=0; i<3; i++)
    {
      if (i==0)  { fprintf(fp,"MINIMUMS\n"); what2print = &minimums;  } else
      if (i==0)  { fprintf(fp,"TRIGGER\n");  what2print = &lastState; } else
      if (i==0)  { fprintf(fp,"MAXIMUMS\n"); what2print = &maximums;  }

      fprintf(fp,"holesOverTop %0.2f , holesTop %0.2f , holesBase %0.2f , holesOverBase %0.2f\n" ,
                  what2print->holesPercentOverTop ,
                  what2print->holesPercentTop ,
                  what2print->holesPercentBase ,
                  what2print->holesPercentOverBase   );



      fprintf(fp,"scoreOverTop %u , scoreTop %u , scoreBase %u , scoreOverBase %u\n" ,
                  what2print->scoreOverTop ,
                  what2print->scoreTop ,
                  what2print->scoreOverBase ,
                  what2print->scoreBase   );


      fprintf(fp,"temperature %0.2f\n" , what2print->objectTemperature);
      fprintf(fp,"______________________________________________________\n\n\n");
    }

    fclose(fp);
  }
}




int initializeClassifier()
{
  unsigned int imageWidth=640;
  unsigned int imageHeight=480;


  minimums.holesPercentOverTop=70;
  maximums.holesPercentOverTop=101;
  lastState.useHolesOverTop=1;

  minimums.holesPercentTop=15;
  maximums.holesPercentTop=75;
  lastState.useHolesTop=1;

  minimums.holesPercentBase=3;
  maximums.holesPercentBase=60;
  lastState.useHolesBase=1;

  //These are initialized by the main.cpp
  //minimums.objectTemperature = 31.5;
  //maximums.objectTemperature = 37.0;


  minimums.scoreTop = 1150;
  maximums.scoreTop = 1500;
  minimums.scoreOverTop = minimums.scoreTop;
  maximums.scoreOverTop = maximums.scoreTop;

  minimums.scoreBase = 550;
  maximums.scoreBase = 1100;
  minimums.scoreOverBase = minimums.scoreBase;
  maximums.scoreOverBase = maximums.scoreBase;
  lastState.useScoreBase=1;

  lastState.badContrastTop=0;



  lastState.baseWidth=440;
  lastState.baseHeight=160;
  lastState.baseX1 = (unsigned int ) ((imageWidth-lastState.baseWidth) / 2);
  lastState.baseY1 = 240;


  lastState.topWidth=300;
  lastState.topHeight=200;

  lastState.topX1 = (unsigned int ) ((imageWidth-lastState.topWidth) / 2);
  lastState.topY1 = (unsigned int ) ((imageHeight-lastState.topHeight) / 2);



  unsigned int overHeight=120;
  unsigned int overPlusWidth=40;

  lastState.overTopX1=lastState.topX1-overPlusWidth;
  lastState.overTopY1=lastState.topY1-overHeight;
  lastState.overTopWidth=lastState.topWidth+(2*overPlusWidth);
  lastState.overTopHeight=overHeight;

  //-------


  unsigned int overBaseHeight=120;
  unsigned int overBasePlusWidth=40;

  lastState.overBaseX1=lastState.baseX1-overBasePlusWidth;
  lastState.overBaseY1=lastState.baseY1-overBaseHeight;
  lastState.overBaseWidth=lastState.baseWidth+(2*overBasePlusWidth);
  lastState.overBaseHeight=overBaseHeight;

  return 0;
}
