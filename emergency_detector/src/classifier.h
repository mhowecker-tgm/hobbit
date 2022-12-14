#ifndef CLASSIFIER_H_INCLUDED
#define CLASSIFIER_H_INCLUDED


#ifdef __cplusplus
extern "C"
{
#endif

enum MAP_CHECK_ENUM
{
  FRONT_0_9M=0,
  FRONT_1_0M,
  FRONT_1_1M,
  //=============
  NUMBER_OF_MAP_CHECKS
};


struct classifierData
{
  unsigned int headLookingDirection;
  //---------------------------------------
  unsigned int badContrastTop;

  //---------------------------------------
  float temperatureX,temperatureY,temperatureZ;
  float ambientTemperature;
  float objectTemperature;
  unsigned int timestampTemperature;
  unsigned int useObjectTemperature;
  //----------------------------------------
  float holesPercentOverTop;
  unsigned int useHolesOverTop;
  float holesPercentTop;
  unsigned int useHolesTop;
  float holesPercentOverBase;
  unsigned int useHolesOverBase;
  float holesPercentBase;
  unsigned int useHolesBase;
  //----------------------------------------
  unsigned int scoreOverTop;
  unsigned int scoreTop;
  unsigned int scoreOverBase;
  unsigned int scoreBase;
  unsigned int useScoreBase;
  //----------------------------------------
  unsigned int mapShouldBeClear[NUMBER_OF_MAP_CHECKS];
  unsigned int totalMapObstacleHits;
  unsigned int useMapInfo;

  //----------------------------------------
  //----------------------------------------
  //----------------------------------------

  unsigned int overTopX1,overTopY1,overTopWidth,overTopHeight;
  unsigned int topX1,topY1,topWidth,topHeight;
  //-------
  unsigned int overBaseX1,overBaseY1,overBaseWidth,overBaseHeight;
  unsigned int baseX1,baseY1,baseWidth,baseHeight;
  //-------

};

extern struct classifierData minimums;
extern struct classifierData lastState;
extern struct classifierData maximums;

extern unsigned int emergencyDetected;
extern unsigned int headIsMoving;


//-----------------------------------------------------------------------
 // Synchronization variables
 extern int maximumFrameDifferenceForTemperatureToBeRelevant;
 extern unsigned int doCalculationsCooldown;
 extern int doCalculations;
 extern unsigned int receivedBaseImages;
 // --------------------------------------------------------------------




int appendClassifierData(const char * filename,unsigned int frameTimestamp);



int initializeClassifier();






#ifdef __cplusplus
}
#endif


#endif
