#include "process.h"
#include "fall_detection.h"
#include "classifier.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "RGBDAcquisition/acquisitionSegment/AcquisitionSegment.h"
#include "RGBDAcquisition/processors/ViewpointChange/ViewpointChange.h"
#include "tools.h"
#include "visualization.h"

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/highgui/highgui.hpp"


#define NORMAL "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */


#define USE_HOBBIT_MAP_DATA 1


#if USE_HOBBIT_MAP_DATA
  #include "hobbit_msgs/GetOccupancyState.h"
    ros::ServiceClient mapSubscriber;
    unsigned int consultHobbitMap=0;
#else
    unsigned int consultHobbitMap=0;
#endif // USE_HOBBIT_MAP_DATA

using namespace std;
using namespace cv;

struct SegmentationFeaturesRGB segConfRGB={0};
struct SegmentationFeaturesDepth segConfDepth={0};
struct SegmentationFeaturesDepth segConfBaseDepth={0};
unsigned int combinationMode=COMBINE_AND;

char defaultDir[]="../../web_interface/bin/emergencies/";
char * imageDir = defaultDir;






unsigned int useTemperatureSensorForLiveFallDetection=0;
unsigned int doCVOutput=0;
unsigned int personDetected=0;
unsigned int autoPlaneSegmentationFlag=0;


float bboxCX,bboxCY,bboxCZ,bboxWidth,bboxHeight,bboxDepth;
unsigned int bboxTimeStamp=0;

int saveClassification=0;
int saveNextTopFrame=0;
int saveNextBottomFrame=0;
unsigned int framesSnapped=0;



int increasePlane()
{
    segConfDepth.planeNormalOffset+=30.0;
    fprintf(stderr,"Plane Increased to %0.2f ",segConfDepth.planeNormalOffset);
}



int decreasePlane()
{
    segConfDepth.planeNormalOffset-=30.0;
    fprintf(stderr,"Plane Decreased to %0.2f ",segConfDepth.planeNormalOffset);
}




int processBoundingBox(
                        float ctX,float ctY,float ctZ,
                        float sizeX,float sizeY,float sizeZ,
                        unsigned int matchingTimestamp)
{
   bboxCX=ctX,bboxCY=ctY,bboxCZ=ctZ,bboxWidth=sizeX,bboxHeight=sizeY,bboxDepth=sizeZ;
   bboxTimeStamp=matchingTimestamp;
/*
   if (userHasFallen(&fallDetectionContext,matchingTimestamp))
        {
          fprintf(stderr,MAGENTA "\n\n  Live Falling User Detected , EMERGENCY \n\n" NORMAL);
          emergencyDetected=1;
        }*/

}





#if USE_HOBBIT_MAP_DATA
bool obstacleDetected(hobbit_msgs::GetOccupancyState::Request  &req, hobbit_msgs::GetOccupancyState::Response &res)
 {
   /*
    Paloma :
    So, the current implementation is like this:
    /get_occupancy_state
    bool cLocalizationMonitor::getOccupancyState(hobbit_msgs::GetOccupancyState::Request  &req, hobbit_msgs::GetOccupancyState::Response &res)
    You can take a look at the GetOccupancyState service in hobbit_msgs, but it is basically what we talked about.
    If you want me to change the name or something or have any problems or whatever let me know, ok?
 */
  return res.is_occupied;
 }
#endif // USE_HOBBIT_MAP_DATA


unsigned int mapSaysThatWhatWeAreLookingAtPossibleFallenUser(unsigned int frameTimestamp)
{
   lastState.totalMapObstacleHits=1; // If USE_HOBBIT_MAP_DATA is not defined this function should always return 1;

   #if USE_HOBBIT_MAP_DATA
       lastState.totalMapObstacleHits=0;
       hobbit_msgs::GetOccupancyState::Request req;
       hobbit_msgs::GetOccupancyState::Response res;


       req.local_point.x = 1.0; req.local_point.y = 0; //1m mprosta sto robot
	   lastState.mapShouldBeClear[0] = obstacleDetected(req,res);
	   lastState.totalMapObstacleHits += lastState.mapShouldBeClear[0];

       req.local_point.x = 1.1; req.local_point.y = 0; //1,1m mprosta
	   lastState.mapShouldBeClear[1] = obstacleDetected(req,res);
	   lastState.totalMapObstacleHits += lastState.mapShouldBeClear[1];

       req.local_point.x = 0.9; req.local_point.y = 0; //0.9m mprosta
	   lastState.mapShouldBeClear[2] = obstacleDetected(req,res);
	   lastState.totalMapObstacleHits += lastState.mapShouldBeClear[2];

       fprintf(stderr,"\nGot %u obstacles from map! \n",lastState.totalMapObstacleHits);
   #endif // USE_HOBBIT_MAP_DATA

  return (lastState.totalMapObstacleHits>0);
}



int mapSaysThatWeMaybeLookingAtFallenUser(unsigned int frameTimestamp)
{
 if (consultHobbitMap)
   {
    return mapSaysThatWhatWeAreLookingAtPossibleFallenUser(frameTimestamp);
   }

 return 1; // If we are not using the map we always think that we might be looking at a fallen person
}




int weDetectAStandingPersonUsingTemperatureSensor(unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                               void * calib ,
                                               unsigned int frameTimestamp )
{
  //Contemplate about emitting a Person message ( not an emergency )
  if ( temperatureSensorSensesHuman( lastState.objectTemperature ,  lastState.timestampTemperature , frameTimestamp) )
     {
       unsigned int x2d = (unsigned int) depthWidth/2;
       unsigned int y2d = (unsigned int) depthHeight/2;
       unsigned short * depthValue = depthFrame + (y2d * depthWidth + x2d );

       struct calibration calibSpontaneous;
       NullCalibration(depthWidth,depthHeight,&calibSpontaneous);
       struct calibration * calibSelected = (struct calibration * ) calib;
       if (calibSelected==0) { calibSelected=&calibSpontaneous; }
       if (transform2DProjectedPointTo3DPoint( calibSelected,
                                               x2d,
                                               y2d,
                                                *depthValue ,
                                               &lastState.temperatureX ,
                                               &lastState.temperatureY ,
                                               &lastState.temperatureZ
                                              )
           )
            {
               if (lastState.temperatureZ==0)    { fprintf(stderr,YELLOW "Will not emit person message because we don't have a depth\n" NORMAL); } else
               if (lastState.temperatureZ<=2500) { return 1; } else
                                                 { fprintf(stderr,YELLOW "Will not emit person message because skeleton is too far (%0.2f mm) to trust thermometer\n" NORMAL,lastState.temperatureZ); }
            }
     }
   return 0;
}





int runServicesThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                       unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                        void * calib ,
                                          unsigned int frameTimestamp )
{
   if (saveNextTopFrame)
      {
        char filename[512];
        snprintf(filename,512,"snapshots/%s/colorFrame_0_%05u.pnm",imageDir,framesSnapped);
        saveRawImageToFile(filename,colorFrame,colorWidth,colorHeight,3,8);
        snprintf(filename,512,"snapshots/%s/depthFrame_0_%05u.pnm",imageDir,framesSnapped);
        saveRawImageToFile(filename,(unsigned char*) depthFrame,depthWidth,depthHeight,1,16);
        saveNextTopFrame=0;
        if ( (saveNextTopFrame==0) && (saveNextBottomFrame==0) )  { ++framesSnapped; }
      }


  unsigned char * segmentedRGB = 0;
  unsigned short * segmentedDepth = 0;
  lastState.topX1 = (unsigned int ) ((colorWidth-lastState.topWidth) / 2);
  lastState.topY1 = (unsigned int ) ((colorHeight-lastState.topHeight) / 2);

  lastState.badContrastTop = detectHighContrastUnusableRGB(colorFrame,colorWidth,colorHeight,40.0);

  //unsigned int temperatureFrameOffset = ABSDIFF(frameTimestamp,tempTimestamp);


  isSkeletonWrong(&fallDetectionContext,frameTimestamp);

  //If we are looking Center then we might detect a person..!
  if (fallDetectionContext.headLookingDirection==HEAD_LOOKING_CENTER)
  {
   if  (
          ( temperatureSensorSensesHuman( lastState.objectTemperature ,  lastState.timestampTemperature , frameTimestamp) )
           ||
          (!useTemperatureSensorForLiveFallDetection) //If we dont want to use temperature check just check using skeleton..!
        )
   {
    if (userHasFallen(&fallDetectionContext,frameTimestamp))
        {
          fprintf(stderr,MAGENTA "\n\n  Live Falling User Detected , EMERGENCY \n\n" NORMAL);
          emergencyDetected=1;
        }

   }


   if ( weDetectAStandingPersonUsingTemperatureSensor( depthFrame  , depthWidth , depthHeight , calib  , frameTimestamp) )
    {
      personDetected=1;
    }


  }

  if (doCalculations>0) { --doCalculations; }
  if (fallDetectionContext.headLookingDirection!=HEAD_LOOKING_DOWN)
  {
    fprintf(stderr,YELLOW "\n\n  Head Not Looking Down , Only doing active user falling check \n\n" NORMAL );
  }
     else
  if ( temperatureSensorSensesHuman( lastState.objectTemperature , lastState.timestampTemperature , frameTimestamp) )
    {
      if (mapSaysThatWeMaybeLookingAtFallenUser(frameTimestamp))
      {
         if (doCalculations<doCalculationsCooldown) { ++doCalculations; }//We do processing ..!
         segmentedRGB = copyRGB(colorFrame ,colorWidth , colorHeight);
         segmentedDepth = copyDepth(depthFrame ,depthWidth , depthHeight);

         if (autoPlaneSegmentationFlag) { segConfDepth.autoPlaneSegmentation=1; autoPlaneSegmentationFlag=0;
                                           fprintf(stderr,RED "Emergency Detector doing auto plane segmentation.." NORMAL);
                                        } else
                                        { segConfDepth.autoPlaneSegmentation=0; }


        fprintf(stderr,"Segmenting 2 frames sized  %ux%u and %ux%u \n",colorWidth , colorHeight,depthWidth , depthHeight);
        segmentRGBAndDepthFrame (
                                   segmentedRGB ,
                                   segmentedDepth ,
                                   colorWidth , colorHeight,
                                   &segConfRGB ,
                                   &segConfDepth ,
                                   (struct calibration*) calib ,
                                   combinationMode
                                );

      unsigned int holesTop=0;
      lastState.scoreTop = viewPointChange_countDepths( segmentedDepth , colorWidth , colorHeight , lastState.topX1 , lastState.topY1 , lastState.topWidth , lastState.topHeight , maximums.scoreTop , 1 , &holesTop );
      lastState.holesPercentTop = (float) (100*holesTop)/(lastState.topWidth*lastState.topHeight);

      fprintf(stderr,"Avg Depth is %u mm , empty area is %0.2f %% , Bot Depth %u mm , empty area is %0.2f %% \n",
              lastState.scoreTop , lastState.holesPercentTop ,
              lastState.scoreBase , lastState.holesPercentBase
             );



      if ( (lastState.useHolesBase) && (receivedBaseImages!=0) && (lastState.holesPercentBase < minimums.holesPercentBase) )
         { fprintf(stderr,RED "\n\n  Too few holes at Base ( min is %0.2f ) , too big a blob , cannot be an emergency\n\n" NORMAL , minimums.holesPercentBase); }
          else
      if ( (lastState.useHolesBase) && (receivedBaseImages!=0) && (lastState.holesPercentBase > maximums.holesPercentBase) )
         { fprintf(stderr,RED "\n\n  Too many holes at Base ( max is %0.2f ) , this cannot be an emergency \n\n" NORMAL  , maximums.holesPercentBase); }
          else
      if ( (lastState.useHolesTop) && (lastState.holesPercentTop < minimums.holesPercentTop) )
         { fprintf(stderr,RED "\n\n  Too few holes at Top ( min is %0.2f ) , too big a blob , cannot be an emergency\n\n" NORMAL , minimums.holesPercentTop); }
          else
      if ( (lastState.useHolesTop) && (lastState.holesPercentTop > maximums.holesPercentTop) )
         { fprintf(stderr,RED "\n\n  Too many holes at Top ( max is %0.2f ) , this cannot be an emergency \n\n" NORMAL , maximums.holesPercentTop ); }
          else
      if (
           (
            ( lastState.scoreBase > minimums.scoreBase) &&
            ( lastState.scoreBase < maximums.scoreBase)
           ) ||
           (lastState.useScoreBase==0) ||
           (receivedBaseImages==0) // If we have no zero images we can't use base camera to judge result
         )
      {
      if (
           ( lastState.scoreTop > minimums.scoreTop) &&
           ( lastState.scoreTop < maximums.scoreTop)
         )
         {
           if (userIsStanding(&fallDetectionContext,frameTimestamp))
            { fprintf(stderr,RED "\n\n  We have a standing user , so he is taking care of fallen user , will not emit emergency \n\n" NORMAL ); }
             else
            {
              //We are almost sure we have a fallen blob , but maybe the blob continues on the top side ( so it is a standing user after all
              unsigned int holesOverTemperatureArea=0;

              lastState.scoreOverTop = viewPointChange_countDepths( segmentedDepth , colorWidth , colorHeight , lastState.topX1 , lastState.topY1-overHeight , lastState.topWidth , overHeight , maximums.scoreTop , 1 , &holesOverTemperatureArea );
              lastState.holesPercentOverTop = (float) (100*holesOverTemperatureArea)/(lastState.topWidth*overHeight);

              fprintf(stderr,MAGENTA "\n\n  Top Avg is %u mm Holes are %0.2f %% \n\n" NORMAL , lastState.scoreOverTop, lastState.holesPercentTop  );

              if ( (lastState.useHolesOverTop)  && (lastState.holesPercentOverTop < minimums.holesPercentOverTop) )
               { fprintf(stderr,RED "\n\n  Blob continues over temperature area ( holes %0.2f %% ), maybe standing person \n\n" NORMAL, lastState.holesPercentOverTop); }
               else
              if ( (lastState.useHolesOverTop)  && (lastState.holesPercentOverTop > maximums.holesPercentOverTop) )
               { fprintf(stderr,RED "\n\n  Blob continues over temperature area ( holes %0.2f %% ), maybe standing person \n\n" NORMAL, lastState.holesPercentOverTop); }
               else
               {
                 fprintf(stderr,MAGENTA "\n\n  Already Fallen User Detected , EMERGENCY \n\n" NORMAL);
                 emergencyDetected=1;
               }
            }
          } else
          {
           fprintf(stderr,RED "\n\n  Top Camera indicates that this , cannot be an emergency ( score %u ,holes %0.2f %% )  score Min %u Max %u \n\n" NORMAL , lastState.scoreTop , lastState.holesPercentTop , minimums.scoreTop , maximums.scoreTop );
          }
      } else
      {
        fprintf(stderr,RED "\n\n  BaseCam indicates that this , cannot be an emergency ( score %u ,holes %0.2f %% ) score Min %u Max %u \n\n" NORMAL , lastState.scoreBase , lastState.holesPercentBase , minimums.scoreBase , maximums.scoreBase );
      }

      } //Map Check
    } //Temperature Check

      if (doCVOutput)
      {
        visualizeTopCam(colorFrame,segmentedRGB,colorWidth,colorHeight,consultHobbitMap,frameTimestamp);
      }



   if (segmentedRGB!=0)      { free (segmentedRGB);   }
   if (segmentedDepth!=0)    { free (segmentedDepth); }

 return emergencyDetected;
}




int runServicesBottomThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                           unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                           void * calib ,
                                           unsigned int frameTimestamp )
{
      if (saveNextBottomFrame)
      {
        char filename[512];
        snprintf(filename,512,"snapshots/%s/colorFrame_1_%05u.pnm",imageDir,framesSnapped);
        saveRawImageToFile(filename,colorFrame,colorWidth,colorHeight,3,8);
        snprintf(filename,512,"snapshots/%s/depthFrame_1_%05u.pnm",imageDir,framesSnapped);
        saveRawImageToFile(filename,(unsigned char*) depthFrame,depthWidth,depthHeight,1,16);
        saveNextBottomFrame=0;
        if ( (saveNextTopFrame==0) && (saveNextBottomFrame==0) )  { ++framesSnapped; }
      }


      if (doCalculations==0) { return 0; }


      lastState.baseX1 = (unsigned int ) ((colorWidth-lastState.baseWidth) / 2);
      lastState.baseY1 = 240;



     unsigned char * segmentedRGB = copyRGB(colorFrame ,colorWidth , colorHeight);
     unsigned short * segmentedDepth = copyDepth(depthFrame ,depthWidth , depthHeight);


     segmentRGBAndDepthFrame (
                                   segmentedRGB ,
                                   segmentedDepth ,
                                   colorWidth , colorHeight,
                                   &segConfRGB ,
                                   &segConfBaseDepth ,
                                   (struct calibration*) calib ,
                                   COMBINE_AND
                             );

      unsigned int holesBase=0;
      lastState.scoreBase = viewPointChange_countDepths( segmentedDepth , colorWidth , colorHeight ,
                                                         lastState.baseX1, lastState.baseY1 , lastState.baseWidth , lastState.baseHeight , maximums.scoreBase , 1 , &holesBase );
      lastState.holesPercentBase = (float) (100*holesBase)/(lastState.baseWidth*lastState.baseHeight);
      fprintf(stderr,"Bottom Avg Depth is %u mm , empty area is %0.2f %% \n",lastState.scoreBase , lastState.holesPercentBase );




      if (doCVOutput)
      {
        visualizeBaseCam(segmentedRGB,colorWidth,colorHeight);
      }

   if (segmentedRGB!=0)      { free (segmentedRGB);   }
   if (segmentedDepth!=0)    { free (segmentedDepth); }

      return 1;
}



void initializeProcess(ros::NodeHandle * nh)
{
 initializeClassifier();

 initializeRGBSegmentationConfiguration(&segConfRGB,640,480);
 initializeDepthSegmentationConfiguration(&segConfDepth,640,480);

#if USE_HOBBIT_MAP_DATA
  mapSubscriber = nh->serviceClient<hobbit_msgs::GetOccupancyState>("/get_occupancy_state", obstacleDetected);
  if (mapSubscriber)
  { fprintf(stderr,"Successfully created a persistant connection to mapping service\n"); } else
  { fprintf(stderr,"Could not create a persistant connection to mapping service\n");     }
#endif // USE_HOBBIT_MAP_DATA

 //Hobbit orientation according to camera
 segConfDepth.maxDepth=2000;
 segConfDepth.doNotGenerateNormalFrom3Points=0;
 segConfDepth.p1[0]=492.23; segConfDepth.p1[1]=615.87; segConfDepth.p1[2]=1757.00;
 segConfDepth.p2[0]=51.46;  segConfDepth.p2[1]=622.97; segConfDepth.p2[2]=1722.00;
 segConfDepth.p3[0]=250.41; segConfDepth.p3[1]=403.77; segConfDepth.p3[2]=2198.00;

 segConfDepth.normal[0]=0.02; segConfDepth.normal[1]=-0.91; segConfDepth.normal[2]=-0.42;
 segConfDepth.center[0]=51.46; segConfDepth.center[1]=622.97; segConfDepth.center[2]=1722.0;
 segConfDepth.planeNormalOffset=40.0;
 segConfDepth.enablePlaneSegmentation =1;

 initializeDepthSegmentationConfiguration(&segConfBaseDepth,640,480);

 segConfBaseDepth.maxDepth=2000;
 segConfBaseDepth.doNotGenerateNormalFrom3Points=0;
 segConfBaseDepth.p1[0]=440.53;   segConfBaseDepth.p1[1]=433.55;  segConfBaseDepth.p1[2]=1145.0;
 segConfBaseDepth.p2[0]=-252.23;  segConfBaseDepth.p2[1]=393.59;  segConfBaseDepth.p2[2]=925.00;
 segConfBaseDepth.p3[0]=-252.71;  segConfBaseDepth.p3[1]=418.15;  segConfBaseDepth.p3[2]=1219.00;

 segConfBaseDepth.normal[0]=0.03;    segConfBaseDepth.normal[1]=-1.00;  segConfBaseDepth.normal[2]=0.08;
 segConfBaseDepth.center[0]=-252.23; segConfBaseDepth.center[1]=393.59; segConfBaseDepth.center[2]=925.00;
 segConfBaseDepth.planeNormalOffset=60.0;
 segConfBaseDepth.enablePlaneSegmentation=1;
}
