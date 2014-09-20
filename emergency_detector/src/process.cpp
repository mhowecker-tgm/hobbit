#include "process.h"
#include "fall_detection.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "RGBDAcquisition/acquisitionSegment/AcquisitionSegment.h"
#include "RGBDAcquisition/processors/ViewpointChange/ViewpointChange.h"


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

using namespace std;
using namespace cv;

struct SegmentationFeaturesRGB segConfRGB={0};
struct SegmentationFeaturesDepth segConfDepth={0};
unsigned int combinationMode=COMBINE_AND;


unsigned int maximumFrameDifferenceForTemperatureToBeRelevant=10;

unsigned int minimumAllowedHolePercentage = 15;
unsigned int maximumAllowedHolePercentage = 75;

float minHumanTemperature = 31.5;
float maxHumanTemperature = 37.0;

unsigned int tempZoneWidth = 300;
unsigned int tempZoneHeight = 200;

unsigned int minScoreTrigger = 1600;
unsigned int maxScoreTrigger = 2000;

unsigned int doCVOutput=0;
unsigned int emergencyDetected=0;
unsigned int personDetected=0;

float temperatureAmbientDetected=0.0; //<- YODO : default value should be 0
float temperatureObjectDetected=0.0; //<- YODO : default value should be 0
float temperatureX=0.0,temperatureY=0.0,temperatureZ=0.0;
unsigned int tempTimestamp=0;

float bboxCX,bboxCY,bboxCZ,bboxWidth,bboxHeight,bboxDepth;
unsigned int bboxTimeStamp=0;




int processBoundingBox(
                        float ctX,float ctY,float ctZ,
                        float sizeX,float sizeY,float sizeZ,
                        unsigned int matchingTimestamp)
{
   bboxCX=ctX,bboxCY=ctY,bboxCZ=ctZ,bboxWidth=sizeX,bboxHeight=sizeY,bboxDepth=sizeZ;
   bboxTimeStamp=matchingTimestamp;

   if (userHasFallen(&fallDetectionContext,matchingTimestamp))
        {
          fprintf(stderr,MAGENTA "\n\n  Live Falling User Detected , EMERGENCY \n\n" NORMAL);
          emergencyDetected=1;
        }

}

unsigned char * copyRGB(unsigned char * source , unsigned int width , unsigned int height)
{
  if ( (source==0)  || (width==0) || (height==0) )
    {
      fprintf(stderr,"copyRGB called with zero arguments\n");
      return 0;
    }

  unsigned char * output = (unsigned char*) malloc(width*height*3*sizeof(unsigned char));
  if (output==0) { fprintf(stderr,"copyRGB could not allocate memory for output\n"); return 0; }
  memcpy(output , source , width*height*3*sizeof(unsigned char));
  return output;
}

unsigned short * copyDepth(unsigned short * source , unsigned int width , unsigned int height)
{
  if ( (source==0)  || (width==0) || (height==0) )
    {
      fprintf(stderr,"copyDepth called with zero arguments\n");
      return 0;
    }

  unsigned short * output = (unsigned short*) malloc(width*height*sizeof(unsigned short));
  if (output==0) { fprintf(stderr,"copyDepth could not allocate memory for output\n"); return 0; }
  memcpy(output , source , width*height*sizeof(unsigned short));
  return output;
}


unsigned int temperatureSensorSensesHuman(unsigned int tempDetected , unsigned int tempTimestamp , unsigned int frameTimestamp)
{
 unsigned int temperatureFrameOffset = ABSDIFF(frameTimestamp,tempTimestamp);
  if (
       (minHumanTemperature<tempDetected) &&
       (tempDetected<maxHumanTemperature) &&
       (temperatureFrameOffset < maximumFrameDifferenceForTemperatureToBeRelevant )
     )
    {
        return 1;
    }
   return 0;
}




int runServicesThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                       unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                        void * calib ,
                                          unsigned int frameTimestamp )
{
  unsigned char * segmentedRGB = 0;
  unsigned short * segmentedDepth = 0;
  unsigned int tempZoneStartX = (unsigned int ) ((colorWidth-tempZoneWidth) / 2);
  unsigned int tempZoneStartY = (unsigned int ) ((colorHeight-tempZoneHeight) / 2);
  unsigned int depthAvg = 0;

  //unsigned int temperatureFrameOffset = ABSDIFF(frameTimestamp,tempTimestamp);



  //If we are looking Center then we might detect a person..!
  if (fallDetectionContext.headLookingDirection==HEAD_LOOKING_CENTER)
  {
     //Contemplate about emitting a Person message ( not an emergency )
     if ( temperatureSensorSensesHuman( temperatureObjectDetected ,  tempTimestamp , frameTimestamp) )
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
                                               &temperatureX ,
                                               &temperatureY ,
                                               &temperatureZ
                                              )
           )
            {
               if (temperatureZ==0)    { fprintf(stderr,YELLOW "Will not emit person message because we don't have a depth\n" NORMAL); } else
               if (temperatureZ<=2500) { personDetected=1; } else
                                       { fprintf(stderr,YELLOW "Will not emit person message because skeleton is too far (%0.2f mm) to trust thermometer\n" NORMAL,temperatureZ); }
            }
     }


   if ( temperatureSensorSensesHuman( temperatureObjectDetected ,  tempTimestamp , frameTimestamp) )
   {
    if (userHasFallen(&fallDetectionContext,frameTimestamp))
        {
          fprintf(stderr,MAGENTA "\n\n  Live Falling User Detected , EMERGENCY \n\n" NORMAL);
          emergencyDetected=1;
        }

   }
  }


  if (fallDetectionContext.headLookingDirection!=HEAD_LOOKING_DOWN)
    { fprintf(stderr,RED "\n\n  Not Looking Down , Thermometer will never pick up the floor\n\n" NORMAL ); }
     else
  if ( temperatureSensorSensesHuman( temperatureObjectDetected ,  tempTimestamp , frameTimestamp) )
  //if ( (minHumanTemperature<temperatureObjectDetected) && (temperatureObjectDetected<maxHumanTemperature) && (temperatureFrameOffset < maximumFrameDifferenceForTemperatureToBeRelevant )  )
    {
        //fprintf(stderr,"runServicesThatNeedColorAndDepth called \n");
         segmentedRGB = copyRGB(colorFrame ,colorWidth , colorHeight);
         segmentedDepth = copyDepth(depthFrame ,depthWidth , depthHeight);
        //fprintf(stderr,"Copied rgb/depth\n");

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

      unsigned int holesEncountered = 0;
      depthAvg = viewPointChange_countDepths( segmentedDepth , colorWidth , colorHeight , tempZoneStartX , tempZoneStartY , tempZoneWidth , tempZoneHeight , maxScoreTrigger , 1 , &holesEncountered );
      fprintf(stderr,"Avg Depth is %u mm , empty area is %0.2f %% \n",depthAvg , (float) (100*holesEncountered)/(tempZoneWidth*tempZoneHeight));


      if (holesEncountered< ( (unsigned int) tempZoneWidth*tempZoneHeight*minimumAllowedHolePercentage/100 ) )
         { fprintf(stderr,RED "\n\n  Too few holes , too big a blob , cannot be an emergency\n\n" NORMAL ); }
          else
      if (holesEncountered> ( (unsigned int) tempZoneWidth*tempZoneHeight*maximumAllowedHolePercentage/100 ) )
         { fprintf(stderr,RED "\n\n  Too many holes , this cannot be an emergency \n\n" NORMAL ); }
          else
      if (
           ( depthAvg > minScoreTrigger) &&
           ( depthAvg < maxScoreTrigger)
         )
         {
           if (userIsStanding(&fallDetectionContext,frameTimestamp))
            { fprintf(stderr,RED "\n\n  We have a standing user , so he is taking care of fallen user , will not emit emergency \n\n" NORMAL ); }
             else
            {
              //We are almost sure we have a fallen blob , but maybe the blob continues on the top side ( so it is a standing user after all
              unsigned int overHeight=120;
              unsigned int depthAvgOver = viewPointChange_countDepths( segmentedDepth , colorWidth , colorHeight , tempZoneStartX , tempZoneStartY-overHeight , tempZoneWidth , overHeight , maxScoreTrigger , 1 , &holesEncountered );
              fprintf(stderr,MAGENTA "\n\n  Top Avg is %u mm Holes are %0.2f %% \n\n" NORMAL , depthAvgOver, (float) (100*holesEncountered)/(tempZoneWidth*overHeight));

              if (holesEncountered> ( (unsigned int) tempZoneWidth*overHeight*80/100 ) )
               {
                 fprintf(stderr,MAGENTA "\n\n  Already Fallen User Detected , EMERGENCY \n\n" NORMAL);
                 emergencyDetected=1;
               } else
               {
                 fprintf(stderr,RED "\n\n  Blob continues over temperature area , maybe standing person \n\n" NORMAL);
               }
            }
          }
    }

      if (doCVOutput)
      {
        cv::Mat bgrMat,rgbMat;
        if (segmentedRGB!=0)  { rgbMat = cv::Mat(colorHeight,colorWidth,CV_8UC3,segmentedRGB,3*colorWidth); } else
                              { rgbMat = cv::Mat(colorHeight,colorWidth,CV_8UC3,colorFrame,3*colorWidth); }

	    cv::cvtColor(rgbMat,bgrMat, CV_RGB2BGR);// opencv expects the image in BGR format

        RNG rng(12345);
        Point centerPt; centerPt.x=colorWidth/2;       centerPt.y=colorHeight/2;
        Point pt1;      pt1.x=tempZoneStartX;               pt1.y=tempZoneStartY;
        Point pt2;      pt2.x=tempZoneStartX+tempZoneWidth; pt2.y=tempZoneStartY+tempZoneHeight;
        Scalar color = Scalar ( rng.uniform(0,255) , rng.uniform(0,255) , rng.uniform(0,255)  );
        Scalar colorEmergency = Scalar ( 0 , 0 , 255  );
        rectangle(bgrMat ,  pt1 , pt2 , color , 2, 8 , 0);


        unsigned int tempColorR=255 , tempColorG=0 , tempColorB=0;
        if (temperatureObjectDetected<30) { tempColorR=0 , tempColorG=0 , tempColorB=255; } else
        if (temperatureObjectDetected>40) { tempColorR=255 , tempColorG=0 , tempColorB=0; } else
                                          { tempColorR=(unsigned int) 125+( 40-temperatureObjectDetected/10 ) * 125 , tempColorG=0 , tempColorB=0; }


        Scalar tempColor = Scalar ( tempColorB , tempColorG , tempColorR );
        circle(bgrMat,  centerPt , 20 , tempColor , 4, 8 , 0);

        char rectVal[256]={0};
        int fontUsed=FONT_HERSHEY_SIMPLEX; //FONT_HERSHEY_SCRIPT_SIMPLEX;
        Point txtPosition;  txtPosition.x = pt1.x+15; txtPosition.y = pt1.y+20;


        if ( emergencyDetected )
        {
          putText(bgrMat , "Emergency Detected ..! " , txtPosition , fontUsed , 0.7 , color , 2 , 8 );
          Point ptIn1; ptIn1.x=tempZoneStartX;               ptIn1.y=tempZoneStartY;
          Point ptIn2; ptIn2.x=tempZoneStartX+tempZoneWidth; ptIn2.y=tempZoneStartY+tempZoneHeight;

          unsigned int i=0;
          for (i=0; i<3; i++)
          {
            ptIn1.x-=10; ptIn1.y-=10;
            ptIn2.x+=10; ptIn2.y+=10;
            rectangle(bgrMat ,  ptIn1 , ptIn2 , colorEmergency , 2, 8 , 0);
          }

          Point ul; ul.x=0;          ul.y=0;
          Point ur; ur.x=colorWidth; ur.y=0;
          Point dl; dl.x=0;          dl.y=colorHeight;
          Point dr; dr.x=colorWidth; dr.y=colorHeight;
          line(bgrMat,ul,ptIn1 , colorEmergency , 2 , 8 , 0);
          line(bgrMat,dr,ptIn2 , colorEmergency , 2 , 8 , 0);

          ptIn1.x+=tempZoneWidth+60;
          ptIn2.x-=tempZoneWidth+60;
          line(bgrMat,ur,ptIn1 , colorEmergency , 2 , 8 , 0);
          line(bgrMat,dl,ptIn2 , colorEmergency , 2 , 8 , 0);
        } else
        if (fallDetectionContext.headLookingDirection!=HEAD_LOOKING_DOWN)
           { putText(bgrMat , "Head Not Looking Down" , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
            else
        if (segmentedRGB==0)
           { putText(bgrMat , "Low Temp / Power Save" , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
            else
           { putText(bgrMat , "Scanning for emergency .." , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
        txtPosition.y += 24; snprintf(rectVal,123,"Score : %u",depthAvg);
        putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , color , 2 , 8 );
        txtPosition.y += 24; snprintf(rectVal,123,"Temperature : %0.2f C",temperatureObjectDetected);
        putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , color , 2 , 8 );

       if (userIsRecent(&fallDetectionContext,frameTimestamp))
       {
        txtPosition.y += 24; snprintf(rectVal,123,"Sk Movem. : %u",fallDetectionContext.skeletonVelocity);
        putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , color , 2 , 8 );

        unsigned int i=0;
        for (i=0; i<fallDetectionContext.numberOfJoints; i++)
         {
           Scalar jointColor = Scalar ( 0 , 255 , 0 );
           if ( ( fallDetectionContext.currentJoint2D[i].x!=0.0 ) || ( fallDetectionContext.currentJoint2D[i].y!=0.0) )
           {
               centerPt.x=fallDetectionContext.currentJoint2D[i].x;       centerPt.y=fallDetectionContext.currentJoint2D[i].y;
               circle(bgrMat,  centerPt , 15 , jointColor , -4, 8 , 0);
           }
         }
       }

	    cv::imshow("emergency_detector visualization",bgrMat);
	    cv::waitKey(1);
      }

   if (segmentedRGB!=0)      { free (segmentedRGB);   }
   if (segmentedDepth!=0)    { free (segmentedDepth); }

 return emergencyDetected;
}


void initializeProcess()
{
 initializeRGBSegmentationConfiguration(&segConfRGB,640,480);
 initializeDepthSegmentationConfiguration(&segConfDepth,640,480);


 segConfDepth.maxDepth=2800;
 //Hobbit orientation according to camera

 segConfDepth.doNotGenerateNormalFrom3Points=0;
 segConfDepth.p1[0]=492.23; segConfDepth.p1[1]=615.87; segConfDepth.p1[2]=1757.00;
 segConfDepth.p2[0]=51.46;  segConfDepth.p2[1]=622.97; segConfDepth.p2[2]=1722.00;
 segConfDepth.p3[0]=250.41; segConfDepth.p3[1]=403.77; segConfDepth.p3[2]=2198.00;

 segConfDepth.normal[0]=0.02; segConfDepth.normal[1]=-0.91; segConfDepth.normal[2]=-0.42;
 segConfDepth.center[0]=51.46; segConfDepth.normal[1]=622.97; segConfDepth.normal[2]=1722.0;

// segConfDepth.doNotGenerateNormalFrom3Points=1;

// segConfDepth.normal[0]=-0.02;
// segConfDepth.normal[1]=-0.78;
// segConfDepth.normal[2]=-0.62;

// segConfDepth.center[0]=114.95;
// segConfDepth.center[1]=501.61;
// segConfDepth.center[2]=1338.0;

 segConfDepth.planeNormalOffset=40.0;

 segConfDepth.enablePlaneSegmentation =1;

}
