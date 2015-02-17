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
struct SegmentationFeaturesDepth segConfBaseDepth={0};
unsigned int combinationMode=COMBINE_AND;



//-----------------------------------------------------------------------
 //The following values are set by the launch file , so change them there..
 // Synchronization variables
 int maximumFrameDifferenceForTemperatureToBeRelevant=20;
 unsigned int doCalculationsCooldown=20;
 int doCalculations=0;
 // --------------------------------------------------------------------


 unsigned int holesTop=0;
 float holesPercentTop=0;

 int minimumAllowedHolePercentage = 15;
 int maximumAllowedHolePercentage = 75;

double minHumanTemperature = 31.5;
double maxHumanTemperature = 37.0;

 int tempZoneWidth = 300;
 int tempZoneHeight = 200;

 int minScoreTrigger = 1350;
 int maxScoreTrigger = 2000;

 unsigned int botWidth=440,botHeight=160,botX1=200,botY1=200;
 unsigned int maxScoreBaseCamera = 3000;
 unsigned int depthBaseAvg=0;
 unsigned int holesBase=0;
 float holesPercentBase=0;
 unsigned int minBaseScoreTrigger = 550;
 unsigned int maxBaseScoreTrigger = 1000;
 float minimumAllowedHolePercentageBase =20;
 float maximumAllowedHolePercentageBase =60;

unsigned int useTemperatureSensorForLiveFallDetection=0;
unsigned int doUseTopHolesForClassification=0;
unsigned int doUseBaseHolesForClassification=1;
unsigned int doCVOutput=0;
unsigned int emergencyDetected=0;
unsigned int personDetected=0;
unsigned int autoPlaneSegmentationFlag=0;

float temperatureAmbientDetected=0.0; //<- YODO : default value should be 0
float temperatureObjectDetected=0.0; //<- YODO : default value should be 0
float temperatureX=0.0,temperatureY=0.0,temperatureZ=0.0;
unsigned int tempTimestamp=0;

float bboxCX,bboxCY,bboxCZ,bboxWidth,bboxHeight,bboxDepth;
unsigned int bboxTimeStamp=0;

int saveNextTopFrame=0;
int saveNextBottomFrame=0;
unsigned int framesSnapped=0;

unsigned int simplePow(unsigned int base,unsigned int exp)
{
if (exp==0) return 1;
unsigned int retres=base;
unsigned int i=0;
for (i=0; i<exp-1; i++)
{
retres*=base;
}
return retres;
}


int saveRawImageToFile(const char * filename,unsigned char * pixels , unsigned int width , unsigned int height , unsigned int channels , unsigned int bitsperpixel)
{
 //fprintf(stderr,"acquisitionSaveRawImageToFile(%s) called\n",filename);
 #if USE_REGULAR_BYTEORDER_FOR_PNM
 //Want Conformance to the NETPBM spec http://en.wikipedia.org/wiki/Netpbm_format#16-bit_extensions
 if (bitsperpixel==16) { swapEndiannessPNM(pixels , width , height , channels , bitsperpixel); }
 #else
  #warning "We are using Our Local Byte Order for saving files , this makes things fast but is incompatible with other PNM loaders"
 #endif // USE_REGULAR_BYTEORDER_FOR_PNM
 if ( (width==0) || (height==0) || (channels==0) || (bitsperpixel==0) ) { fprintf(stderr,"acquisitionSaveRawImageToFile(%s) called with zero dimensions\n",filename); return 0;}
 if(pixels==0) { fprintf(stderr,"acquisitionSaveRawImageToFile(%s) called for an unallocated (empty) frame , will not write any file output\n",filename); return 0; }
 if (bitsperpixel>16) { fprintf(stderr,"PNM does not support more than 2 bytes per pixel..!\n"); return 0; }
 FILE *fd=0;
 fd = fopen(filename,"wb");
 if (fd!=0)
  {
   unsigned int n;
   if (channels==3) fprintf(fd, "P6\n");
   else if (channels==1) fprintf(fd, "P5\n");
   else
  {
   fprintf(stderr,"Invalid channels arg (%u) for SaveRawImageToFile\n",channels);
   fclose(fd);
   return 1;
  }
   fprintf(fd, "#hobbit emergency view\n");
   fprintf(fd, "%d %d\n%u\n", width, height , simplePow(2 ,bitsperpixel)-1);
   float tmp_n = (float) bitsperpixel/ 8;
   tmp_n = tmp_n * width * height * channels ;
   n = (unsigned int) tmp_n;
   fwrite(pixels, 1 , n , fd);
   fflush(fd);
   fclose(fd);
   return 1;
  }
else
{
fprintf(stderr,"SaveRawImageToFile could not open output file %s\n",filename);
return 0;
}
return 0;
}



int setHobbitEMode()
{
    minHumanTemperature=27.5;
    segConfDepth.planeNormalOffset=430.0;
    segConfDepth.maxDepth=2500;
}


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


int detectHighContrastUnusableRGB(unsigned char * rgbFrame , unsigned int width , unsigned int height , float percentageHigh)
{
  unsigned char * rgbPtr = rgbFrame;
  unsigned char * rgbLimit = rgbFrame + width * height * 3;

  float tmp = percentageHigh / 100;
        tmp = tmp * width * height;
  unsigned int targetHighContrastPixels = (unsigned int) tmp;
  unsigned int highContrastPixels = 0;

  unsigned char r, g , b;
  while (rgbPtr<rgbLimit)
  {
    r = *rgbPtr++;
    g = *rgbPtr++;
    b = *rgbPtr++;
    if (
         ( (r<45) && (g<45)  && (b<45) ) ||
         ( (r>200) && (g>200)  && (b>200) )
       )
    {
     ++highContrastPixels;
     if ( highContrastPixels>targetHighContrastPixels)
         {
           //This spams console output and is included in OpenCV visualization
           //fprintf(stderr,"Bad View with %0.2f + high contrast points \n",(float) (100*highContrastPixels)/(width*height));
           return 1;
         }
    }
  }
  //fprintf(stderr,"Good View with %0.2f high contrast points \n",(float) (100*highContrastPixels)/(width*height));
 return 0;
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


unsigned int mapSaysThatWhatWeAreLookingAtShouldBeFreespace(float x,float y,float z , unsigned int frameTimestamp)
{
   fprintf(stderr,"mapSaysThatWhatWeAreLookingAtShouldBeFreespace() not implemented yet\n");
   /*

    Paloma :
So, the current implementation is like this:

/get_occupancy_state

bool cLocalizationMonitor::getOccupancyState(hobbit_msgs::GetOccupancyState::Request  &req, hobbit_msgs::GetOccupancyState::Response &res)

You can take a look at the GetOccupancyState service in hobbit_msgs, but it is basically what we talked about.

If you want me to change the name or something or have any problems or whatever let me know, ok?


   */

  return 1;
}



int mapSaysThatWeMaybeLookingAtFallenUser(unsigned int frameTimestamp)
{
 //  mapSaysThatWhatWeAreLookingAtShouldBeFreespace()
 return 1;
}


int runServicesThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                       unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                        void * calib ,
                                          unsigned int frameTimestamp )
{
   if (saveNextTopFrame)
      {
        char filename[512];
        snprintf(filename,512,"colorFrame_1_%05u.pnm",framesSnapped);
        saveRawImageToFile(filename,colorFrame,colorWidth,colorHeight,3,8);
        snprintf(filename,512,"depthFrame_1_%05u.pnm",framesSnapped);
        saveRawImageToFile("depthFrame_1_00000.pnm",(unsigned char*) depthFrame,depthWidth,depthHeight,1,16);
        saveNextTopFrame=0;
        if ( (saveNextTopFrame==0) && (saveNextBottomFrame==0) )  { ++framesSnapped; }
      }


  unsigned char * segmentedRGB = 0;
  unsigned short * segmentedDepth = 0;
  unsigned int tempZoneStartX = (unsigned int ) ((colorWidth-tempZoneWidth) / 2);
  unsigned int tempZoneStartY = (unsigned int ) ((colorHeight-tempZoneHeight) / 2);
  unsigned int depthAvg = 0;
  unsigned int badView = detectHighContrastUnusableRGB(colorFrame,colorWidth,colorHeight,40.0);

  //unsigned int temperatureFrameOffset = ABSDIFF(frameTimestamp,tempTimestamp);


  isSkeletonWrong(&fallDetectionContext);

  //If we are looking Center then we might detect a person..!
  if (fallDetectionContext.headLookingDirection==HEAD_LOOKING_CENTER)
  {
   if  (
          ( temperatureSensorSensesHuman( temperatureObjectDetected ,  tempTimestamp , frameTimestamp) )
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


  }

  if (doCalculations>0) { --doCalculations; }
  if (fallDetectionContext.headLookingDirection!=HEAD_LOOKING_DOWN)
    { fprintf(stderr,RED "\n\n  Not Looking Down , Thermometer will never pick up the floor\n\n" NORMAL ); }
     else
  if ( temperatureSensorSensesHuman( temperatureObjectDetected ,  tempTimestamp , frameTimestamp) )
  //if ( (minHumanTemperature<temperatureObjectDetected) && (temperatureObjectDetected<maxHumanTemperature) && (temperatureFrameOffset < maximumFrameDifferenceForTemperatureToBeRelevant )  )
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

      depthAvg = viewPointChange_countDepths( segmentedDepth , colorWidth , colorHeight , tempZoneStartX , tempZoneStartY , tempZoneWidth , tempZoneHeight , maxScoreTrigger , 1 , &holesTop );
      holesPercentTop = (float) (100*holesTop)/(tempZoneWidth*tempZoneHeight);

      fprintf(stderr,"Avg Depth is %u mm , empty area is %0.2f %% , Bot Depth %u mm , empty area is %0.2f %% \n",
              depthAvg , holesPercentTop ,
              depthBaseAvg , holesPercentBase
             );



      if ( (doUseBaseHolesForClassification) && (holesPercentBase< minimumAllowedHolePercentageBase ) )
         { fprintf(stderr,RED "\n\n  Too few holes at Base , too big a blob , cannot be an emergency\n\n" NORMAL ); }
          else
      if ( (doUseBaseHolesForClassification) && (holesPercentBase>  maximumAllowedHolePercentageBase ) )
         { fprintf(stderr,RED "\n\n  Too many holes at Base , this cannot be an emergency \n\n" NORMAL ); }
          else
      if ( (doUseTopHolesForClassification) && (holesPercentTop< minimumAllowedHolePercentage ) )
         { fprintf(stderr,RED "\n\n  Too few holes at Top , too big a blob , cannot be an emergency\n\n" NORMAL ); }
          else
      if ( (doUseTopHolesForClassification) && (holesPercentTop>  maximumAllowedHolePercentage ) )
         { fprintf(stderr,RED "\n\n  Too many holes at Top , this cannot be an emergency \n\n" NORMAL ); }
          else
      if (
            ( depthBaseAvg > minBaseScoreTrigger) &&
            ( depthBaseAvg < maxBaseScoreTrigger)
         )
      {
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
              unsigned int holesOverTemperatureArea=0;
              unsigned int overHeight=120;
              unsigned int depthAvgOver = viewPointChange_countDepths( segmentedDepth , colorWidth , colorHeight , tempZoneStartX , tempZoneStartY-overHeight , tempZoneWidth , overHeight , maxScoreTrigger , 1 , &holesOverTemperatureArea );

              fprintf(stderr,MAGENTA "\n\n  Top Avg is %u mm Holes are %0.2f %% \n\n" NORMAL , depthAvgOver, holesPercentTop  );

              if (holesOverTemperatureArea> ( (unsigned int) tempZoneWidth*overHeight*80/100 ) )
               {
                 fprintf(stderr,MAGENTA "\n\n  Already Fallen User Detected , EMERGENCY \n\n" NORMAL);
                 emergencyDetected=1;
               } else
               {
                 fprintf(stderr,RED "\n\n  Blob continues over temperature area , maybe standing person \n\n" NORMAL);
               }
            }
          } else
          {
           fprintf(stderr,RED "\n\n  Top Camera indicates that this , cannot be an emergency ( score %u ,holes %0.2f %% )  score Min %u Max %u \n\n" NORMAL , depthAvg , holesPercentTop , minScoreTrigger , maxScoreTrigger );
          }
      } else
      {
        fprintf(stderr,RED "\n\n  BaseCam indicates that this , cannot be an emergency ( score %u ,holes %0.2f %% ) score Min %u Max %u \n\n" NORMAL , depthBaseAvg , holesPercentBase , minBaseScoreTrigger , maxBaseScoreTrigger );
      }

      } //Map Check
    } //Temperature Check

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
        circle(bgrMat,  centerPt , 30 , tempColor , 4, 8 , 0);

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
        //======================================================================================================
        if (fallDetectionContext.headLookingDirection==HEAD_LOOKING_CENTER)
             { putText(bgrMat , "Horizontal Only Live Fall" , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
            else
        if (segmentedRGB==0)
             { putText(bgrMat , "Low Temp / Power Save" , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
            else
        if (fallDetectionContext.headLookingDirection!=HEAD_LOOKING_DOWN)
             { putText(bgrMat , "Head Not Looking Down" , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
            else
             { putText(bgrMat , "Scanning for fallen user.." , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
        //======================================================================================================
        txtPosition.y += 24; snprintf(rectVal,123,"Score Top : %u / Bot : %u ",depthAvg,depthBaseAvg);
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


       if (badView)
       {
        txtPosition.y += 24; snprintf(rectVal,123,"BAD VIEW!");
        putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , color , 2 , 8 );

       }

	    cv::imshow("emergency_detector visualization",bgrMat);
	    cv::waitKey(1);
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
        snprintf(filename,512,"colorFrame_1_%05u.pnm",framesSnapped);
        saveRawImageToFile(filename,colorFrame,colorWidth,colorHeight,3,8);
        snprintf(filename,512,"depthFrame_1_%05u.pnm",framesSnapped);
        saveRawImageToFile("depthFrame_1_00000.pnm",(unsigned char*) depthFrame,depthWidth,depthHeight,1,16);
        saveNextBottomFrame=0;
        if ( (saveNextTopFrame==0) && (saveNextBottomFrame==0) )  { ++framesSnapped; }
      }


      if (doCalculations==0) { return 0; }


      botX1 = (unsigned int ) ((colorWidth-botWidth) / 2);
      botY1 = 240;



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

      depthBaseAvg = viewPointChange_countDepths( segmentedDepth , colorWidth , colorHeight , botX1, botY1 , botWidth , botHeight , maxScoreBaseCamera , 1 , &holesBase );
      holesPercentBase = (float) (100*holesBase)/(botWidth*botHeight);
      fprintf(stderr,"Bottom Avg Depth is %u mm , empty area is %0.2f %% \n",depthBaseAvg , holesPercentBase );


      if (doCVOutput)
      {
        cv::Mat bgrMat,rgbMat;
        rgbMat = cv::Mat(colorHeight,colorWidth,CV_8UC3,segmentedRGB,3*colorWidth);

	    cv::cvtColor(rgbMat,bgrMat, CV_RGB2BGR);// opencv expects the image in BGR format

        Point ptIn1; ptIn1.x=botX1;               ptIn1.y=botY1;
        Point ptIn2; ptIn2.x=botX1+botWidth;      ptIn2.y=botY1+botHeight;
        Scalar colorEmergency = Scalar ( 255 , 0 , 0 );

        rectangle(bgrMat ,  ptIn1 , ptIn2 , colorEmergency , 2, 8 , 0);


        char rectVal[256]={0};
        int fontUsed=FONT_HERSHEY_SIMPLEX; //FONT_HERSHEY_SCRIPT_SIMPLEX;
        Point txtPosition;  txtPosition.x = ptIn1.x+15; txtPosition.y = ptIn1.y;


        txtPosition.y += 24; snprintf(rectVal,123,"Base Score : %u",depthBaseAvg);
        putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , colorEmergency , 2 , 8 );

        txtPosition.y += 24; snprintf(rectVal,123,"Base Holes %0.2f %%",holesPercentBase);
        putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , colorEmergency , 2 , 8 );


	    cv::imshow("emergency_detector base visualization",bgrMat);
	    cv::waitKey(1);
      }

   if (segmentedRGB!=0)      { free (segmentedRGB);   }
   if (segmentedDepth!=0)    { free (segmentedDepth); }

      return 1;
}



void initializeProcess()
{
 initializeRGBSegmentationConfiguration(&segConfRGB,640,480);
 initializeDepthSegmentationConfiguration(&segConfDepth,640,480);


 //Hobbit orientation according to camera
 segConfDepth.maxDepth=2800;
 segConfDepth.doNotGenerateNormalFrom3Points=0;
 segConfDepth.p1[0]=492.23; segConfDepth.p1[1]=615.87; segConfDepth.p1[2]=1757.00;
 segConfDepth.p2[0]=51.46;  segConfDepth.p2[1]=622.97; segConfDepth.p2[2]=1722.00;
 segConfDepth.p3[0]=250.41; segConfDepth.p3[1]=403.77; segConfDepth.p3[2]=2198.00;

 segConfDepth.normal[0]=0.02; segConfDepth.normal[1]=-0.91; segConfDepth.normal[2]=-0.42;
 segConfDepth.center[0]=51.46; segConfDepth.center[1]=622.97; segConfDepth.center[2]=1722.0;
 segConfDepth.planeNormalOffset=40.0;
 segConfDepth.enablePlaneSegmentation =1;

 initializeDepthSegmentationConfiguration(&segConfBaseDepth,640,480);

 segConfBaseDepth.maxDepth=2800;
 segConfBaseDepth.doNotGenerateNormalFrom3Points=0;
 segConfBaseDepth.p1[0]=440.53;   segConfBaseDepth.p1[1]=433.55;  segConfBaseDepth.p1[2]=1145.0;
 segConfBaseDepth.p2[0]=-252.23;  segConfBaseDepth.p2[1]=393.59;  segConfBaseDepth.p2[2]=925.00;
 segConfBaseDepth.p3[0]=-252.71;  segConfBaseDepth.p3[1]=418.15;  segConfBaseDepth.p3[2]=1219.00;

 segConfBaseDepth.normal[0]=0.03;    segConfBaseDepth.normal[1]=-1.00;  segConfBaseDepth.normal[2]=0.08;
 segConfBaseDepth.center[0]=-252.23; segConfBaseDepth.center[1]=393.59; segConfBaseDepth.center[2]=925.00;
 segConfBaseDepth.planeNormalOffset=55.0;
 segConfBaseDepth.enablePlaneSegmentation=1;
}
