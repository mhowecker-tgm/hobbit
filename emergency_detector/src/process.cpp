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

unsigned int tempZoneWidth = 300;
unsigned int tempZoneHeight = 200;

unsigned int minScoreTrigger = 1500;
unsigned int maxScoreTrigger = 2000;

unsigned int doCVOutput=0;
unsigned int emergencyDetected=0;

float temperatureAmbientDetected=0.0; //<- YODO : default value should be 0
float temperatureObjectDetected=0.0; //<- YODO : default value should be 0
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
   fprintf(stderr,"Received BBOX center(%0.2f,%0.2f,%0.2f) size(%0.2f,%0.2f,%0.2f) ts(%u)\n", ctX,ctY,ctZ, sizeX , sizeY , sizeZ , bboxTimeStamp);

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

  unsigned int temperatureFrameOffset = ABSDIFF(frameTimestamp,tempTimestamp);



  if ( (32<temperatureObjectDetected) && (temperatureObjectDetected<37) && (temperatureFrameOffset < maximumFrameDifferenceForTemperatureToBeRelevant )  )
    {
        fprintf(stderr,"runServicesThatNeedColorAndDepth called \n");
         segmentedRGB = copyRGB(colorFrame ,colorWidth , colorHeight);
         segmentedDepth = copyDepth(depthFrame ,depthWidth , depthHeight);
        fprintf(stderr,"Copied rgb/depth\n");

        fprintf(stderr,"Segmenting 2 frames sized  %ux%u and %ux%u \n",colorWidth , colorHeight,depthWidth , depthHeight);
        segmentRGBAndDepthFrame (
                                   segmentedRGB ,
                                   segmentedDepth ,
                                   colorWidth , colorHeight,
                                   &segConfRGB ,
                                   &segConfDepth ,
                                   0 ,
                                   combinationMode
                                );


      depthAvg = viewPointChange_countDepths( segmentedDepth , colorWidth , colorHeight , tempZoneStartX , tempZoneStartY , tempZoneWidth , tempZoneHeight , maxScoreTrigger , 1 );
      fprintf(stderr,"RECT Score is %u \n",depthAvg);

      if (
           ( depthAvg > minScoreTrigger) &&
           ( depthAvg < maxScoreTrigger)
         )
         {
           if (userIsStanding(&fallDetectionContext,frameTimestamp))
            {
             fprintf(stderr,RED "\n\n  We have a standing user , so he is taking care of fallen user , will not emit emergency \n\n" NORMAL );
            } else
            {
              fprintf(stderr,MAGENTA "\n\n  Already Fallen User Detected , EMERGENCY \n\n" NORMAL);
              emergencyDetected=1;
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

        //temperatureObjectDetected+=0.5;
        //if (temperatureObjectDetected>40) { temperatureObjectDetected=28;}

        Scalar tempColor = Scalar ( tempColorB , tempColorG , tempColorR );
        circle(bgrMat,  centerPt , 20 , tempColor , 4, 8 , 0);

        char rectVal[123]={0};



        int fontUsed=FONT_HERSHEY_SIMPLEX; //FONT_HERSHEY_SCRIPT_SIMPLEX;
        Point txtPosition;  txtPosition.x = pt1.x+15; txtPosition.y = pt1.y+20;
        if (segmentedRGB==0)
        {
          putText(bgrMat , "Low Temp / Power Save" , txtPosition , fontUsed , 0.7 , color , 2 , 8 );
        } else
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
        {
          putText(bgrMat , "Scanning for emergency .." , txtPosition , fontUsed , 0.7 , color , 2 , 8 );
        }
        txtPosition.y += 24; snprintf(rectVal,123,"Score : %u",depthAvg);
        putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , color , 2 , 8 );
        txtPosition.y += 24; snprintf(rectVal,123,"Temperature : %0.2f C",temperatureObjectDetected);
        putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , color , 2 , 8 );

       if (userIsRecent(&fallDetectionContext,frameTimestamp))
       {
        unsigned int i=0;
        for (i=0; i<fallDetectionContext.numberOfJoints; i++)
         {
           Scalar jointColor = Scalar ( 0 , 255 , 0 );
           if ( ( fallDetectionContext.lastJoint2D[i].x!=0.0 ) || ( fallDetectionContext.lastJoint2D[i].y!=0.0) )
           {
               centerPt.x=fallDetectionContext.lastJoint2D[i].x;       centerPt.y=fallDetectionContext.lastJoint2D[i].y;
               circle(bgrMat,  centerPt , 10 , jointColor , -4, 8 , 0);
           }
         }

       }


	    cv::imshow("emergency_detector segmented rgb",bgrMat);
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
