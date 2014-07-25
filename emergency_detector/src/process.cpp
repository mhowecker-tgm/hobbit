#include "process.h"
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

struct SegmentationFeaturesRGB segConfRGB={0};
struct SegmentationFeaturesDepth segConfDepth={0};
unsigned int combinationMode=0;


unsigned int doCVOutput=0;
unsigned int emergencyDetected=0;
float temperatureAmbientDetected=36.0; //<- YODO : default value should be 0
float temperatureObjectDetected=36.0; //<- YODO : default value should be 0


int processBoundingBox(float sizeX,float sizeY,float sizeZ)
{
  if (sizeZ!=0)
  {
   fprintf(stderr,"Received BBOX %f , %f , %f \n", sizeX , sizeY , sizeZ );
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
  if ( (35<temperatureObjectDetected) && (temperatureObjectDetected<37)  )
    {
        fprintf(stderr,"runServicesThatNeedColorAndDepth called \n");
        unsigned char * segmentedRGB = copyRGB(colorFrame ,colorWidth , colorHeight);
        unsigned short * segmentedDepth = copyDepth(depthFrame ,depthWidth , depthHeight);
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

      unsigned int depthAvg = viewPointChange_countDepths( segmentedDepth , colorWidth , colorHeight , 147 , 169 , 300 , 200 , 1000 );
      fprintf(stderr,"RECT Score is %u \n",depthAvg);

      if (doCVOutput)
      {
        cv::Mat segDepth(depthHeight,depthWidth,CV_16UC1 ,segmentedDepth,depthWidth);
	    cv::Mat segDepthNorm;
	    cv::normalize(segDepth,segDepthNorm,0,65536,CV_MINMAX,CV_16UC1);
	    cv::imshow("emergency_detector segmented depth",segDepthNorm);
      }

      if (
           ( depthAvg > 1000) &&
           ( depthAvg < 2000)
         )
                       {
                        fprintf(stderr,MAGENTA "\n\n ? EMERGENCY ?  \n\n" NORMAL);
                         // emergencyDetected=1;
                       }



       fprintf(stderr,"Freeing\n");
       free (segmentedRGB);
       free (segmentedDepth);
       fprintf(stderr,"Done\n");
    }
 return emergencyDetected;
}


void initializeProcess()
{
 initializeRGBSegmentationConfiguration(&segConfRGB,640,480);
 initializeDepthSegmentationConfiguration(&segConfDepth,640,480);


 segConfDepth.maxDepth=1800;
 //Hobbit orientation according to camera

 segConfDepth.doNotGenerateNormalFrom3Points=1;

 segConfDepth.normal[0]=-0.02;
 segConfDepth.normal[1]=-0.78;
 segConfDepth.normal[2]=-0.62;

 segConfDepth.center[0]=114.95;
 segConfDepth.center[1]=501.61;
 segConfDepth.center[2]=1338.0;

 segConfDepth.planeNormalOffset=40.0;
}
