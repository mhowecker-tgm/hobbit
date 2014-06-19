#include "process.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "RGBDAcquisition/acquisitionSegment/AcquisitionSegment.h"


struct SegmentationFeaturesRGB segConfRGB={0};
struct SegmentationFeaturesDepth segConfDepth={0};
unsigned int combinationMode=0;


unsigned int emergencyDetected=0;
float temperatureDetected=0.0;


int processNewTemperatureReading(float temperature)
{
    temperatureDetected=temperature;
}


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



       fprintf(stderr,"Freeing\n");
       free (segmentedRGB);
       free (segmentedDepth);
       fprintf(stderr,"Done\n");

 return 0;
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
