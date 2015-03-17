#include "tools.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "classifier.h"


#define ABSDIFF(num1,num2) ( (num1-num2) >=0 ? (num1-num2) : (num2 - num1) )

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
   fprintf(fd, "#HOBBIT_EMERGENCY_TEMPERATURE(%0.2f) OBSTACLE_CHECKS(%u,%u,%u)\n",lastState.objectTemperature,lastState.mapShouldBeClear[0],lastState.mapShouldBeClear[1],lastState.mapShouldBeClear[2]);
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




unsigned int temperatureSensorSensesHuman(float tempDetected , unsigned int tempTimestamp , unsigned int frameTimestamp)
{
 unsigned int temperatureFrameOffset = ABSDIFF(frameTimestamp,tempTimestamp);
  if (
       (minimums.objectTemperature<=tempDetected) &&
       (tempDetected<=maximums.objectTemperature) &&
       (temperatureFrameOffset < maximumFrameDifferenceForTemperatureToBeRelevant )
     )
    {
        fprintf(stderr,"HUMAN TEMP %0.2f <= %0.2f <= %0.2f , sensor lag %u ",minimums.objectTemperature , tempDetected , maximums.objectTemperature , temperatureFrameOffset);
        return 1;
    }
   fprintf(stderr,"NONHUMAN TEMP %0.2f <!= %0.2f <!= %0.2f , sensor lag %u ",minimums.objectTemperature , tempDetected , maximums.objectTemperature , temperatureFrameOffset);
   return 0;
}
