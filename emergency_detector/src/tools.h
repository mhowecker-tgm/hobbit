#ifndef EMERG_TOOLS_H_INCLUDED
#define EMERG_TOOLS_H_INCLUDED



unsigned int simplePow(unsigned int base,unsigned int exp);
int saveRawImageToFile(const char * filename,unsigned char * pixels , unsigned int width , unsigned int height , unsigned int channels , unsigned int bitsperpixel);
int detectHighContrastUnusableRGB(unsigned char * rgbFrame , unsigned int width , unsigned int height , float percentageHigh) ;
unsigned char * copyRGB(unsigned char * source , unsigned int width , unsigned int height) ;
unsigned short * copyDepth(unsigned short * source , unsigned int width , unsigned int height);
unsigned int temperatureSensorSensesHuman(unsigned int tempDetected , unsigned int tempTimestamp , unsigned int frameTimestamp);


#endif
