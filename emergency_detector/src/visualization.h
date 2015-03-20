#ifndef EMERG_VISUALIZATION_H_INCLUDED
#define EMERG_VISUALIZATION_H_INCLUDED


extern float farBorderZ;
extern float closeBorderZ;
extern unsigned int lowBorderY;

int visualizeTopCam(unsigned char * colorFrame,unsigned char * segmentedRGB,unsigned int colorWidth , unsigned int colorHeight , unsigned int consultHobbitMap , unsigned int frameTimestamp);

int visualizeBaseCam(unsigned char * segmentedRGB,unsigned int colorWidth , unsigned int colorHeight);


#endif
