#ifndef EXTACQUISITION_H_INCLUDED
#define EXTACQUISITION_H_INCLUDED


extern int devID;
extern int useRGBDAcqusition;
extern unsigned int width;
extern unsigned int height;

extern unsigned int draw_out;


void externalAcquisitionCallback();

void switchDrawOutTo(unsigned int newVal);


int doDrawOutFrame( unsigned char * rgbFrame , unsigned int rgbWidth , unsigned int rgbHeight ,
                     unsigned char * depthFrame , unsigned int depthWidth , unsigned int depthHeight );
int doDrawOut();

int getKeyPressed();

void switchDrawOutTo(unsigned int newVal);


int  acquistionStartUp(char * moduleName , unsigned int devUsed , char * from , unsigned int width , unsigned int height , unsigned int fps);
int acquisitionRewind();
#endif // ACQUISITION_H_INCLUDED
