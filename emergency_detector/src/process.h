
#ifndef EMERGENCY_DETECTOR_H_INCLUDED
#define EMERGENCY_DETECTOR_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif


extern unsigned int doCVOutput;
extern unsigned int emergencyDetected;
extern unsigned int personDetected;
extern unsigned int autoPlaneSegmentation;



extern float temperatureX,temperatureY,temperatureZ;
extern float temperatureAmbientDetected;
extern float temperatureObjectDetected;
extern unsigned int tempTimestamp;



extern  int maximumFrameDifferenceForTemperatureToBeRelevant;

extern  int minimumAllowedHolePercentage;
extern  int maximumAllowedHolePercentage;

extern double minHumanTemperature;
extern double maxHumanTemperature;

extern  int tempZoneWidth;
extern  int tempZoneHeight;

extern  int minScoreTrigger;
extern  int maxScoreTrigger;


int setHobbitEMode();

int increasePlane();
int decreasePlane();

int processNewTemperatureReading(float temperature);

int processBoundingBox( float ctX,float ctY,float ctZ,
                        float sizeX,float sizeY,float sizeZ,
                        unsigned int matchingTimestamp);


int runServicesThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                       unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                        void * calib ,
                                          unsigned int frameTimestamp );



int runServicesBottomThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                           unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                           void * calib ,
                                           unsigned int frameTimestamp );

void initializeProcess();

#ifdef __cplusplus
}
#endif


#endif
