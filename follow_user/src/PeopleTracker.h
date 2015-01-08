#ifndef PEOPLE_TRACKER_H_INCLUDED
#define PEOPLE_TRACKER_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif


struct peopleTrackerTarget
{
	float x,y,vx,vy,rad;
	int id,state,status;
	//enum ACTOR_STATE
};


struct peopleTrackerMotion
{
	float dX,dY,dZ;
	float angleX,angleY,angleZ;
};


struct peopleTrackerContext
{
  void * tracker;
  unsigned int framesProcessed;
  unsigned int visualizationEnabled;

  void * targetDetectedCallback;
  void * positionUpdateCallback;
};


int peopleTracker_RegisterTargetDetectedEvent(struct peopleTrackerContext * ptcx,void * callback);
int peopleTracker_RegisterPositionUpdateEvent(struct peopleTrackerContext * ptcx,void * callback);

int peopleTracker_Reset(struct peopleTrackerContext * ptcx);

struct peopleTrackerContext * peopleTracker_Initialize(const char * optionsFile);
int peopleTracker_Close(struct peopleTrackerContext * ptcx);

int peopleTracker_SetVisualization(struct peopleTrackerContext * ptcx,unsigned int active);


int peopleTracker_NewFrame(
                            struct peopleTrackerContext * ptcx ,
                            unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                            unsigned short * depthFrame , unsigned int depthWidth , unsigned int depthHeight ,
                            unsigned int frameTimestamp
                           );


#ifdef __cplusplus
}
#endif


#endif // PEOPLE_TRACKER
