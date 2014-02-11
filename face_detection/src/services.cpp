#include "services.h"
#include "pose.h"

#include "FaceDetection.h"

#include <ros/ros.h>
#include <ros/spinner.h>

#include "face_detection/Person.h"

ros::Publisher personBroadcaster;

unsigned char dontPublishPersons=0;

unsigned int actualTimestamp=0;
float actualX=0.0,actualY=0.0,actualZ=0.0,actualTheta=0.0,actualConfidence=0.0;

unsigned char * colorFrameCopy=0;
unsigned short * depthFrameCopy =0;


void broadcastNewPerson()
{
  if (dontPublishPersons) { return ; }

  face_detection::Person msg;
  msg.x = actualX;
  msg.y = actualY;
  msg.z = actualZ;
  msg.theta = actualTheta;

  msg.inFieldOfView = 1;
  msg.confidence = actualConfidence;
  msg.timestamp=actualTimestamp;

  fprintf(stderr,"Publishing a new Person\n");
  personBroadcaster.publish(msg);
}


void broadcastDetectedFace(unsigned int frameNumber ,struct detectedFace * faceFound)
{
    fprintf(stderr,"Broadcasting a head \n");

     actualTimestamp=frameNumber;
     actualConfidence=0.4; // We aren't particularly sure
     actualX=faceFound->headX;
     actualY=faceFound->headY;
     actualZ=faceFound->headZ;
     actualTheta=0.0;

     postPoseTransform((char*) "head",/*-1.0**/faceFound->headX/1000,/*-1.0**/faceFound->headY/1000,faceFound->headZ/1000);

     broadcastNewPerson();
}


int runServicesThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                       unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                        struct calibration * calib ,
                                         unsigned int frameTimestamp )
{
  if ( (colorFrameCopy==0) ||  (depthFrameCopy==0) ) { fprintf(stderr,"Cannot run handtracker due to not allocated intermediate buffer\n"); return 0; }
  //Unfortunately gestures need its dedicated frame buffer read/write so we copy frames here before passing them
  memcpy(colorFrameCopy,colorFrame,colorWidth*colorHeight*3*sizeof(unsigned char));
  memcpy(depthFrameCopy,depthFrame,depthWidth*depthHeight*1*sizeof(unsigned short));

  int retres = DetectFaces(frameTimestamp , colorFrameCopy , colorWidth , colorHeight ,
                                            depthFrameCopy  , depthWidth , depthHeight ,
                                            calib , 60 , 150   );

  return retres;
}


int registerServices(ros::NodeHandle * nh,unsigned int width,unsigned int height)
{
  colorFrameCopy = (unsigned char * ) malloc(width*height*3*sizeof(unsigned char));
  if (colorFrameCopy==0) { fprintf(stderr,"Cannot make an intermidiate copy of color frame \n");  }
  depthFrameCopy = (unsigned short * ) malloc(width*height*1*sizeof(unsigned short));
  if (depthFrameCopy==0) { fprintf(stderr,"Cannot make an intermidiate copy of depth frame \n"); }

  personBroadcaster = nh->advertise <face_detection::Person> ("persons", 1000);

  InitFaceDetection((char*) "haarcascade_frontalface_alt.xml");
  registerFaceDetectedEvent((void *) &broadcastDetectedFace);
}

int stopServices()
{
    CloseFaceDetection() ;

  if (colorFrameCopy!=0) { free(colorFrameCopy); colorFrameCopy=0; }
  if (depthFrameCopy!=0) { free(depthFrameCopy); depthFrameCopy=0; }
}
