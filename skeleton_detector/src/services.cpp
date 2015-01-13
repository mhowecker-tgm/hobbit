#include "services.h"

//#include "HobbitTrackerLib.h"

#include <ros/ros.h>
#include <ros/spinner.h>
#include "pose.h"

//#define SKPREFIX "SK_"
#define SKPREFIX ""


#define NORMAL "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */


#define BROADCAST_HOBBIT 1
#define MAXIMUM_DISTANCE_FOR_POINTING 400

#include <skeleton_detector/Person.h>
#include <skeleton_detector/Skeleton2D.h>
#include <skeleton_detector/Skeleton3D.h>
#include <skeleton_detector/SkeletonBBox.h>

#include "hobbit_msgs/Fitness.h"

#define USE_PERSON_AGGREGATOR 1

#if USE_PERSON_AGGREGATOR
 #define PERSON_TOPIC "/skeleton_detector/persons"
#else
 #define PERSON_TOPIC "persons"
#endif // USE_PERSON_AGGREGATOR



#include "hobbit_msgs/Event.h"
#include <std_msgs/String.h>
ros::Publisher gestureEventBroadcaster;

#include "skeleton_detector/PointEvents.h"

ros::Publisher joint2DBroadcaster;
ros::Publisher joint3DBroadcaster;
ros::Publisher jointBBoxBroadcaster;
ros::Publisher personBroadcaster;
ros::Publisher pointEventsBroadcaster;

ros::Publisher  fitnessXPCBroadcaster;
ros::Subscriber fitnessTabletSubscriber;

#define divisor 1000
//ros::Publisher gestureBroadcaster;

unsigned char dontPublishSkeletons=0;
unsigned char dontPublishPointEvents=0;
unsigned char dontPublishPersons=0;

unsigned int processingMode = PROCESSING_MODE_UPPER_GESTURE_BODY_TRACKER;//PROCESSING_MODE_SIMPLE_PERSON_DETECTOR;

unsigned int actualTimestamp=0;
unsigned int actualInFieldOfView=0;
float actualX=0.0,actualY=0.0,actualZ=0.0,actualTheta=0.0,actualConfidence=0.51;

unsigned char * colorFrameCopy=0;   unsigned int colorCopyWidth = 0; unsigned int colorCopyHeight = 0;
unsigned short * depthFrameCopy =0; unsigned int depthCopyWidth = 0; unsigned int depthCopyHeight = 0;


int reallocateCopySpaceIfNeccessary(
                                     unsigned int newColorWidth , unsigned int newColorHeight ,
                                     unsigned int newDepthWidth , unsigned int newDepthHeight
                                  )
{
  if  ( (newColorWidth * newColorHeight * 3 > colorCopyWidth * colorCopyHeight * 3 ) )
  {
    if (colorFrameCopy!=0) { free(colorFrameCopy); colorFrameCopy=0; }
    colorFrameCopy = (unsigned char * ) malloc(newColorWidth*newColorHeight*3*sizeof(unsigned char));
    if (colorFrameCopy==0) { fprintf(stderr,"Cannot make an intermediate copy of color frame \n");  }
    colorCopyWidth=newColorWidth;
    colorCopyHeight=newColorHeight;
  }


  if  ( (newDepthWidth * newDepthHeight  > depthCopyWidth * depthCopyHeight ) )
  {
    if (depthFrameCopy!=0) { free(depthFrameCopy); depthFrameCopy=0; }
    depthFrameCopy = (unsigned short * ) malloc(newDepthWidth*newDepthHeight*1*sizeof(unsigned short));
    if (depthFrameCopy==0) { fprintf(stderr,"Cannot make an intermediate copy of depth frame \n"); }
    depthCopyWidth=newDepthWidth;
    depthCopyHeight=newDepthHeight;
  }




  return 1;
}


float simpPow(float base,unsigned int exp)
{
    if (exp==0) return 1;
    float retres=base;
    unsigned int i=0;
    for (i=0; i<exp-1; i++)
    {
        retres*=base;
    }
    return retres;
}


void broadcastNewPerson()
{
  if (dontPublishPersons) { return ; }

  skeleton_detector::Person msg;
  msg.x = actualX;
  msg.y = actualY;
  msg.z = actualZ;
  msg.source = 2;
  msg.theta = actualTheta;

  msg.inFieldOfView = actualInFieldOfView;
  msg.confidence = actualConfidence;
  msg.timestamp=actualTimestamp;

  fprintf(stderr,GREEN "Publishing a new Person\n" NORMAL);
  personBroadcaster.publish(msg);
  //ros::spinOnce();
}


void broadcast2DJoints(struct skeletonHuman * skeletonFound)
{
  if (dontPublishPersons) { return ; }

  skeleton_detector::Skeleton2D msg;
  msg.joints2D.resize(HUMAN_SKELETON_PARTS * 2, 0.0);

  for (unsigned int i=0; i<HUMAN_SKELETON_PARTS; i++)
  {
    msg.joints2D[2*i+0]=skeletonFound->joint2D[i].x;
    msg.joints2D[2*i+1]=skeletonFound->joint2D[i].y;
  }
  msg.numberOfJoints=HUMAN_SKELETON_PARTS;
  msg.timestamp=actualTimestamp;

  joint2DBroadcaster.publish(msg);
}


void broadcast3DJoints(struct skeletonHuman * skeletonFound)
{
  if (dontPublishPersons) { return ; }

  skeleton_detector::Skeleton3D msg;
  msg.joints3D.resize(HUMAN_SKELETON_PARTS * 3, 0.0);

  for (unsigned int i=0; i<HUMAN_SKELETON_PARTS; i++)
  {
    msg.joints3D[3*i+0]=skeletonFound->joint[i].x;
    msg.joints3D[3*i+1]=skeletonFound->joint[i].y;
    msg.joints3D[3*i+2]=skeletonFound->joint[i].z;
  }
  msg.numberOfJoints=HUMAN_SKELETON_PARTS;
  msg.timestamp=actualTimestamp;

  joint3DBroadcaster.publish(msg);
}

void broadcast2DBBox(struct skeletonHuman * skeletonFound)
{
  if (dontPublishPersons) { return ; }

  skeleton_detector::SkeletonBBox msg;

  msg.width3D=skeletonFound->bboxDimensions.x;
  msg.height3D=skeletonFound->bboxDimensions.y;
  msg.depth3D=skeletonFound->bboxDimensions.z;

  msg.centerX3D = skeletonFound->centerOfMass.x;
  msg.centerY3D = skeletonFound->centerOfMass.y;
  msg.centerZ3D = skeletonFound->centerOfMass.z;

  msg.width2D=skeletonFound->bboxDimensions.x;
  msg.height2D=skeletonFound->bboxDimensions.y;

  msg.centerX2D = skeletonFound->centerOfMass.x;
  msg.centerY2D = skeletonFound->centerOfMass.y;

  msg.timestamp=actualTimestamp;

  jointBBoxBroadcaster.publish(msg);
}



void broadcastNewRepetition(unsigned int frameNumber,struct exerciseData * exercise)
{
  hobbit_msgs::Fitness msg;
  std::stringstream ss;
  ss<<"C_EXERCISE_REPΕΤΙΤΙΟΝ";
  msg.command=ss.str();

  fitnessXPCBroadcaster.publish(msg);
}


void fitnessRecvMessage(const hobbit_msgs::Fitness & msg)
{
  //Test Trigger with rostopic pub /fitness_tablet hobbit_msgs/Fitness " { command: C_EXERCISE_STARTED , params: [  { name: '1' , value: 'STARTED' } ] } " -1
  if (strcmp("C_EXERCISE_STARTED",msg.command.c_str())==0)
  {
    unsigned int isLeftHand=0;
    unsigned int exerciseID =atoi(msg.params[0].name.c_str());
    fprintf(stderr,"Started Exercise %u \n",exerciseID);

    if ( (MORE_IS_LEFT_EXERCISE<exerciseID) && (exerciseID<LESS_IS_LEFT_EXERCISE) )
    {
      isLeftHand=1;
    }

    hobbitFitnessFunction_StartExercise(actualTimestamp,exerciseID,isLeftHand,5/*TODO*/);
  }
  else
 //Test Trigger with rostopic pub /fitness_tablet hobbit_msgs/Fitness " { command: C_EXERCISE_STOPPED , params: [  { name: '1' , value: 'STOPPED' } ] } " -1
  if (strcmp("C_EXERCISE_STOPPED",msg.command.c_str())==0)
  {
    fprintf(stderr,"Stopped Exercises\n");

    hobbitFitnessFunction_StopExercise(actualTimestamp);
   }
  else
  {
    fprintf(stderr,"Unknown command arrived %s \n",msg.command.c_str());
  }
}





void broadcastNewGesture(unsigned int frameNumber,struct handGesture * gesture)
{
    if (gesture==0)  { return ; }

    hobbit_msgs::Event evt;
    std::stringstream ss;
    switch (gesture->gestureID)
    {
     case GESTURE_NONE   : ss<<"G_NONE";   break;
     case GESTURE_CANCEL : ss<<"G_CANCEL"; break;
     case GESTURE_HELP   : ss<<"G_HELP";   break;
     case GESTURE_YES    : ss<<"G_YES";    break;
     case GESTURE_NO     : ss<<"G_NO";     break;
     case GESTURE_REWARD : ss<<"G_REWARD"; break;
     case GESTURE_POINT  : ss<<"G_POINT";  break;
     case GESTURE_COME   : ss<<"G_COME";  break;
     case GESTURE_WAVE   : ss<<"G_WAVE";  break;
     default :             ss<<"G_NOTFOUND"; break;
    };

    //sROS.data=ss.str();
    evt.event=ss.str();
    evt.header.seq = frameNumber;
    evt.header.frame_id = "skeleton_detector";
    evt.header.stamp = ros::Time::now();
    evt.sessionID  = "SessionID";
    evt.confidence = 1.0;
    evt.params.resize(0);
    fprintf(stderr,"Publishing a new Event ( %u ) \n",gesture->gestureID);
    gestureEventBroadcaster.publish(evt);


    return ;
}


void broadcastPointing(unsigned int frameNumber ,struct skeletonPointing * skeletonPointingFound)
{
  if (dontPublishPointEvents) { return ; }

  if (
       ( (skeletonPointingFound->pointingVector.x>-0.0005)&&(skeletonPointingFound->pointingVector.x<0.0005) ) &&
       ( (skeletonPointingFound->pointingVector.y>-0.0005)&&(skeletonPointingFound->pointingVector.y<0.0005) ) &&
       ( (skeletonPointingFound->pointingVector.z>-0.0005)&&(skeletonPointingFound->pointingVector.z<0.0005) )
     )
  {
    fprintf(stderr,YELLOW "Skeleton is not pointing somewhere\n" NORMAL);
    return ;
  }

  //David Wants to Flip Y
  signed int YFlipper = -1;

  skeleton_detector::PointEvents msg;
  msg.x = skeletonPointingFound->pointStart.x;
  msg.y = YFlipper*skeletonPointingFound->pointStart.y;
  msg.z = skeletonPointingFound->pointStart.z;

  msg.vectorX = skeletonPointingFound->pointingVector.x;
  msg.vectorY = YFlipper*skeletonPointingFound->pointingVector.y;
  msg.vectorZ = skeletonPointingFound->pointingVector.z;

  msg.leftHand = skeletonPointingFound->isLeftHand;
  msg.rightHand = skeletonPointingFound->isRightHand;

  msg.timestamp=frameNumber;

  fprintf(stderr,GREEN "Publishing a new Pointing Event\n" NORMAL);
  pointEventsBroadcaster.publish(msg);

}


int considerSkeletonPointing(unsigned int frameNumber,struct skeletonHuman * skeletonFound)
{
  struct skeletonPointing skelPF={0};

  float distanceLeft = sqrt(
                             simpPow(skeletonFound->joint[HUMAN_SKELETON_TORSO].x - skeletonFound->joint[HUMAN_SKELETON_LEFT_HAND].x ,2)  +
                             simpPow(skeletonFound->joint[HUMAN_SKELETON_TORSO].y - skeletonFound->joint[HUMAN_SKELETON_LEFT_HAND].y ,2)  +
                             simpPow(skeletonFound->joint[HUMAN_SKELETON_TORSO].z - skeletonFound->joint[HUMAN_SKELETON_LEFT_HAND].z ,2)
                           );
  float distanceRight = sqrt(
                             simpPow(skeletonFound->joint[HUMAN_SKELETON_TORSO].x - skeletonFound->joint[HUMAN_SKELETON_RIGHT_HAND].x ,2)  +
                             simpPow(skeletonFound->joint[HUMAN_SKELETON_TORSO].y - skeletonFound->joint[HUMAN_SKELETON_RIGHT_HAND].y ,2)  +
                             simpPow(skeletonFound->joint[HUMAN_SKELETON_TORSO].z - skeletonFound->joint[HUMAN_SKELETON_RIGHT_HAND].z ,2)
                             );


  if ( (distanceLeft<MAXIMUM_DISTANCE_FOR_POINTING) && (distanceRight<MAXIMUM_DISTANCE_FOR_POINTING) ) { fprintf(stderr,"Cutting off pointing "); return 0; }






  int doHand=1; //1 = right , 2 =left
  if (distanceLeft<distanceRight) { doHand=2; }

  if (doHand==2)
  {
   skelPF.pointStart.x = skeletonFound->joint[HUMAN_SKELETON_LEFT_ELBOW].x;
   skelPF.pointStart.y = skeletonFound->joint[HUMAN_SKELETON_LEFT_ELBOW].y;
   skelPF.pointStart.z = skeletonFound->joint[HUMAN_SKELETON_LEFT_ELBOW].z;
   skelPF.pointEnd.x = skeletonFound->joint[HUMAN_SKELETON_LEFT_HAND].x;
   skelPF.pointEnd.y = skeletonFound->joint[HUMAN_SKELETON_LEFT_HAND].y;
   skelPF.pointEnd.z = skeletonFound->joint[HUMAN_SKELETON_LEFT_HAND].z;
   skelPF.pointingVector.x = skelPF.pointEnd.x - skelPF.pointStart.x;
   skelPF.pointingVector.y = skelPF.pointEnd.y - skelPF.pointStart.y;
   skelPF.pointingVector.z = skelPF.pointEnd.z - skelPF.pointStart.z;
   skelPF.isLeftHand=1;
   skelPF.isRightHand=0;

   broadcastPointing(frameNumber,&skelPF);
   return 1;
  } else
  if (doHand==1)
  {
   skelPF.pointStart.x = skeletonFound->joint[HUMAN_SKELETON_RIGHT_ELBOW].x;
   skelPF.pointStart.y = skeletonFound->joint[HUMAN_SKELETON_RIGHT_ELBOW].y;
   skelPF.pointStart.z = skeletonFound->joint[HUMAN_SKELETON_RIGHT_ELBOW].z;
   skelPF.pointEnd.x = skeletonFound->joint[HUMAN_SKELETON_RIGHT_HAND].x;
   skelPF.pointEnd.y = skeletonFound->joint[HUMAN_SKELETON_RIGHT_HAND].y;
   skelPF.pointEnd.z = skeletonFound->joint[HUMAN_SKELETON_RIGHT_HAND].z;
   skelPF.pointingVector.x = skelPF.pointEnd.x - skelPF.pointStart.x;
   skelPF.pointingVector.y = skelPF.pointEnd.y - skelPF.pointStart.y;
   skelPF.pointingVector.z = skelPF.pointEnd.z - skelPF.pointStart.z;
   skelPF.isLeftHand=0;
   skelPF.isRightHand=1;

   broadcastPointing(frameNumber,&skelPF);
   return 1;
  }

 return 0;
}


void broadcastNewSkeleton(unsigned int frameNumber,unsigned int skeletonID , struct skeletonHuman * skeletonFound )
{
    if ( (dontPublishSkeletons) || (skeletonFound==0) ) { return ; }

  fprintf(stderr,GREEN "Broadcasting a skeleton at TF\n" NORMAL);
  unsigned int i=0;

  //DO flips here ?
  for (i=0; i<HUMAN_SKELETON_PARTS; i++)
    {
      skeletonFound->joint[i].z = -1 * skeletonFound->joint[i].z;
    }

     //Do TF Broadcast here
     char tag[256]={0};
     for ( i=0; i<HUMAN_SKELETON_PARTS; i++ )
      {
        if (
              (skeletonFound->joint[i].x!=0) ||
              (skeletonFound->joint[i].y!=0) ||
              (skeletonFound->joint[i].z!=0)
           )
            {
             sprintf(tag,"%s%s",SKPREFIX,jointNames[i]);
             postPoseTransform(tag,skeletonFound->joint[i].x/divisor,skeletonFound->joint[i].y/divisor,skeletonFound->joint[i].z/divisor);
            }
      }

     for ( i=0; i<8; i++ )
      {
       sprintf(tag,SKPREFIX "bbox/point%u",i);
       postPoseTransform(tag,skeletonFound->bbox[i].x/divisor,skeletonFound->bbox[i].y/divisor,-1* skeletonFound->bbox[i].z/divisor);
      }

      actualTimestamp=frameNumber;
      actualX=skeletonFound->joint[HUMAN_SKELETON_HEAD].x;
      actualY=-1.0*skeletonFound->joint[HUMAN_SKELETON_HEAD].y;
      actualZ=skeletonFound->joint[HUMAN_SKELETON_HEAD].z;
      broadcastNewPerson();

      considerSkeletonPointing(frameNumber,skeletonFound);

      if (processingMode != PROCESSING_MODE_SIMPLE_PERSON_DETECTOR)
                                    { broadcast2DJoints(skeletonFound); } // We only produce 2d Joints when we have hands ( i.e. when not using simple person detector )
      broadcast2DBBox(skeletonFound);

    return ;
}

int runServicesThatNeedColorAndDepth(unsigned char * colorFrame , unsigned int colorWidth , unsigned int colorHeight ,
                                       unsigned short * depthFrame  , unsigned int depthWidth , unsigned int depthHeight ,
                                        struct calibrationHobbit * calib ,
                                         unsigned int frameTimestamp )
{
  if ( (colorFrameCopy==0) ||  (depthFrameCopy==0) ) { fprintf(stderr,"Cannot run handtracker due to not allocated intermediate buffer\n"); return 0; }

  //Make sure we have enough space on our buffers..!
  reallocateCopySpaceIfNeccessary(colorWidth , colorHeight , depthWidth ,depthHeight );

  //Unfortunately gestures need its dedicated frame buffer read/write so we copy frames here before passing them
  memcpy(colorFrameCopy,colorFrame,colorWidth*colorHeight*3*sizeof(unsigned char));
  memcpy(depthFrameCopy,depthFrame,depthWidth*depthHeight*1*sizeof(unsigned short));

 // fprintf(stderr,"Passing new frame .. ");
  int retres = hobbitUpperBodyTracker_NewFrame(colorFrameCopy , colorWidth , colorHeight ,
                                               depthFrameCopy  , depthWidth , depthHeight ,
                                               calib ,
                                               processingMode ,
                                               frameTimestamp );

 // fprintf(stderr," survived \n");


  return retres;
}




int registerServices(ros::NodeHandle * nh,unsigned int width,unsigned int height)
{
  colorFrameCopy = (unsigned char * ) malloc(width*height*3*sizeof(unsigned char));
  if (colorFrameCopy==0) { fprintf(stderr,"Cannot make an intermidiate copy of color frame \n");  }
  depthFrameCopy = (unsigned short * ) malloc(width*height*1*sizeof(unsigned short));
  if (depthFrameCopy==0) { fprintf(stderr,"Cannot make an intermidiate copy of depth frame \n"); }

  hobbitUpperBodyTracker_Initialize(width , height);
  hobbitUpperBodyTracker_RegisterSkeletonDetectedEvent((void *) &broadcastNewSkeleton);
  hobbitUpperBodyTracker_setFloor(51.46 , 622.97 , 1722.0 , 0.02 , -0.91 , -0.42); //default plane
  hobbitUpperBodyTracker_useGestures(1);
  gestureEventBroadcaster = nh->advertise <hobbit_msgs::Event> ("Event", 1000);
  hobbitUpperBodyTracker_RegisterGestureDetectedEvent((void *) &broadcastNewGesture);


  hobbitFitnessFunction_RegisterExerciseRepetitionDetected((void *) &broadcastNewRepetition);
  //hobbitFitnessFunction_RegisterExerciseRepetitionErrorDetected(void * callback)
  //hobbitFitnessFunction_RegisterExerciseRepetitionBatchCompleted(void * callback)


  pointEventsBroadcaster = nh->advertise <skeleton_detector::PointEvents> ("pointEvents", 1000);
  personBroadcaster = nh->advertise <skeleton_detector::Person> (PERSON_TOPIC, divisor);
  joint2DBroadcaster = nh->advertise <skeleton_detector::Skeleton2D> ("joints2D", 1000);
  joint3DBroadcaster = nh->advertise <skeleton_detector::Skeleton3D> ("joints3D", 1000);
  jointBBoxBroadcaster = nh->advertise <skeleton_detector::SkeletonBBox> ("jointsBBox", 1000);

  fitnessXPCBroadcaster = nh->advertise <hobbit_msgs::Fitness> ("fitness_xpc", 1000);
  fitnessTabletSubscriber = nh->subscribe("fitness_tablet",1000,fitnessRecvMessage);

}

int stopServices()
{
  hobbitUpperBodyTracker_Close();

  if (colorFrameCopy!=0) { free(colorFrameCopy); colorFrameCopy=0; }
  if (depthFrameCopy!=0) { free(depthFrameCopy); depthFrameCopy=0; }
}
