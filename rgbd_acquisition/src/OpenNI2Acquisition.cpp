#include <stdio.h>
#include <stdlib.h>
#include <string.h> //<- just for a strncpy :P

#include "OpenNI2Acquisition.h"

//#define BUILD_OPENNI2 1
//#define USE_CALIBRATION 1
//#define BUILD_OPENNI2 1

#define MOD_FACEDETECTION 0
#define MOD_NITE2 1
#define BUILD_OPENNI2 1
#define USE_WAITFORANYSTREAM_TO_GRAB 0

#if BUILD_OPENNI2

#include <unistd.h>

#include <OpenNI.h>
#include <PS1080.h>

    #if MOD_NITE2
       #include "Nite2.h"
    #endif

    #if MOD_FACEDETECTION
       #include "FaceDetection.h"
    #endif

#define MAX_OPENNI2_DEVICES 16
#define ANY_OPENNI2_DEVICE MAX_OPENNI2_DEVICES*2
#define MAX_TRIES_FOR_EACH_FRAME 10

#define DEFAULT_WIDTH 640
#define DEFAULT_HEIGHT 480

#define DEFAULT_FX 575.815735
#define DEFAULT_FY 575.815735

//Default Kinect things for reference
#define DEFAULT_FOCAL_LENGTH 120.0
#define DEFAULT_PIXEL_SIZE 0.1052

using namespace std;
using namespace openni;


int currentAllocatedDevices = 0;
Device device[MAX_OPENNI2_DEVICES];
VideoStream depth[MAX_OPENNI2_DEVICES] , color[MAX_OPENNI2_DEVICES];
VideoFrameRef depthFrame[MAX_OPENNI2_DEVICES],colorFrame[MAX_OPENNI2_DEVICES];
unsigned int frameSnapped=0;

#if USE_CALIBRATION
 struct calibration calibRGB[MAX_OPENNI2_DEVICES];
 struct calibration calibDepth[MAX_OPENNI2_DEVICES];
#endif


unsigned int lastDepthComesFromNite[MAX_OPENNI2_DEVICES]={0};
unsigned int pauseFaceDetection = 0;
unsigned int faceDetectionFramesBetweenScans = 1;
unsigned int pauseSkeletonDetection = 0;
unsigned int skeletonDetectionFramesBetweenScans = 5;


enum howToOpenDevice
{
  OPENNI2_OPEN_REGULAR_ENUM = 0,
  OPENNI2_OPEN_USING_STRING ,
  OPENNI2_OPEN_AS_MANIFEST
};


/*
   --------------------------------------------------------------------------------
   --------------------------------------------------------------------------------
                               OpenNI2 context calls
   --------------------------------------------------------------------------------
   --------------------------------------------------------------------------------
*/



int initializeOpenNI(unsigned int MAX_DEVICES_NEEDED,char * settings)
{
   unsigned int openMode=OPENNI2_OPEN_REGULAR_ENUM;
   if (settings!=0)
   {
    if (strstr(settings,".ini")!=0) { openMode=OPENNI2_OPEN_AS_MANIFEST; } else
    if (strstr(settings,".xml")!=0) { openMode=OPENNI2_OPEN_AS_MANIFEST; }
   }
   if (openMode!=OPENNI2_OPEN_REGULAR_ENUM)
   {
      fprintf(stderr,"\n\n\n\nWARNING : Opening manifests ( i.e. %s ) cannot be done in OpenNI2 \n",settings);
      fprintf(stderr,"OpenNI2 automatically parses OpenNI.ini and PS1080.ini so if you want to add something\n");
      fprintf(stderr,"do it there \n\n\n\n\n");
   }


   //Startup Everything!
   if(OpenNI::initialize()!=STATUS_OK)
    {
        fprintf(stderr,"Could not initialize OpenNI : %s \n",OpenNI::getExtendedError());
        return 0;
    }


    if (MAX_DEVICES_NEEDED>=MAX_OPENNI2_DEVICES)
      {
          fprintf(stderr,"\n\n\n\nPlease note that the current version of OpenNI2Aquisition  has a static limit of %u devices\n\n\n",MAX_OPENNI2_DEVICES);
      }
  return 1;
}


int closeOpenNI()
{
  fprintf(stderr,"Shutting down OpenNI\n");
  OpenNI::shutdown();
  return 1;
}



/*
   --------------------------------------------------------------------------------
   --------------------------------------------------------------------------------
                               OpenNI2 per device calls
   --------------------------------------------------------------------------------
   --------------------------------------------------------------------------------
*/



int getOpenNI2NumberOfDevices()
{
    Array<DeviceInfo> deviceInfoList;
    OpenNI::enumerateDevices(&deviceInfoList);
    return deviceInfoList.getSize();
}


const char * getURIForDeviceNumber(int deviceNumber,char * outURI,unsigned int maxOutURILength)
{
    //This call returns a URI for a device number
    Array<DeviceInfo> deviceInfoList;
    OpenNI::enumerateDevices(&deviceInfoList);



    if (deviceInfoList.getSize()==0)
      {
        fprintf(stderr,"\n\n\n\n\n FOUND NO DEVICES , please connect an OpenNI2 compatible device \n\n\n\n\n ");
        return openni::ANY_DEVICE;
      }


    if (deviceNumber==ANY_OPENNI2_DEVICE) { return openni::ANY_DEVICE; }
      else
    if (deviceNumber>deviceInfoList.getSize())
       {
         fprintf(stderr,"Device with index %u does not exist , our total devices are %u ..\n",deviceNumber,deviceInfoList.getSize());
         fprintf(stderr,"Returning a null URI , this will probably stop the rest of the program\n");
         return 0;
       }
      else
    if (deviceInfoList.getSize()==1)
      {
         //We only have 1 device connected and we asked for deviceNumber zero , might as well return ANY_DEVICE
         //and be done with the getURIForDeviceNumber function here..
         return openni::ANY_DEVICE;
      }


    fprintf(stderr,"Found %u device(s)\n",deviceInfoList.getSize());
    int i=0;
    for (i=0; i<deviceInfoList.getSize(); i++)
     {
      if (deviceNumber==i)
      {
        fprintf(stderr,"- - - - - - - - - -\n");
        fprintf(stderr,"Selecting device #%u\n",i);
        fprintf(stderr,"- - - - - - - - - -\n");
        fprintf(stderr,"Device Index : %u \n",i);
        fprintf(stderr,"URI : %s \n",deviceInfoList[i].getUri());
        fprintf(stderr,"Name  : %s \n",deviceInfoList[i].getName());
        fprintf(stderr,"Vendor : %s \n",deviceInfoList[i].getVendor());
        fprintf(stderr,"USB vendor:productid = %04x:%04x (lsusb style) \n",deviceInfoList[i].getUsbVendorId(),deviceInfoList[i].getUsbProductId());
        fprintf(stderr,"- - - - - - - - - -\n");

        //Returning deviceInfoList[i].getUri() may point to garbage after the function exits and objects get destructed
        //Thats why we use the buffer space provided by outURI and return our sure to work buffer instead
        if ( (outURI==0) || (maxOutURILength<strlen(deviceInfoList[i].getUri())) )
             {
                fprintf(stderr,"getURIForDeviceNumber does not have enough accomodating space for the URI so it will return any availiable device \n");
                return openni::ANY_DEVICE;
             }
        strncpy(outURI,deviceInfoList[i].getUri(),maxOutURILength);

        return outURI;
      }
    }

   return openni::ANY_DEVICE;
}

int initializeOpenNIDevice(int deviceID , char * deviceName  , Device &device , VideoStream &color , VideoStream &depth ,unsigned int width ,unsigned int height , unsigned int fps)
{
   unsigned int openMode=OPENNI2_OPEN_REGULAR_ENUM; /* 0 = regular deviceID and enumeration*/
   if (deviceName!=0)
   {
      //If our deviceName contains a .oni we assume that we have an oni file to open
      if (strstr(deviceName,".oni")!=0)
         {
           fprintf(stderr,"Found an .ONI filename , trying to open it..\n");
           openMode=OPENNI2_OPEN_USING_STRING;
         } else
      if (strlen(deviceName)>7)
        {
           fprintf(stderr,"deviceName is too long (%lu chars) , assuming it is a Device URI ..\n",strlen(deviceName));
           openMode=OPENNI2_OPEN_USING_STRING;
        }

   }

   switch (openMode)
   {
     //-------------------------------------------------------------------------------------
     //If we have an ONI file to open just pass it as an argument to device.open(deviceName)
     case OPENNI2_OPEN_USING_STRING :
      if (device.open(deviceName) != STATUS_OK)
      {
        fprintf(stderr,"Could not open using given string ( %s ) : %s \n",deviceName,OpenNI::getExtendedError());
        return 0;
      }
     break;
     //-------------------------------------------------------------------------------------
     //If we don't have a deviceName we assume deviceID points to the device we want to open so we will try to use
     //the openNI enumerator to get the specific device URI for device with number deviceID and use this to device.open( devURI )
     case OPENNI2_OPEN_REGULAR_ENUM :
     default :
      //We have to supply our own buffer to hold the uri device string , so we make one here
      char devURIBuffer[512]={0};
      if (device.open(getURIForDeviceNumber(deviceID,devURIBuffer,512)) != STATUS_OK)
      {
        fprintf(stderr,"Could not open an OpenNI device : %s \n",OpenNI::getExtendedError());
        return 0;
      }
     break;
   }

if (device.getSensorInfo(SENSOR_DEPTH)  != NULL)
    {
        Status rc = depth.create(device, SENSOR_DEPTH);
        if (rc == STATUS_OK)
        {
            VideoMode depthMode = depth.getVideoMode();
            depthMode.setResolution(width,height);
            depthMode.setFps(fps);
            Status rc = depth.setVideoMode(depthMode);
            if (rc != STATUS_OK) { fprintf(stderr,"Error getting color at video mode requested %u x %u @ %u fps\n%s\n",width,height,fps,OpenNI::getExtendedError()); }

            if(depth.start()!= STATUS_OK)
            {
                fprintf(stderr,"Couldn't start the color stream: %s \n",OpenNI::getExtendedError());
                return 0;
            }
        }
        else
        {
            fprintf(stderr,"Couldn't create depth stream: %s \n",OpenNI::getExtendedError());
            return 0;
        }
    }

    if (device.getSensorInfo(SENSOR_COLOR) != NULL)
    {
        Status rc = color.create(device, SENSOR_COLOR);
        if (rc == STATUS_OK)
        {
            VideoMode colorMode = color.getVideoMode();
            colorMode.setResolution(width,height);
            colorMode.setFps(fps);
            Status rc = color.setVideoMode(colorMode);
            if (rc != STATUS_OK) { fprintf(stderr,"Error getting depth at video mode requested %u x %u @ %u fps\n%s\n",width,height,fps,OpenNI::getExtendedError()); }

            if(color.start() != STATUS_OK)
            {
                fprintf(stderr,"Couldn't start the color stream: %s \n",OpenNI::getExtendedError());
                return 0;
            }
        }
        else
        {
            fprintf(stderr,"Couldn't create depth stream: %s \n",OpenNI::getExtendedError());
            OpenNI::getExtendedError();
            return 0;
        }
    }

    //Mirroring is disabled
    depth.setMirroringEnabled (false);
    color.setMirroringEnabled (false);


    fprintf(stdout,"Device Initialization Requested %u x %u @ %u fps \n",width,height,fps);
   return 1;
}




int closeOpenNIDevice(Device &device , VideoStream &color , VideoStream &depth)
{
    fprintf(stderr,"Stopping depth and color streams\n");
    depth.stop();
    color.stop();
    depth.destroy();
    color.destroy();
    device.close();
    return 1;
}

/*
   --------------------------------------------------------------------------------
   --------------------------------------------------------------------------------
                               OpenNI2 read frames
   --------------------------------------------------------------------------------
   --------------------------------------------------------------------------------
*/


int readFrameBlocking(VideoStream &stream,VideoFrameRef &frame , unsigned int max_tries)
{
  unsigned int tries_for_frame = 0 ;
  while (  tries_for_frame < max_tries  )
          {
            stream.readFrame(&frame);
	        if (frame.isValid()) { return 1; }
	        ++tries_for_frame;
          }

  if (!frame.isValid()) { fprintf(stderr,"Could not get a valid frame even after %u tries \n",max_tries); return 0; }
  return (tries_for_frame<max_tries);
}

int readOpenNiColorAndDepth(VideoStream &color , VideoStream &depth,VideoFrameRef &colorFrame,VideoFrameRef &depthFrame)
{
  #if USE_WAITFORANYSTREAM_TO_GRAB
   #warning "Please turn #define USE_WAITFORANYSTREAM_TO_GRAB 0"
   #warning "This is a bad idea taken from OpenNI2/Samples/SimpleViewer , we dont just want to update 'any' frame we really want to snap BOTH and do that sequentially"
   #warning "It is better to sequencially grab them instead of waiting for any stream a couple of times "
   openni::VideoStream** m_streams = new openni::VideoStream*[2];
   m_streams[0] = &depth;
   m_streams[1] = &color;


  unsigned char haveDepth=0,haveColor=0;
  int changedIndex;
  while ( (!haveDepth) || (!haveColor) )
  {
	openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
	if (rc != openni::STATUS_OK)
	{
		fprintf(stderr,"Wait failed\n");
		return 0 ;
	}

  unsigned int i=0;

	switch (changedIndex)
	{
	case 0:
		depth.readFrame(&depthFrame);
		haveDepth=1;
    break;
	case 1:
		color.readFrame(&colorFrame);
		haveColor=1;
    break;
	default:
		printf("Error in wait\n");
		return 0;
	}
  }

	delete m_streams;
	return 1;
  #else
    //Using serial frame grabbing
    readFrameBlocking(depth,depthFrame,MAX_TRIES_FOR_EACH_FRAME); // depth.readFrame(&depthFrame);
    readFrameBlocking(color,colorFrame,MAX_TRIES_FOR_EACH_FRAME); // color.readFrame(&colorFrame);

    if(depthFrame.isValid() && colorFrame.isValid()) { return 1; }

    fprintf(stderr,"Depth And Color frames are wrong!\n");
  #endif
    return 0;

}



int readOpenNiColor(VideoStream &color , VideoFrameRef &colorFrame)
{
    readFrameBlocking(color,colorFrame,MAX_TRIES_FOR_EACH_FRAME); // color.readFrame(&colorFrame);

    if (colorFrame.isValid()) { return 1; }
    fprintf(stderr,"Color frame  is wrong!\n");
    return 0;
}


/*
   --------------------------------------------------------------------------------
   --------------------------------------------------------------------------------
*/







/*


   EXTERNAL EXPOSED FUNCTIONS

*/
int mapOpenNI2DepthToRGB(int devID)
{
  device[devID].setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
  return 1;
}


int mapOpenNI2RGBToDepth(int devID)
{
  return 0;
  //Un commenting the next line leads to an error :
  //IMAGE_REGISTRATION_COLOR_TO_DEPTH was not declared in this scope , is this not implemented by OPENNI2 ?
  // device[devID].setImageRegistrationMode(IMAGE_REGISTRATION_COLOR_TO_DEPTH);
  return 1;
}

int badDeviceID(int devID,const char * callFile,unsigned int callLine)
{
  if (devID<=currentAllocatedDevices) { return 0; }
  fprintf(stderr,"Bad Device ID detected (%i/%i) , File %s : Line %u , ignoring request\n",devID,currentAllocatedDevices,callFile,callLine);
  return 1;
}


int startOpenNI2Module(unsigned int max_devs,char * settings)
{
    currentAllocatedDevices  = max_devs;
    int retres = initializeOpenNI(max_devs,settings);

    #if MOD_NITE2
       startNite2(max_devs);
   #endif

   return retres;
}

int stopOpenNI2Module()
{
   #if MOD_NITE2
       stopNite2();
   #endif

  return closeOpenNI();
}


int snapOpenNI2Frames(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
  int retres=0; // Initially failed snap :P
  #if MOD_NITE2
   //If we use nite  , things get ugly , because nite wants to snap the depth frame for itself
   //So we have to workaround not snapping a depth frame and returning the depth frame that nite snapped
       lastDepthComesFromNite[devID]=0;
       if ( (skeletonDetectionFramesBetweenScans!=0) && (!pauseSkeletonDetection) )
       {
          if ( (skeletonDetectionFramesBetweenScans==1) || (frameSnapped%skeletonDetectionFramesBetweenScans ==1) )
          {
            if ( loopNite2(devID,frameSnapped) )
            {
             //If we managed to grab a frame using NITE2 then only grab color outside
             readOpenNiColor(color[devID],colorFrame[devID]);
             #warning "Check for success of color and depth operation here"
             retres=1;
             lastDepthComesFromNite[devID]=1;
            }
          }
       }
       //If we don't have a skeleton snap do a regular snap here
       if (!lastDepthComesFromNite[devID])
       {
         retres=readOpenNiColorAndDepth(color[devID],depth[devID],colorFrame[devID],depthFrame[devID]);
       }
  #else
    //Regular Snapping of OpenNI2 both color and depth
    retres=readOpenNiColorAndDepth(color[devID],depth[devID],colorFrame[devID],depthFrame[devID]);




 #endif

    #if MOD_FACEDETECTION
       if ( (faceDetectionFramesBetweenScans!=0) && (!pauseFaceDetection)  )
       {
        if ( (faceDetectionFramesBetweenScans==1) || (frameSnapped%faceDetectionFramesBetweenScans==0) )
        {
         struct calibration calib;
         getOpenNI2ColorCalibration(devID,&calib);
         DetectFaces(frameSnapped,
                      (unsigned char*) getOpenNI2ColorPixels(devID) , getOpenNI2ColorWidth(devID) , getOpenNI2ColorHeight(devID) ,
                      (unsigned short*) getOpenNI2DepthPixels(devID) , getOpenNI2DepthWidth(devID) , getOpenNI2DepthHeight(devID) ,
                      &calib ,
                      45,150 //Min / Max
                     );
        }
       }
    #endif


  ++frameSnapped;
  return retres;
}

int NullCalibration(unsigned int width,unsigned int height, struct calibration * calib)
{
  calib->width=width;
  calib->height=height;

  calib->intrinsicParametersSet=0;
  calib->extrinsicParametersSet=0;


  calib->nearPlane=1.0;
  calib->farPlane=1000.0;

  calib->intrinsic[0]=0.0;  calib->intrinsic[1]=0.0;  calib->intrinsic[2]=0.0;
  calib->intrinsic[3]=0.0;  calib->intrinsic[4]=0.0;  calib->intrinsic[5]=0.0;
  calib->intrinsic[6]=0.0;  calib->intrinsic[7]=0.0;  calib->intrinsic[8]=1.0;

  calib->k1=0.0;  calib->k2=0.0; calib->p1=0.0; calib->p2=0.0; calib->k3=0.0;

  calib->extrinsicRotationRodriguez[0]=0.0; calib->extrinsicRotationRodriguez[1]=0.0; calib->extrinsicRotationRodriguez[2]=0.0;
  calib->extrinsicTranslation[0]=0.0; calib->extrinsicTranslation[1]=0.0; calib->extrinsicTranslation[2]=0.0;

  /*cx*/calib->intrinsic[CALIB_INTR_CX]  = (double) width/2;
  /*cy*/calib->intrinsic[CALIB_INTR_CY]  = (double) height/2;

  //-This is a bad initial estimation i guess :P
  /*fx*/ calib->intrinsic[CALIB_INTR_FX] = (double) (DEFAULT_FX * width) / DEFAULT_WIDTH;   //<- these might be wrong
  /*fy*/ calib->intrinsic[CALIB_INTR_FY] = (double) (DEFAULT_FY * height)  / DEFAULT_HEIGHT;    //<- these might be wrong
  //--------------------------------------------

  calib->depthUnit=1000.0; //Default is meters to millimeters

  return 1;
}


int FocalLengthAndPixelSizeToCalibration(double focalLength , double pixelSize ,unsigned int width,unsigned int height ,  struct calibration * calib)
{
  NullCalibration(width,height,calib);

  if (pixelSize!=0)
  {
   calib->intrinsic[CALIB_INTR_FX] = (double) focalLength/pixelSize;
   calib->intrinsic[CALIB_INTR_FY] = (double) focalLength/pixelSize;
   calib->intrinsicParametersSet=1;

   return 1;
  }

  fprintf(stderr,"FocalLengthAndPixelSizeToCalibration(with focalLength %f and pixelSize %f) cannot yield a valid calibration\n",focalLength , pixelSize);
  return 0;
}

int createOpenNI2Device(int devID,char * devName,unsigned int width,unsigned int height,unsigned int framerate)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
    if (! initializeOpenNIDevice(devID,devName,device[devID],color[devID],depth[devID],width,height,framerate) )
     {
         fprintf(stderr,"Could not initialize device with ID %u \n",devID);
         return 0;
     }

    unsigned int forceMapDepthToRGB=0;
    #if MOD_NITE2
       createNite2Device(devID,&device[devID]);
       forceMapDepthToRGB=1;
    #endif

    #if MOD_FACEDETECTION
       #warning "maybe provide a different path for the HAAR cascade for face detection although this should be ok"
       InitFaceDetection((char*) "haarcascade_frontalface_alt.xml");
       forceMapDepthToRGB=1;
    #endif

     //Snapping an initial frame to populate Image Sizes , etc..
    if ( ! snapOpenNI2Frames(devID) )
    {
     //Could not snap the initial frame
      fprintf(stderr,"Could not snap an initial frame off the OpenNI2 device\n");
      //return 0; although we have initializedOpenNI without an error what should we return here..
      //we will return 1!
    } else
    {

     //Frame Width/Height /Focal length , etc should be ok..
     fprintf(stdout,"Color Frames : %u x %u , channels %u , bitsperpixel %u \n",getOpenNI2ColorWidth(devID), getOpenNI2ColorHeight(devID) , getOpenNI2ColorChannels(devID) , getOpenNI2ColorBitsPerPixel(devID));
     fprintf(stdout,"Color Focal Length : %0.2f\n",getOpenNI2ColorFocalLength(devID));
     fprintf(stdout,"Color Pixel Size : %0.2f\n",getOpenNI2ColorPixelSize(devID));

     fprintf(stdout,"Depth Frames : %u x %u , channels %u , bitsperpixel %u \n",getOpenNI2DepthWidth(devID), getOpenNI2DepthHeight(devID), getOpenNI2DepthChannels(devID) , getOpenNI2DepthBitsPerPixel(devID));
     fprintf(stdout,"Depth Focal Length : %0.2f\n",getOpenNI2DepthFocalLength(devID));
     fprintf(stdout,"Depth Pixel Size : %0.2f\n",getOpenNI2DepthPixelSize(devID));
    }

    #if USE_CALIBRATION
     //Populate our calibration data ( if we have them
     FocalLengthAndPixelSizeToCalibration(getOpenNI2ColorFocalLength(devID),getOpenNI2ColorPixelSize(devID),getOpenNI2ColorWidth(devID),getOpenNI2ColorHeight(devID),&calibRGB[devID]);
     FocalLengthAndPixelSizeToCalibration(getOpenNI2DepthFocalLength(devID),getOpenNI2DepthPixelSize(devID),getOpenNI2DepthWidth(devID),getOpenNI2DepthHeight(devID),&calibDepth[devID]);
    #endif

    if (forceMapDepthToRGB)
       { mapOpenNI2DepthToRGB(devID); }


    return 1;
  }


 int destroyOpenNI2Device(int devID)
 {
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
     #if MOD_NITE2
       destroyNite2Device(devID);
     #endif
    #if MOD_FACEDETECTION
       CloseFaceDetection();
    #endif
     return closeOpenNIDevice(device[devID] , color[devID] , depth[devID]);
 }


int getTotalOpenNI2FrameNumber(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
  #warning "Todo , check here for oni file length etc..\n"
  //fprintf(stderr,"Todo , check here for oni file length etc..\n");
  return 0;
}

int getCurrentOpenNI2FrameNumber(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
  return colorFrame[devID].getFrameIndex();
}

//COLOR FRAMES
int getOpenNI2ColorWidth(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
   return colorFrame[devID].getWidth();
}
int getOpenNI2ColorHeight(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
   return colorFrame[devID].getHeight();
}
int getOpenNI2ColorDataSize(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
    return colorFrame[devID].getDataSize();
}
int getOpenNI2ColorChannels(int devID)
{
  return 3;
}
int getOpenNI2ColorBitsPerPixel(int devID)
{
  return 8;
}
unsigned char * getOpenNI2ColorPixels(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
   return (unsigned char *)colorFrame[devID].getData();
}


double getOpenNI2ColorFocalLength(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
    int zpd=0;
    color[devID].getProperty(XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE,&zpd);
    return (double) zpd;
}

double getOpenNI2ColorPixelSize(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
    double zpps=0.0;
    color[devID].getProperty(XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE,&zpps);

    fprintf(stderr,"Note : OpenNI2 gives us half the true pixel size ? ? \n");
    zpps*=2.0;

    return (double) zpps;
}




//DEPTH FRAMES
int getOpenNI2DepthWidth(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
   return depthFrame[devID].getWidth();
}
int getOpenNI2DepthHeight(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
   return depthFrame[devID].getHeight();
}
int getOpenNI2DepthDataSize(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
    return depthFrame[devID].getDataSize();
}
int getOpenNI2DepthChannels(int devID)
{
  return 1;
}
int getOpenNI2DepthBitsPerPixel(int devID)
{
  return 16;
}
unsigned short * getOpenNI2DepthPixels(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }

  #if MOD_NITE2
   if (lastDepthComesFromNite[devID]) { return getNite2DepthFrame(devID); }
  #endif

 return (unsigned short *) depthFrame[devID].getData();
}


double getOpenNI2DepthFocalLength(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
    int zpd=0;
    depth[devID].getProperty(XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE,&zpd);
    if (zpd==0) {
                  fprintf(stderr,"Please Note That getOpenNI2DepthFocalLength returned zero , so we return getOpenNI2ColorFocalLength\n");
                  return  getOpenNI2ColorFocalLength(devID);
                }

    return (double) zpd;
}

double getOpenNI2DepthPixelSize(int devID)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
    double zpps=0.0;
    depth[devID].getProperty(XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE,&zpps);

    fprintf(stderr,"Note : OpenNI2 gives us half the true pixel size ? ? \n");
    zpps*=2.0;
    if (zpps==0) {
                  fprintf(stderr,"Please Note That getOpenNI2DepthPixelSize returned zero , so we return getOpenNI2ColorPixelSize\n");
                  return  getOpenNI2ColorPixelSize(devID);
                 }

    return (double) zpps;
}


#if USE_CALIBRATION
int getOpenNI2ColorCalibration(int devID,struct calibration * calib)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
    memcpy((void*) calib,(void*) &calibRGB[devID],sizeof(struct calibration));
    return 1;
}

int getOpenNI2DepthCalibration(int devID,struct calibration * calib)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
    memcpy((void*) calib,(void*) &calibDepth[devID],sizeof(struct calibration));
    return 1;
}


int setOpenNI2ColorCalibration(int devID,struct calibration * calib)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
    memcpy((void*) &calibRGB[devID] , (void*) calib,sizeof(struct calibration));
    return 1;
}

int setOpenNI2DepthCalibration(int devID,struct calibration * calib)
{
  if (badDeviceID(devID,__FILE__,__LINE__)) { return 0; }
    memcpy((void*) &calibDepth[devID] , (void*) calib,sizeof(struct calibration));
    return 1;
}
#endif



#else
//Null build of this library just outputs a warning when linked
int startOpenNI2Module(unsigned int max_devs,char * settings)
{
    fprintf(stderr,"startOpenNI2Module called on a dummy build of OpenNI2Acquisition!\n");
    fprintf(stderr,"Please consider enabling #define BUILD_OPENNI2 1 on acquisition/acquisition_setup.h\n");
    return 0;
  return 1;
}
#endif
