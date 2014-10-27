#include "extAcquisition.h"

#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "OpenNI2Acquisition.h"


int useRGBDAcqusition=0;
volatile static bool first = true;

int devID=0;

unsigned int width = 640;
unsigned int height = 480;

unsigned int draw_out = 0;
unsigned int counter = 0;



unsigned int getTicks()
{
 return cvGetTickCount();
}

void switchDrawOutTo(unsigned int newVal)
{
   if ( (draw_out==1) && (newVal==0) )
   {
  	 cv::destroyWindow("Depth");
   	 cv::destroyWindow("RGB");

   	 cv::destroyAllWindows();
     cv::waitKey(1);

	 cv::waitKey(1);
   }
   draw_out = newVal;
}



int doDrawOutFrame( unsigned char * rgbFrame , unsigned int rgbWidth , unsigned int rgbHeight ,
                     unsigned short * depthFrame , unsigned int depthWidth , unsigned int depthHeight )
{
  if (!draw_out)  { return 0; }
  //-----------------------------------------------------------------------------------------------------------------------
  cv::Mat wrappedColor(rgbHeight,rgbWidth, CV_8UC3,  rgbFrame , rgbWidth*3); // does not copy
  cv::Mat rgbTmp = wrappedColor.clone();
  //Take care of drawing stuff as visual output
  cv::Mat bgrMat,rgbMat(rgbHeight,rgbWidth,CV_8UC3,rgbTmp.data,3*rgbWidth);
  cv::cvtColor(rgbMat,bgrMat, CV_RGB2BGR);// opencv expects the image in BGR format


  //-----------------------------------------------------------------------------------------------------------------------
  cv::Mat wrappedDepth(depthHeight,depthWidth, CV_16UC1 ,depthFrame, depthWidth*2); // does not copy

  cv::Mat depthNorm;
  cv::normalize(wrappedDepth,depthNorm,0,255,CV_MINMAX,CV_8UC1);

  //After we have our bgr Frame ready and we added the FPS text , lets show it!
  cv::imshow("Depth",depthNorm);
  cv::imshow("RGB",bgrMat);

  return 1;
}


int doDrawOut(  )
{
   return doDrawOutFrame(getOpenNI2ColorPixels(devID),getOpenNI2ColorWidth(devID),getOpenNI2ColorHeight(devID),
                          getOpenNI2DepthPixels(devID),getOpenNI2DepthWidth(devID),getOpenNI2DepthHeight(devID)  );
}


int getKeyPressed()
{
   if  (draw_out==1) { return cv::waitKey(3)&0x000000FF; }
   return ' '; /*No Windows -> No keys :P */
}


void externalAcquisitionCallback()
{
   snapOpenNI2Frames(devID); // <-- this also passes the new frames to skeleton/face detection!
   first=0;
}


int  acquistionStartUp(char * moduleName , unsigned int devUsed , char * from , unsigned int width , unsigned int height , unsigned int fps)
{
     if (!startOpenNI2Module(10,(char*) 0) ) { ROS_ERROR("Could not start OpenNI2 Module"); return 0; }
     devID=devUsed;

     ROS_INFO("Trying to open an OpenNI2 Device");
     if (!createOpenNI2Device(devID,from,width,height,fps)) { ROS_ERROR("Could not start OpenNI2 Device");  return 0; }

     ROS_INFO("Trying to snap the first frame out of an OpenNI2 Device");
     if (!snapOpenNI2Frames(devID)) { ROS_ERROR("Could not snap a first frame from OpenNI2 Device");  return 0; }
     if (!snapOpenNI2Frames(devID)) { ROS_ERROR("Could not snap a first frame from OpenNI2 Device");  return 0; }
     if (!snapOpenNI2Frames(devID)) { ROS_ERROR("Could not snap a first frame from OpenNI2 Device");  return 0; }
     if (!snapOpenNI2Frames(devID)) { ROS_ERROR("Could not snap a first frame from OpenNI2 Device");  return 0; }
     ROS_INFO("Survived snapping the first frame out of OpenNI2 Device");

    first=1; //This is a "first frame" for the new mode!
    useRGBDAcqusition=1; //We are now using RGBDAcquisition
    return 1;
}
