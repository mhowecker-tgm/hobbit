#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "classifier.h"
#include "tools.h"
#include "fall_detection.h"

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/highgui/highgui.hpp"


using namespace std;
using namespace cv;


unsigned int lowBorderY=480;


int visualizeTopCam(unsigned char * colorFrame,unsigned char * segmentedRGB,unsigned int colorWidth , unsigned int colorHeight ,
                    unsigned int consultHobbitMap , unsigned int frameTimestamp)
{
     char rectVal[256]={0};
     int fontUsed=FONT_HERSHEY_SIMPLEX; //FONT_HERSHEY_SCRIPT_SIMPLEX;
     cv::Mat bgrMat,rgbMat;

     if (segmentedRGB!=0)  { rgbMat = cv::Mat(colorHeight,colorWidth,CV_8UC3,segmentedRGB,3*colorWidth); } else
                           { rgbMat = cv::Mat(colorHeight,colorWidth,CV_8UC3,colorFrame,3*colorWidth); }

	    cv::cvtColor(rgbMat,bgrMat, CV_RGB2BGR);// opencv expects the image in BGR format

        RNG rng(12345);

        Scalar color = Scalar ( rng.uniform(0,255) , rng.uniform(0,255) , rng.uniform(0,255)  );
        Scalar colorEmergency = Scalar ( 0 , 0 , 255  );
        Point pt1;
        Point pt2;

        //Draw Low Border
        pt1.x=0; pt2.x=lastState.topX1;
        pt1.y=lowBorderY;  pt2.y=lowBorderY;
        line( bgrMat,pt1 , pt2 , color, 1, 1 );

        pt1.x=lastState.topX1+lastState.topWidth; pt2.x=colorWidth;
        line( bgrMat,pt1 , pt2 , color, 1, 1 );
        //------------------


        Point centerPt; centerPt.x=colorWidth/2;       centerPt.y=colorHeight/2;
        pt1.x=lastState.topX1;                     pt1.y=lastState.topY1;
        pt2.x=lastState.topX1+lastState.topWidth;  pt2.y=lastState.topY1+lastState.topHeight;

        rectangle(bgrMat ,  pt1 , pt2 , color , 2, 8 , 0);


        if (fallDetectionContext.headLookingDirection==HEAD_LOOKING_DOWN)
        {
         pt2.x=lastState.topX1+lastState.topWidth;  pt2.y=lastState.topY1-overHeight;
         rectangle(bgrMat ,  pt1 , pt2 , color , 1, 8 , 0);
         pt2.x = pt1.x+15;
         pt2.y += 24; snprintf(rectVal,123,"Over : %u ",lastState.scoreOverTop);
         putText(bgrMat , rectVal, pt2 , fontUsed , 0.5 , color , 2 , 8 );
         pt2.y += 24; snprintf(rectVal,123,"Holes : %0.2f %%",lastState.holesPercentOverTop);
         putText(bgrMat , rectVal, pt2 , fontUsed , 0.5 , color , 2 , 8 );


         pt2.y += 24; snprintf(rectVal,123,"Bord : %u ",lowBorderY);
         putText(bgrMat , rectVal, pt2 , fontUsed , 0.5 , color , 2 , 8 );
        }

        unsigned int tempColorR=255 , tempColorG=0 , tempColorB=0;
        if (lastState.objectTemperature<30) { tempColorR=0 , tempColorG=0 , tempColorB=255; } else
        if (lastState.objectTemperature>40) { tempColorR=255 , tempColorG=0 , tempColorB=0; } else
                                          { tempColorR=(unsigned int) 125+( 40-lastState.objectTemperature/10 ) * 125 , tempColorG=0 , tempColorB=0; }


        Scalar tempRColor = Scalar ( 0 , 0 , 255 );
        Scalar tempColor = Scalar ( tempColorB , tempColorG , tempColorR );
        circle(bgrMat,  centerPt , 25 , tempColor , 4, 8 , 0);

        if ( temperatureSensorSensesHuman( lastState.objectTemperature ,  lastState.timestampTemperature , frameTimestamp) )
        {
           circle(bgrMat,  centerPt , 10 , tempRColor , 4, 8 , 0);
        }


        Point txtPosition;  txtPosition.x = pt1.x+15; txtPosition.y = pt1.y+20;


        if ( emergencyDetected )
        { // EMERGENCY GRAPHICS HERE ----------------------------------------------------------------------
          putText(bgrMat , "Emergency Detected ..! " , txtPosition , fontUsed , 0.7 , color , 2 , 8 );
          Point ptIn1; ptIn1.x=lastState.topX1;                    ptIn1.y=lastState.topY1;
          Point ptIn2; ptIn2.x=lastState.topX1+lastState.topWidth; ptIn2.y=lastState.topY1+lastState.topHeight;

          unsigned int i=0;
          for (i=0; i<3; i++)
          {
            ptIn1.x-=10; ptIn1.y-=10;
            ptIn2.x+=10; ptIn2.y+=10;
            rectangle(bgrMat ,  ptIn1 , ptIn2 , colorEmergency , 2, 8 , 0);
          }

          Point ul; ul.x=0;          ul.y=0;
          Point ur; ur.x=colorWidth; ur.y=0;
          Point dl; dl.x=0;          dl.y=colorHeight;
          Point dr; dr.x=colorWidth; dr.y=colorHeight;
          line(bgrMat,ul,ptIn1 , colorEmergency , 2 , 8 , 0);
          line(bgrMat,dr,ptIn2 , colorEmergency , 2 , 8 , 0);

          ptIn1.x+=lastState.topWidth+60;
          ptIn2.x-=lastState.topWidth+60;
          line(bgrMat,ur,ptIn1 , colorEmergency , 2 , 8 , 0);
          line(bgrMat,dl,ptIn2 , colorEmergency , 2 , 8 , 0);
        } // EMERGENCY GRAPHICS HERE ----------------------------------------------------------------------
          else
        //======================================================================================================

        if (fallDetectionContext.headLookingDirection==HEAD_MOVING_FAST)
             { putText(bgrMat , "Head Moving" , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
            else
        if (fallDetectionContext.headLookingDirection==HEAD_LOOKING_CENTER)
             { putText(bgrMat , "Horizontal - Live Fall" , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
            else
        if (fallDetectionContext.headLookingDirection==HEAD_LOOKING_LEFT)
             { putText(bgrMat , "Left / Disabled" , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
            else
        if (fallDetectionContext.headLookingDirection==HEAD_LOOKING_RIGHT)
             { putText(bgrMat , "Right / Disabled" , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
            else
        if (fallDetectionContext.headLookingDirection==HEAD_LOOKING_LITTLE_DOWN)
             { putText(bgrMat , "Diagonal - Live Fall" , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
            else
        if (segmentedRGB==0)
             { putText(bgrMat , "Low Temp / Power Save" , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
            else
        if (fallDetectionContext.headLookingDirection!=HEAD_LOOKING_DOWN)
             { putText(bgrMat , "Head Not Looking Down" , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
            else
             { putText(bgrMat , "Scanning for fallen user.." , txtPosition , fontUsed , 0.7 , color , 2 , 8 ); }
        //======================================================================================================
        txtPosition.y += 24; snprintf(rectVal,123,"Top : %u / Bot : %u ",lastState.scoreTop,lastState.scoreBase);
        putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , color , 2 , 8 );
        txtPosition.y += 24; snprintf(rectVal,123,"Temperature : %0.2f C",lastState.objectTemperature);
        putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , color , 2 , 8 );


        txtPosition.y += 50;


        if (consultHobbitMap)
        {
          txtPosition.y += 24; snprintf(rectVal,123,"Obstacle map hits : %u ",lastState.totalMapObstacleHits);
          putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , color , 2 , 8 );
        }

       if (userIsRecent(&fallDetectionContext,frameTimestamp))
       {
        txtPosition.y += 24; snprintf(rectVal,123,"Sk Movem. : %u",fallDetectionContext.skeletonVelocity);
        putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , color , 2 , 8 );

        unsigned int i=0;
        for (i=0; i<fallDetectionContext.numberOfJoints; i++)
         {
           Scalar jointColor = Scalar ( 0 , 255 , 0 );
           if ( ( fallDetectionContext.currentJoint2D[i].x!=0.0 ) || ( fallDetectionContext.currentJoint2D[i].y!=0.0) )
           {
               centerPt.x=fallDetectionContext.currentJoint2D[i].x;       centerPt.y=fallDetectionContext.currentJoint2D[i].y;

               if (fallDetectionContext.currentJoint2D[i].y > lowBorderY)
               {
                   jointColor=tempRColor;
               }

               circle(bgrMat,  centerPt , 15 , jointColor , -4, 8 , 0);
           }
         }
       }




       if (lastState.badContrastTop)
       {
        txtPosition.y += 24; snprintf(rectVal,123,"BAD Contrast!");
        putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , color , 2 , 8 );

       }

	    cv::imshow("emergency_detector visualization",bgrMat);
	    cv::waitKey(1);
}










int visualizeBaseCam(unsigned char * segmentedRGB,unsigned int colorWidth , unsigned int colorHeight)
{
 cv::Mat bgrMat,rgbMat;
 rgbMat = cv::Mat(colorHeight,colorWidth,CV_8UC3,segmentedRGB,3*colorWidth);

 cv::cvtColor(rgbMat,bgrMat, CV_RGB2BGR);// opencv expects the image in BGR format

 Point ptIn1; ptIn1.x=lastState.baseX1;                          ptIn1.y=lastState.baseY1;
 Point ptIn2; ptIn2.x=lastState.baseX1+lastState.baseWidth;      ptIn2.y=lastState.baseY1+lastState.baseHeight;
 Scalar colorEmergency = Scalar ( 255 , 0 , 0 );

 rectangle(bgrMat ,  ptIn1 , ptIn2 , colorEmergency , 2, 8 , 0);


  char rectVal[256]={0};
        int fontUsed=FONT_HERSHEY_SIMPLEX; //FONT_HERSHEY_SCRIPT_SIMPLEX;
        Point txtPosition;  txtPosition.x = ptIn1.x+15; txtPosition.y = ptIn1.y;


        txtPosition.y += 24; snprintf(rectVal,123,"Base Score : %u",lastState.scoreBase);
        putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , colorEmergency , 2 , 8 );

        txtPosition.y += 24; snprintf(rectVal,123,"Base Holes %0.2f %%",lastState.holesPercentBase);
        putText(bgrMat , rectVal, txtPosition , fontUsed , 0.7 , colorEmergency , 2 , 8 );


	    cv::imshow("emergency_detector base visualization",bgrMat);
	    cv::waitKey(1);
}
