#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "classifier.h"

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/highgui/highgui.hpp"


using namespace std;
using namespace cv;

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
