/*
 * cvtest.cpp
 *
 *  Created on: Jan 6, 2011
 *      Author: walter
 */
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/highgui.h>
#include  <iostream>
#include <vector>
#include <sys/time.h>


int main (int argc, char** argv)
{
    timeval start;
    timeval end;

	int border = 10;
	cv::namedWindow("img");
	cv::namedWindow("mask");

    cv::Mat mask;
	cv::Mat tmpfg,tmpbg;
	cv::Mat imgMat_s = cv::imread( "frame0001.jpg", 1 );
	cv::Mat dispMat_s = cv::imread( "frame0000.jpg", 0 );
	cv::Mat imgMat = cv::Mat::zeros(imgMat_s.cols + 2*border, imgMat_s.rows + 2*border, CV_8UC1);
	cv::Mat dispMat= cv::Mat::zeros(imgMat_s.cols + 2*border, imgMat_s.rows + 2*border, CV_8UC1);
	cv::Mat fgMat = cv::imread( "screwdriver.jpg", 0);

	cv::copyMakeBorder(imgMat_s, imgMat, border, border, border, border, cv::BORDER_CONSTANT, 0 );
	cv::copyMakeBorder(dispMat_s, dispMat, border, border, border, border, cv::BORDER_CONSTANT, 1 );
	mask = cv::Mat::zeros(imgMat.size(), CV_8UC1);

	cv::threshold(dispMat,dispMat,0,cv::GC_FGD,cv::THRESH_BINARY_INV);
	// correct for the shift of the disp-image wrt the rgb image, correct non-filled pixels
	cv::Rect rup,rside;
	rup.x = 0; rup.y=0; rup.width = dispMat.cols; rup.height = 40+border;
	rside.x = dispMat.cols - 60-border; rside.y=0; rside.width = 60+border; rside.height = dispMat.rows;
	cv::rectangle(dispMat,rup,0,-1); 	cv::rectangle(dispMat,rside,0,-1);

	// for every cluster
    gettimeofday(&start, NULL);

	std::vector<cv::Point2i> pts;
	for (int i = 0; i < fgMat.rows; ++i)
		for (int j = 0; j < fgMat.cols; ++j)
		{
			if (fgMat.at<uchar>(i,j) == cv::GC_FGD)
				pts.push_back(cv::Point2i(j+border,i+border));
		}

	dispMat.copyTo(mask);

	for (unsigned int i = 0; i < pts.size(); ++i)
		mask.at<uchar>(pts[i]) = cv::GC_FGD;

	cv::Mat seedMat;
	mask.copyTo(seedMat);

	cv::imshow("seedMat", imgMat_s);
	cv::waitKey();

	cv::Rect rect;
	cv::floodFill(seedMat,pts[0],cv::GC_FGD,&rect,cv::Scalar(0),cv::Scalar(0));  // connect GC_FGD + GC_PR_FGD
	cv::imshow("seedMat", imgMat_s);
	cv::waitKey();
	cv::Rect rect_b(rect);	rect_b.x -= border; rect_b.y -= border;	rect_b.width += 2*border;	rect_b.height += 2*border;

	for (unsigned int i = 0; i < pts.size(); ++i)
		mask.at<uchar>(pts[i]) = cv::GC_FGD;

	cv::Mat mroi = mask(rect);
	cv::Mat image_small_roi = imgMat(rect_b);
	cv::Mat mask_small = cv::Mat::zeros(rect_b.height, rect_b.width, CV_8UC1);
	cv::Mat mask_roi = mask_small(cv::Rect(border,border,rect.width,rect.height));
	cv::Mat seed_roi = mask(rect);

	seed_roi.copyTo(mask_roi);
	cv::medianBlur(mask_roi,mask_roi,5);
    cv::Mat ctmp = cv::Mat::zeros(mask_small.size(), CV_8UC1);
    mask_small.copyTo(ctmp);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(ctmp, contours, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    mask_small.copyTo(ctmp);
	cv::drawContours(mask_small,contours,-1,cv::GC_PR_BGD,5);   // GC_PR_BGD  broad boada
	cv::drawContours(mask_small,contours,-1,cv::GC_PR_BGD,-1);  // fill interiour, as wrong marked GC_BGD mess up the results
	ctmp.copyTo(mask_small,ctmp);

	cv::threshold(ctmp,ctmp,0,1,cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point> > contours_GC_FGD;
    cv::findContours(ctmp, contours_GC_FGD, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	cv::drawContours(mask_small,contours_GC_FGD,-1,cv::GC_PR_BGD,2);   // shrink the GC_FGD, kinect tends to broaden real results

    gettimeofday(&end, NULL);
    std::cout << (end.tv_usec - start.tv_usec) / 1000 << " ms\n";

    gettimeofday(&start, NULL);
    	cv::grabCut(image_small_roi, mask_small, rect, tmpbg, tmpfg, 2, cv::GC_INIT_WITH_MASK);
    	//cv::grabCut(image_small_roi, mask_small, rect, tmpbg, tmpfg, 1, cv::GC_EVAL);
    gettimeofday(&end, NULL);
    std::cout << (end.tv_usec - start.tv_usec) / 1000 << " ms\n";

    cv::Mat binMask;
    binMask.create( mask_small.size(), CV_8UC1 );
    binMask = mask_small & 1;
	cv::Mat res;

//	mask_small.copyTo(res, binMask);
	image_small_roi.copyTo(res, binMask);

	cv::findContours(binMask,contours, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    std::vector<std::vector<cv::Point> > contours_p;
    contours_p.resize(1);
    cv::approxPolyDP(cv::Mat(contours[0]), contours_p[0], 1, true);

    cv::drawContours( res, contours_p, -1, cv::Scalar(255,0,255),1);//, CV_AA, hierarchy, std::abs(_levels) );


	cv::Rect aabb2d = cv::boundingRect(cv::Mat(contours[0]));
	cv::RotatedRect oobb2d = cv::minAreaRect(cv::Mat(contours[0]));
    cv::Point2f vtx[4];
    oobb2d.points(vtx);

    //cv::drawContours(res, contours, -1, cv::Scalar(0,0,255),1);

    for(int i = 0; i < 4; i++ )
        cv::line(res, vtx[i], vtx[(i+1)%4], cv::Scalar(0, 255, 0), 1, CV_AA);

    cv::rectangle(res,aabb2d,cv::Scalar(255,0,0),1);

	cv::imshow("img", res);
	cv::imshow("mask", mask_small*96);
	cv::waitKey();

	return (0);
}
