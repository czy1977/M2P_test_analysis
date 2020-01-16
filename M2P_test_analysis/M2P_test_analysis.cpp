// M2P_test_analysis.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include "markerDetect.h"
#include "OCVInterface.h"
#include "CBlobDetectorController.h"
#include "COpenCVVideoControlBar.h"
#define MAIN_WINDOW_NAME "Frame"

using namespace std;
using namespace cv;

float maxBlobSize = 10000;
float minBlobSize = 200;
float minCircularity = 0.7;
#define KEYCODE_ESCAP 27
#define KEYCODE_SPACE 32
#define KEYCODE_LEFT 2424832
#define KEYCODE_RIGHT 2555904

void initWindow() {
	cv::namedWindow(MAIN_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::resizeWindow(MAIN_WINDOW_NAME, 960, 540);


}

enum FRAME_CONTROL {
	FRAME_PLAY,
	FRAME_PAUSE,
	FRAME_FORWARD_ONE_FRAME,
	FRAME_BACKWARD_ONE_FRAME,
	FRAME_ONE_FRAME_PAUSE
} ;
FRAME_CONTROL frameControlFlag;

Mat GetVideoFrame(VideoCapture & capf, FRAME_CONTROL & control) {
	Mat frame;
	if (control == FRAME_PLAY) {	
		
	}
	else if (control == FRAME_PAUSE || control == FRAME_ONE_FRAME_PAUSE) {
		double frame_index = capf.get(CV_CAP_PROP_POS_FRAMES);
		Mat frame2;
		capf.retrieve(frame2);
		frame = frame2.clone();
		return frame;
		//capf.set(CV_CAP_PROP_POS_FRAMES, frame_index);
	}
	else if (control == FRAME_FORWARD_ONE_FRAME) {
		double frame_index = capf.get(CV_CAP_PROP_POS_FRAMES);
		control = FRAME_ONE_FRAME_PAUSE;

	}
	else if (control == FRAME_BACKWARD_ONE_FRAME) {
		double frame_index = capf.get(CV_CAP_PROP_POS_FRAMES);
		capf.set(CV_CAP_PROP_POS_FRAMES, frame_index - 2.0);
		control = FRAME_ONE_FRAME_PAUSE;
	}
	
	capf >> frame;
	return frame;
}

int main() {


	VideoCapture cap("D:/dev/python/jupyter/2020-01-09_latency/C0009-converted.mp4");
	cap.set(CV_CAP_PROP_POS_FRAMES, 1000);
	frameControlFlag = FRAME_PLAY;
	if (!cap.isOpened()) {
		
		cout << "Error opening video stream or file" << endl;
		return -1;
	}

	initWindow();

	CBlobDetectorController blob1;
	std::shared_ptr<cv::SimpleBlobDetector::Params> blobparams(new cv::SimpleBlobDetector::Params);
	blob1.open(blobparams,"bigmarker");

	COpenCVVideoControlBar controlbar(MAIN_WINDOW_NAME);
	
	
	while (1) {

		Mat frame;

		frame = GetVideoFrame(cap, frameControlFlag);

		controlbar.UpdateStatus(cap);	


		if (frame.empty())
			break;
		vector<KeyPoint> corners;
		bool isFoundFlag = FindWhiteInBlackCircleGrid(frame, maxBlobSize, minBlobSize, minCircularity, corners);
		if (corners.size() == 4) {
			vector<int> hullID;
			vector<Point2f> pt;
			KeyPoint::convert(corners, pt);
			convexHull(pt, hullID, false, false);
			if (hullID.size() == 4) {
				putText(frame, "1", pt[hullID[0]], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
				putText(frame, "2", pt[hullID[1]], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
				putText(frame, "3", pt[hullID[2]], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
				putText(frame, "4", pt[hullID[3]], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
				circle(frame, Point2f(corners[0].pt.x, corners[0].pt.y), 3, Scalar(255, 0, 0), 3);
				circle(frame, Point2f(corners[1].pt.x, corners[1].pt.y), 3, Scalar(255, 0, 0), 3);
				circle(frame, Point2f(corners[2].pt.x, corners[2].pt.y), 3, Scalar(255, 0, 0), 3);
				circle(frame, Point2f(corners[3].pt.x, corners[3].pt.y), 3, Scalar(255, 0, 0), 3);
			}
		}
		
		if (corners.size() == 5) {
			vector<int> hullID;
			vector<Point2f> pt;
			KeyPoint::convert(corners, pt);
			convexHull(pt, hullID, false, false);
			//cout << "hull: " << hullID.size() << endl;

			if (hullID.size() == 4) {
				putText(frame, "1", pt[hullID[0]], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
				putText(frame, "2", pt[hullID[1]], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
				putText(frame, "3", pt[hullID[2]], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
				putText(frame, "4", pt[hullID[3]], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
				int centerID = 10 - hullID[0] - hullID[1]- hullID[2]- hullID[3];
				circle(frame, pt[hullID[0]], 3, Scalar(255, 0, 0), 3);
				circle(frame, pt[hullID[1]], 3, Scalar(255, 0, 0), 3);
				circle(frame, pt[hullID[2]], 3, Scalar(255, 0, 0), 3);
				circle(frame, pt[hullID[3]], 3, Scalar(255, 0, 0), 3);
				circle(frame, pt[centerID], 3, Scalar(0, 0, 255), 3);
			}
			else {
				cout << "..." << endl;
			}

		}

		imshow("Frame", frame);

		// Press  ESC on keyboard to exit
		int c = (int)waitKeyEx(1);
		//cout << (int)c <<endl;	
		if (c == KEYCODE_ESCAP)
			break;

		else if (c== KEYCODE_SPACE) {
			if(frameControlFlag == FRAME_PAUSE)
				frameControlFlag = FRAME_PLAY;
			else
				frameControlFlag = FRAME_PAUSE;
		}
		else if (c == KEYCODE_LEFT) {
				frameControlFlag = FRAME_BACKWARD_ONE_FRAME;
		}
		else if (c == KEYCODE_RIGHT) {
			frameControlFlag = FRAME_FORWARD_ONE_FRAME;
		}
	}

	// When everything done, release the video capture object
	cap.release();

	// Closes all the frames
	destroyAllWindows();

	return 0;
}
