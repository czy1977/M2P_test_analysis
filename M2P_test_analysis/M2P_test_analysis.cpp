// M2P_test_analysis.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include "markerDetect.h"
#include "OCVInterface.h"
#include "CBlobDetectorController.h"
#include "COpenCVVideoControlBar.h"
#include "CReferenceBoard.h"
#include "M2P_test_analysis.h"

#define MAIN_WINDOW_NAME "Frame"

using namespace std;
using namespace cv;

float maxBlobSize = 10000;
float minBlobSize = 100;
float minCircularity = 0.7;

#define KEYCODE_ESCAP 27
#define KEYCODE_SPACE 32
#define KEYCODE_LEFT 2424832
#define KEYCODE_RIGHT 2555904

struct MOUSE_STATE {
	int event;
	int x;
	int y;
	int flags;
} mouse_state;

cv::Mat4f boardTrans;

void initWindow() {
	cv::namedWindow(MAIN_WINDOW_NAME, cv::WINDOW_NORMAL);
	//cv::resizeWindow(MAIN_WINDOW_NAME, 960, 540);


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
Point2f DrawUVValue(Mat frame, vector<Point2f> inputArray,Point2f pt) {
	CReferenceBoard refBoard;
	Point2f uv = refBoard.GetUVCoordinate(inputArray,pt);
	
	std::ostringstream uvText;
	uvText << std::setprecision(2);
	uvText << uv.x << "," << uv.y;

	putText(frame, uvText.str(), pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);

	return uv;
}
#define UV_AVRAGE_NUMBER 20
Point2f GetMeanValue(std::list<cv::Point2f> &uvHistory, int number)
{
	Point2f uv(0, 0);
	int count = 0;
	for (auto it = uvHistory.begin(); it != uvHistory.end() && count < number; it++, count++) {
		uv += (*it);
	}
	uv /= (float)count;
	return uv;
}
void DrawStartUV(Mat frame,list<Point2f> uvHistory, vector<Point2f> inputArray) {
	CReferenceBoard refBoard;
	
	if (uvHistory.size() < UV_AVRAGE_NUMBER)
	{
		return;
	}
	Point2f uv = GetMeanValue(uvHistory, UV_AVRAGE_NUMBER);
	Matx33f homography = refBoard.GetTrans(inputArray);
	Point2f reprojectedPoint = refBoard.GetReprojectedCoordinate(homography,uv);

	cv::drawMarker(frame, reprojectedPoint, Scalar(255, 255, 0), MARKER_CROSS,40);
}

void SaveUVList( std::string path, list<Point2f> uvList) {
	std::ofstream o(path, std::ofstream::trunc);
	o << std::setprecision(6);
	for (auto it = uvList.begin(); it != uvList.end(); it++) {
		Point2f uv = *it;
		o << uv.x << " , " << uv.y << std::endl;
	}
	o.close();
}

#define VIDEO_FILE ("C0009-converted.mp4")
#define UVLOG_FILE ("uvList.csv")
int main() {
	Mat frame;
	std::list<Point2f> uvList;

	VideoCapture cap(VIDEO_FILE);
	//frame = GetVideoFrame(cap, frameControlFlag);
	
	cap.set(CV_CAP_PROP_POS_FRAMES, 1500);
	frameControlFlag = FRAME_PLAY;
	if (!cap.isOpened()) {
		
		cout << "Error opening video stream or file" << endl;
		return -1;
	}

	initWindow();

	CBlobDetectorController blobConfigBar;
	std::shared_ptr<cv::SimpleBlobDetector::Params> blobparams(new cv::SimpleBlobDetector::Params);
	
	InitBlobParams(blobparams, minBlobSize, maxBlobSize, minCircularity);
	blobConfigBar.open(blobparams,"bigmarker");


	COpenCVVideoControlBar controlbar(MAIN_WINDOW_NAME);

	cv::setMouseCallback(MAIN_WINDOW_NAME,[](int event, int x, int y, int flags, void *userdata)-> void {
		mouse_state.event = event;
		mouse_state.x = x;
		mouse_state.y = y;
		mouse_state.flags = flags;
	});
	
	
	while (1) {



		frame = GetVideoFrame(cap, frameControlFlag);

		controlbar.UpdateStatus(cap);	


		if (frame.empty())
			break;
		vector<KeyPoint> corners;
		
		
		bool isFoundFlag = FindWhiteInBlackCircleGrid(blobparams,frame, corners);


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

				if (mouse_state.flags && EVENT_FLAG_LBUTTON) {
					DrawUVValue(frame, { pt[hullID[1]], pt[hullID[0]], pt[hullID[3]], pt[hullID[2]] }, Point2f(mouse_state.x, mouse_state.y));
				}
				
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


				vector<Point2f> inputArray = { pt[hullID[1]], pt[hullID[0]], pt[hullID[3]], pt[hullID[2]] };
				Point2f uv = DrawUVValue(frame, inputArray, pt[centerID]);
				uvList.push_back(uv);

				if (mouse_state.flags && EVENT_FLAG_LBUTTON) {
					DrawUVValue(frame, inputArray, Point2f(mouse_state.x, mouse_state.y));
				}
				
				DrawStartUV(frame,uvList, inputArray);
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


	SaveUVList(UVLOG_FILE,uvList);

	return 0;
}
