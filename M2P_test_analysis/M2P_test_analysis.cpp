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
#define UV_AVRAGE_NUMBER 20

#define KEYCODE_ESCAP 27
#define KEYCODE_SPACE 32
#define KEYCODE_LEFT 2424832
#define KEYCODE_RIGHT 2555904


using namespace std;
using namespace cv;

float maxBlobSize = 5000;
float minBlobSize = 200;
float minCircularity = 0.7f;


#define VIDEO_FILE ("C0001-converted.mp4")
#define UVLOG_FILE ("log.csv")

//#define VIDEO_FILE ("C0008.mp4")
//#define UVLOG_FILE ("log8.csv")
#define VIDEO_START_FRAME (400)

struct MOUSE_STATE {
	int event;
	int x;
	int y;
	int flags;
} mouse_state;




void initWindow() {
	cv::namedWindow(MAIN_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::resizeWindow(MAIN_WINDOW_NAME, 960, 540);
}


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
Point2f  GetUVValue(CReferenceBoard refBoard,  Point2f pt) {
	return refBoard.GetUVCoordinate( pt);
}
void DrawUVValue(Mat frame, Point2f uv, Point2f pt) {
	
	std::ostringstream uvText;
	uvText << std::setprecision(2);
	uvText << uv.x << "," << uv.y;
	putText(frame, uvText.str(), pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
	return ;
}

Point2f GetMeanValue(std::list<cv::Point2f> &uvHistory, int maxNumber)
{
	Point2f uv(0, 0);
	int count = 0;
	for (auto it = uvHistory.begin(); it != uvHistory.end() && count < maxNumber; it++, count++) {
		uv += (*it);
	}
	uv /= (float)count;
	return uv;
}
void DrawStartUV(Mat frame, Point2f pt) {
	cv::drawMarker(frame, pt, Scalar(255, 255, 0), MARKER_CROSS,40);
}

void GetExpectedPositionFromMeanValue(std::list<cv::Point2f> &uvHistory, CReferenceBoard &refBoard,  cv::Point2f &reprojectedPoint)
{
	Point2f uv = GetMeanValue(uvHistory, UV_AVRAGE_NUMBER);
	reprojectedPoint = refBoard.GetReprojectedCoordinate(uv);
}

void SaveLog( std::string path, list<LOG_INFO> uvList) {
	std::ofstream o(path, std::ofstream::trunc);
	o << std::setprecision(6);
	// output header
	o << "u,v,real_x,real_y,expected_x,expected_y" << std::endl;

	for (auto it = uvList.begin(); it != uvList.end(); it++) {
		Point2f uv = it->uv;
		cv::Point2f realPositionInPixel = it->realPositionInPixel;
		cv::Point2f expectedPositionInPixel = it->expectedPositionInPixel;
		o << uv.x << " , " << uv.y << " , ";
		o << realPositionInPixel.x << " , " << realPositionInPixel.y << " , ";
		o << expectedPositionInPixel.x << " , " << expectedPositionInPixel.y;
		o<< std::endl;
	}
	o.close();
}


int main() {
	Mat frame;

	std::list<LOG_INFO> logList;
	std::list<cv::Point2f> uvHistoryList;


	CReferenceBoard renderenceBoard;

	VideoCapture cap(VIDEO_FILE);
	//frame = GetVideoFrame(cap, frameControlFlag);
	
	cap.set(CV_CAP_PROP_POS_FRAMES, VIDEO_START_FRAME);
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

		LOG_INFO stepLog;
		frame = GetVideoFrame(cap, frameControlFlag);
		controlbar.UpdateStatus(cap);	


		if (frame.empty())
			break;
		vector<KeyPoint> corners;
		
		
		bool isFoundFlag =false;
		isFoundFlag = FindWhiteInBlackCircleGrid(blobparams, frame, corners);


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
					vector<Point2f> inputArray = { pt[hullID[1]], pt[hullID[0]], pt[hullID[3]], pt[hullID[2]] };
					renderenceBoard.UpdateCurrentTransform(inputArray);
					Point2f uv = renderenceBoard.GetUVCoordinate(Point2f((float)mouse_state.x,(float) mouse_state.y));
					DrawUVValue(frame, uv, Point2f((float)mouse_state.x, (float)mouse_state.y));
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
				renderenceBoard.UpdateCurrentTransform(inputArray);
				Point2f uv = renderenceBoard.GetUVCoordinate(pt[centerID]);
				uvHistoryList.push_back(uv);
				Point2f expectedPosition(0,0);
				GetExpectedPositionFromMeanValue(uvHistoryList, renderenceBoard,  expectedPosition);
				Point2f centerPt(pt[centerID]);
				DrawUVValue(frame, uv, centerPt);
				
				stepLog.uv = uv;
				stepLog.realPositionInPixel = centerPt;
				stepLog.expectedPositionInPixel = expectedPosition;
				logList.push_back(stepLog);

				

				if (mouse_state.flags && EVENT_FLAG_LBUTTON) {
					Point2f uv = renderenceBoard.GetUVCoordinate(Point2f(mouse_state.x, mouse_state.y));
					DrawUVValue(frame, uv, Point2f(mouse_state.x, mouse_state.y));
				}
			
				DrawStartUV(frame , expectedPosition);
			}
			else {
				cout << "..." << endl;
			}

		}
		imshow("Frame", frame);

		bool needQuit = false;
		ProcessMainLoopKeyEvent(needQuit, frameControlFlag);
		if (needQuit) 
			break;
	}

	// When everything done, release the video capture object
	cap.release();

	// Closes all the frames
	destroyAllWindows();


	SaveLog(UVLOG_FILE,logList);

	return 0;
}

void ProcessMainLoopKeyEvent(bool & needQuit, FRAME_CONTROL & control)
{
	needQuit = false;
	// Press  ESC on keyboard to exit
	int c = (int)waitKeyEx(1);
	//cout << (int)c <<endl;	
	if (c == KEYCODE_ESCAP)
	{
		needQuit = true;
		return;
	}
	else if (c == KEYCODE_SPACE) {
		if (control == FRAME_PAUSE)
			control = FRAME_PLAY;
		else
			control = FRAME_PAUSE;
	}
	else if (c == KEYCODE_LEFT) {
		control = FRAME_BACKWARD_ONE_FRAME;
	}
	else if (c == KEYCODE_RIGHT) {
		control = FRAME_FORWARD_ONE_FRAME;
	}
}
