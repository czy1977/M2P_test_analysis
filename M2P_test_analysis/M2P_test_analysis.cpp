// M2P_test_analysis.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <memory>
#include "opencv2/opencv.hpp"
#include <iostream>
#include "markerDetect.h"
#include "OCVInterface.h"
#include "CBlobDetectorController.h"
#include "COpenCVVideoControlBar.h"
#include "CReferenceBoard.h"
#include "M2P_test_analysis.h"
#include "MarkerDetectInROI.h"

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
#define UVLOG_FILE ("log1.csv")

//#define VIDEO_FILE ("video/C0017.mp4")
//#define UVLOG_FILE ("M2P_test_analysis_python/logs/log_temp.csv")


#define VIDEO_START_FRAME (200)
#define LOG_FORMAT_VERSION 1 



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

Point2f  GetUVValue(CReferenceBoard & refBoard,  Point2f pt) {
	return refBoard.GetUVCoordinate( pt);
}
void DrawUVValue(Mat frame, Point2f uv, Point2f pt) {
	
	std::ostringstream uvText;
	uvText << std::setprecision(2);
	uvText << uv.x << "," << uv.y;
	std::string text = uvText.str();
	cv::putText(frame, text, pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
}
void DrawStartUV(Mat  frame, Point2f pt) {
	cv::drawMarker(frame, pt, Scalar(255, 255, 0), MARKER_CROSS, 40);
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


void GetExpectedPositionFromMeanValue(std::list<cv::Point2f> &uvHistory, CReferenceBoard &refBoard,  cv::Point2f &reprojectedPoint)
{
	Point2f uv = GetMeanValue(uvHistory, UV_AVRAGE_NUMBER);
	reprojectedPoint = refBoard.GetReprojectedCoordinate(uv);
}


void SaveLog( std::string path, list<LOG_INFO> & logList) {
	std::ofstream o(path, std::ofstream::trunc);
	o << std::setprecision(6);

#if LOG_FORMAT_VERSION==1
	// output header
	o << "frame_id,u,v,real_x,real_y,expected_x,expected_y" << std::endl;

	for (auto it = logList.begin(); it != logList.end(); it++) {
		int id = it->frameID;
		Point2f uv = it->uv;
		cv::Point2f realPositionInPixel = it->realPositionInPixel;
		cv::Point2f expectedPositionInPixel = it->expectedPositionInPixel;
		if (isnan(uv.x))
			continue;
		o << id << " , ";
		o << uv.x << " , " << uv.y << " , ";
		o << realPositionInPixel.x << " , " << realPositionInPixel.y << " , ";
		o << expectedPositionInPixel.x << " , " << expectedPositionInPixel.y;
		o << std::endl;
}
#endif // LOG_FORMAT_VERSION==1

#if LOG_FORMAT_VERSION==2
	// output header
	o << "frame_id,u,v,real_x,real_y,expected_x,expected_y" << std::endl;

	for (auto it = logList.begin(); it != logList.end(); it++) {
		Point2f uv = it->uv;
		cv::Point2f realPositionInPixel = it->realPositionInPixel;
		cv::Point2f expectedPositionInPixel = it->expectedPositionInPixel;
		o << id << " , ";
		o << uv.x << " , " << uv.y << " , ";
		o << realPositionInPixel.x << " , " << realPositionInPixel.y << " , ";
		o << expectedPositionInPixel.x << " , " << expectedPositionInPixel.y;

	}
#endif // LOG_FORMAT_VERSION==2

	o.close();
}


void PushLogEmpty(list<LOG_INFO> & logList, int frameID) {
	LOG_INFO log;
	log.frameID = frameID;
	log.uv.x= log.uv.y= NAN;
	log.realPositionInPixel.x = log.realPositionInPixel.y = NAN;
	log.expectedPositionInPixel.x = log.expectedPositionInPixel.y = NAN;
	logList.push_back(log);
}
void PushLog(list<LOG_INFO> & logList, int frameID, const cv::Point2f & expectedPosition) {
	LOG_INFO log;
	log.frameID = frameID;
	log.uv.x = log.uv.y = NAN;
	log.realPositionInPixel.x = log.realPositionInPixel.y = NAN;
	log.expectedPositionInPixel=expectedPosition;
	logList.push_back(log);
}
void PushLog(list<LOG_INFO> & logList, int frameID,const cv::Point2f & uv, const cv::Point2f & realPositionInPixel, const cv::Point2f & expectedPosition ) {
	LOG_INFO log;
	log.frameID = frameID;
	log.uv = uv;
	log.realPositionInPixel = realPositionInPixel;
	log.expectedPositionInPixel = expectedPosition;
	logList.push_back(log);
}

int main() {
	Mat frame;
	list<LOG_INFO> reportLogList;
	std::list<cv::Point2f> uvHistoryList;
	MarkerDetectInROI *mdROI = new MarkerDetectInROI;

	CReferenceBoard renderenceBoard;

	VideoCapture cap(VIDEO_FILE);
	
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
	mdROI->InitBlobParams(blobparams, minBlobSize, maxBlobSize, minCircularity);
	blobConfigBar.open(blobparams,"bigmarker");


	COpenCVVideoControlBar controlbar(MAIN_WINDOW_NAME);

	cv::setMouseCallback(MAIN_WINDOW_NAME,[](int event, int x, int y, int flags, void *userdata)-> void {
		mouse_state.event = event;
		mouse_state.x = x;
		mouse_state.y = y;
		mouse_state.flags = flags;
	});
	bool needQuit = false;
	
	while (!needQuit) {

		frame = GetVideoFrame(cap, frameControlFlag);
		controlbar.UpdateStatus(cap);	

	
		if (frame.empty()) {
			
			cout << "frame empty"<<endl;
			break;
		}
			
		vector<KeyPoint> corners;
		
		
		bool isFoundFlag =false;
		//isFoundFlag = FindWhiteInBlackCircleGrid(blobparams, frame, corners);
		int roiSize = 80;
		int marginSize = 20;
		isFoundFlag = mdROI->FindMarkers(blobparams, blobparams, frame, roiSize, marginSize);


		if (mdROI->cornerNum == 4) {
			putText(frame, "1", mdROI->corners[0], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
			putText(frame, "2", mdROI->corners[1], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
			putText(frame, "3", mdROI->corners[2], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
			putText(frame, "4", mdROI->corners[3], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
			circle(frame, mdROI->corners[0], 3, Scalar(255, 0, 0), 3);
			circle(frame, mdROI->corners[1], 3, Scalar(255, 0, 0), 3);
			circle(frame, mdROI->corners[2], 3, Scalar(255, 0, 0), 3);
			circle(frame, mdROI->corners[3], 3, Scalar(255, 0, 0), 3);

			vector<Point2f> inputArray = { mdROI->corners[1], mdROI->corners[0], mdROI->corners[3], mdROI->corners[2] };
			renderenceBoard.UpdateCurrentTransform(inputArray);
			Point2f expectedPosition(0, 0);
			GetExpectedPositionFromMeanValue(uvHistoryList, renderenceBoard, expectedPosition);
			
			if (mouse_state.flags && EVENT_FLAG_LBUTTON) {
				vector<Point2f> inputArray = { mdROI->corners[1], mdROI->corners[0], mdROI->corners[3], mdROI->corners[2] };
				renderenceBoard.UpdateCurrentTransform(inputArray);
				Point2f uv = renderenceBoard.GetUVCoordinate(Point2f((float)mouse_state.x,(float) mouse_state.y));
				DrawUVValue(frame, uv, Point2f((float)mouse_state.x, (float)mouse_state.y));
			}				
			PushLog(reportLogList, controlbar.position, expectedPosition);
		}		
		else if (mdROI->cornerNum == 5) {

				putText(frame, "1", mdROI->corners[0], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
				putText(frame, "2", mdROI->corners[1], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
				putText(frame, "3", mdROI->corners[2], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
				putText(frame, "4", mdROI->corners[3], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
				circle(frame, mdROI->corners[0], 3, Scalar(255, 0, 0), 3);
				circle(frame, mdROI->corners[1], 3, Scalar(255, 0, 0), 3);
				circle(frame, mdROI->corners[2], 3, Scalar(255, 0, 0), 3);
				circle(frame, mdROI->corners[3], 3, Scalar(255, 0, 0), 3);
				circle(frame, mdROI->corners[4], 3, Scalar(0, 0, 255), 3);

				vector<Point2f> inputArray = { mdROI->corners[1], mdROI->corners[0], mdROI->corners[3], mdROI->corners[2] };
				renderenceBoard.UpdateCurrentTransform(inputArray);

				Point2f uv = renderenceBoard.GetUVCoordinate(mdROI->corners[4]);
				uvHistoryList.push_back(uv);
				Point2f expectedPosition(0,0);
				GetExpectedPositionFromMeanValue(uvHistoryList, renderenceBoard,  expectedPosition);
				Point2f centerPt(mdROI->corners[4]);

				DrawUVValue(frame, uv, centerPt);
				PushLog(reportLogList, controlbar.position,uv, centerPt, expectedPosition);

				if (mouse_state.flags && EVENT_FLAG_LBUTTON) 
				{
					Point2f uv = renderenceBoard.GetUVCoordinate(Point2f((float)mouse_state.x, (float)mouse_state.y));
					DrawUVValue(frame, uv, Point2f((float)mouse_state.x, (float)mouse_state.y));
				}
			
				DrawStartUV(frame , expectedPosition);
		}
		else {
			cout << "..." << endl;
			PushLogEmpty(reportLogList, controlbar.position);
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


	SaveLog(UVLOG_FILE,reportLogList);

	delete mdROI;
	system("pause");
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
