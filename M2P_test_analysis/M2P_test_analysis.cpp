// M2P_test_analysis.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <memory>
#include <ctime>
#include "opencv2/opencv.hpp"
#include <iostream>
#include "markerDetect.h"
#include "OCVInterface.h"
#include "CBlobDetectorControllerCVUI.h"
#include "COpenCVVideoControlBar.h"
#include "CReferenceBoard.h"
#include "M2P_test_analysis.h"
#include "MarkerDetectInROI.h"
#include "cmdline/cmdline.h"


#define MAIN_WINDOW_NAME "Frame"
#define UV_AVRAGE_NUMBER 20


#define KEYCODE_ESCAP 27
#define KEYCODE_SPACE 32
#define KEYCODE_LEFT 2424832
#define KEYCODE_RIGHT 2555904


using namespace std;
using namespace cv;

float maxBlobSize = 8000;
float minBlobSize = 1500;
//float minCircularity = 0.7f;
float minCircularity = 0.77f;

float maxVirtualBlobSize = 5000;
float minVirtualBlobSize = 1000;
float minVirtualCircularity = 0.5f;

//float maxBlobSize = 5000;
//float minBlobSize = 200;
//float minCircularity = 0.7f;
int roiSize = 80;
int marginSize = 20;


#define VIDEO_FILE "video/video.mp4"
#define UVLOG_FILE "logs/log.csv"


#define VIDEO_START_FRAME (0)
#define VIDEO_END_FRAME (300000)
#define LOG_FORMAT_VERSION 1 

//#define OUTPUT_FPS
#define RUN_EMPTY_LOOP 0

#define SKIP_DRAWING 0

#define MULTI_THREAD_DECODE 1


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
#if SKIP_DRAWING
	return;
#endif // SKIP_DRAWING
	std::ostringstream uvText;
	uvText << std::setprecision(2);
	uvText << uv.x << "," << uv.y;
	std::string text = uvText.str();
	cv::putText(frame, text, pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
}
void DrawStartUV(Mat  frame, Point2f pt) {
#if SKIP_DRAWING
	return;
#endif // SKIP_DRAWING

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
	return;
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
void initArgParser(int argc, char *argv[], cmdline::parser & parser) {
  // add specified type of variable.
  // 1st argument is long name
  // 2nd argument is short name (no short name if '\0' specified)
  // 3rd argument is description
  // 4th argument is mandatory (optional. default is false)
  // 5th argument is default value  (optional. it used when mandatory is false)
	parser.add<std::string>("video", 'v', "video file path", true, VIDEO_FILE);
	parser.add<std::string>("log", 'l', "output log file path", true, UVLOG_FILE);

	parser.set_program_name("M2P_test_analysis");
	parser.parse_check(argc, argv);
}

int main(int argc, char *argv[]) {
	Mat frame;
	list<LOG_INFO> reportLogList;
	std::list<cv::Point2f> uvHistoryList;
	MarkerDetectInROI *mdROI = new MarkerDetectInROI;

	CReferenceBoard renderenceBoard;



	// parse cmd line argument
	cmdline::parser cmdParser;
	initArgParser(argc, argv, cmdParser);
	std::string videoFile;	
	std::string logFile;
	videoFile = cmdParser.get<string>("video");
	logFile = cmdParser.get<string>("log");


	VideoCapture cap(videoFile);
	
	cap.set(CV_CAP_PROP_POS_FRAMES, VIDEO_START_FRAME);
	frameControlFlag = FRAME_PLAY;
	if (!cap.isOpened()) {
		
		cout << "Error opening video stream or file" << endl;
		return -1;
	}

	initWindow();

	CBlobDetectorControllerCVUI blobEnvMarkConfigBar;
	CBlobDetectorControllerCVUI blobVirtualMarkConfigBar;
	std::shared_ptr<cv::SimpleBlobDetector::Params> EnvMarkBlobParams(new cv::SimpleBlobDetector::Params);
	std::shared_ptr<cv::SimpleBlobDetector::Params> virtualMarkBlobParams(new cv::SimpleBlobDetector::Params);

	
	InitBlobParams(EnvMarkBlobParams, minBlobSize, maxBlobSize, minCircularity);

	InitBlobParams(virtualMarkBlobParams, minVirtualBlobSize, maxVirtualBlobSize, minVirtualCircularity);

	mdROI->InitBlobParams(EnvMarkBlobParams, minBlobSize, maxBlobSize, minCircularity);
	mdROI->InitBlobParams(virtualMarkBlobParams, minVirtualBlobSize, maxVirtualBlobSize, minVirtualCircularity);

	blobEnvMarkConfigBar.open(EnvMarkBlobParams,"big_mark");
	blobVirtualMarkConfigBar.open(virtualMarkBlobParams, "small_mark");



	COpenCVVideoControlBar controlbar(MAIN_WINDOW_NAME);

	cv::setMouseCallback(MAIN_WINDOW_NAME,[](int event, int x, int y, int flags, void *userdata)-> void {
		mouse_state.event = event;
		mouse_state.x = x;
		mouse_state.y = y;
		mouse_state.flags = flags;
	});
	bool needQuit = false;
	std::clock_t lastTime = clock();

	while (!needQuit) {

		blobEnvMarkConfigBar.loop();
		blobVirtualMarkConfigBar.loop();
		std::clock_t currentTime = clock();
#ifdef OUTPUT_FPS
		cout << "FPS is:" << CLOCKS_PER_SEC/ (double)(currentTime - lastTime) << endl;		
#endif // OUTPUT_FPS
		lastTime = currentTime;


		frame = GetVideoFrame(cap, frameControlFlag);
		controlbar.UpdateStatus(cap);	

	
		if (controlbar.position > VIDEO_END_FRAME) {
			cout << "reach to max frame" << endl;
			break;
		}
		if (frame.empty()) {
			
			cout << "frame empty"<<endl;
			break;
		}
#if RUN_EMPTY_LOOP
		imshow("Frame", frame);
		continue;
#endif // RUN_EMPTY_LOOP

		
		
		vector<KeyPoint> corners;
		
		
		bool isFoundFlag =false;


		isFoundFlag = mdROI->FindMarkers(EnvMarkBlobParams, virtualMarkBlobParams, frame, roiSize, marginSize);


		if (mdROI->cornerNum == 4) {
#if SKIP_DRAWING
#else
			putText(frame, "1", mdROI->corners[0], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
			putText(frame, "2", mdROI->corners[1], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
			putText(frame, "3", mdROI->corners[2], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
			putText(frame, "4", mdROI->corners[3], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
			circle(frame, mdROI->corners[0], 3, Scalar(255, 0, 0), 3);
			circle(frame, mdROI->corners[1], 3, Scalar(255, 0, 0), 3);
			circle(frame, mdROI->corners[2], 3, Scalar(255, 0, 0), 3);
			circle(frame, mdROI->corners[3], 3, Scalar(255, 0, 0), 3);
#endif // SKIP_DRAWING


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
#if SKIP_DRAWING
#else
			putText(frame, "1", mdROI->corners[0], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
			putText(frame, "2", mdROI->corners[1], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
			putText(frame, "3", mdROI->corners[2], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
			putText(frame, "4", mdROI->corners[3], cv::FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
			circle(frame, mdROI->corners[0], 3, Scalar(255, 0, 0), 3);
			circle(frame, mdROI->corners[1], 3, Scalar(255, 0, 0), 3);
			circle(frame, mdROI->corners[2], 3, Scalar(255, 0, 0), 3);
			circle(frame, mdROI->corners[3], 3, Scalar(255, 0, 0), 3);
			circle(frame, mdROI->corners[4], 3, Scalar(0, 0, 255), 3);
#endif // SKIP_DRAWING


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
			cout << "out of mark number ..."<< mdROI->cornerNum << endl;
			for (int i = 0; i < mdROI->cornerNum; i++) {
				circle(frame, mdROI->corners[i], 3, Scalar(255, 0, 0), 3);
			}
			

			PushLogEmpty(reportLogList, controlbar.position);
		}

		
		imshow("Frame", frame);

		bool needQuit = false;
		ProcessMainLoopKeyEvent(needQuit, frameControlFlag);
		if (needQuit) 
			break;
#ifdef OUTPUT_FPS
		std::clock_t finishTime = clock();
		cout << " The run time is:" << (double)( finishTime - currentTime ) / CLOCKS_PER_SEC *1000 << "ms" << endl;
#endif // OUTPUT_FPS
		
		
	}

	// When everything done, release the video capture object
	cap.release();

	// Closes all the frames
	destroyAllWindows();


	SaveLog(logFile,reportLogList);

	delete mdROI;
	//system("pause");
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
