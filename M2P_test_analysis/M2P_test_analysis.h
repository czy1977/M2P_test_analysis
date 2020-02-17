#pragma once
#include "opencv2/opencv.hpp"

enum FRAME_CONTROL {
	FRAME_PLAY,
	FRAME_PAUSE,
	FRAME_FORWARD_ONE_FRAME,
	FRAME_BACKWARD_ONE_FRAME,
	FRAME_ONE_FRAME_PAUSE
};

typedef struct {
	cv::Point2f uv;
	cv::Point2f realPositionInPixel;
	cv::Point2f expectedPositionInPixel;
} LOG_INFO;

Point2f GetMeanValue(std::list<cv::Point2f> &uvHistory, int maxNumber);

void DrawStartUV(Mat frame,  cv::Point2f pt);

void GetExpectedPositionFromMeanValue(std::list<cv::Point2f>& uvHistory, CReferenceBoard & refBoard, cv::Point2f & reprojectedPoint);


void ProcessMainLoopKeyEvent(bool & needQuit, FRAME_CONTROL & control);
