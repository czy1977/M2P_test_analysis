#pragma once
#include "opencv2/opencv.hpp"

#include "CReferenceBoard.h"

enum FRAME_CONTROL {
	FRAME_PLAY,
	FRAME_PAUSE,
	FRAME_FORWARD_ONE_FRAME,
	FRAME_BACKWARD_ONE_FRAME,
	FRAME_ONE_FRAME_PAUSE
};

typedef struct {
	int frameID;
	cv::Point2f uv;
	cv::Point2f realPositionInPixel;
	cv::Point2f expectedPositionInPixel;
} LOG_INFO;

cv::Point2f GetMeanValue(std::list<cv::Point2f> &uvHistory, int maxNumber);


void GetExpectedPositionFromMeanValue(std::list<cv::Point2f>& uvHistory, CReferenceBoard & refBoard, cv::Point2f & reprojectedPoint);


void ProcessMainLoopKeyEvent(bool & needQuit, FRAME_CONTROL & control);
void PushLogEmpty(std::list<LOG_INFO> & logList, int frameID);
void PushLog(std::list<LOG_INFO> & logList, int frameID, const cv::Point2f & expectedPosition);
void PushLog(std::list<LOG_INFO> & logList, int frameID, const cv::Point2f & expectedPosition, const cv::Point2f & uv, const cv::Point2f & realPositionInPixel);
