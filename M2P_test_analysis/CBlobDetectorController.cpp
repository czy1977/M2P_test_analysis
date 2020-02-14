#include "pch.h"
#include "CBlobDetectorController.h"
#include <opencv2/highgui.hpp>
#define AREA_RANGE 5000
#define MINCIRCULARITY_RANGE 100

void _CBlobDetectorController_UpdateCallBack( int pos,void* pdata) {
	CBlobDetectorController * pInstance = static_cast <CBlobDetectorController*>(pdata);
	if (NULL != pInstance) {
		pInstance->dirty();
	}
}

void CBlobDetectorController::dirty()
{
	needUpdate = true;
}

CBlobDetectorController::CBlobDetectorController()
	:needUpdate(false)
{
}


CBlobDetectorController::~CBlobDetectorController()
{
}

void CBlobDetectorController::open(std::shared_ptr<cv::SimpleBlobDetector::Params> parameters, std::string name)
{
	targetParameter = parameters;
	windowName = name + "_win";
	cv::namedWindow(windowName, cv::WINDOW_NORMAL);
	cv::setWindowTitle(windowName, name);
	cv::resizeWindow(windowName,cv::Size(400,100 ));

	cv::createTrackbar("filterByArea", windowName, &filterByArea, 1, [](int pos, void* pdata)-> void {
		CBlobDetectorController * pInstance = static_cast <CBlobDetectorController*>(pdata);
		if (NULL != pInstance) 
		{
			pInstance->targetParameter->filterByArea = (pos == 1) ? true : false;
			pInstance->dirty();
		}
	}, this);
	cv::setTrackbarPos("filterByArea", windowName, targetParameter->filterByArea?1:0);

	cv::createTrackbar("minArea", windowName, &minArea, AREA_RANGE, [](int pos, void* pdata)-> void {
		CBlobDetectorController * pInstance = static_cast <CBlobDetectorController*>(pdata);
		if (NULL != pInstance)
		{
			pInstance->targetParameter->minArea = pos;
			pInstance->dirty();
		}
	}, this);
	cv::setTrackbarPos("minArea", windowName, (int)targetParameter->minArea);

	cv::createTrackbar("maxArea", windowName, &maxArea, AREA_RANGE, [](int pos, void* pdata)-> void {
		CBlobDetectorController * pInstance = static_cast <CBlobDetectorController*>(pdata);
		if (NULL != pInstance)
		{
			pInstance->targetParameter->maxArea = pos;
			pInstance->dirty();
		}
	}, this);
	cv::setTrackbarPos("maxArea", windowName, (int)targetParameter->maxArea);

	cv::createTrackbar("minCircularity", windowName, &minCircularity, MINCIRCULARITY_RANGE, [](int pos, void* pdata)-> void {
		CBlobDetectorController * pInstance = static_cast <CBlobDetectorController*>(pdata);
		if (NULL != pInstance)
		{
			pInstance->targetParameter->minCircularity = (float)pos / (float)MINCIRCULARITY_RANGE;
			pInstance->dirty();
		}
	}, this);
	cv::setTrackbarPos("minCircularity", windowName, (int)(targetParameter->minCircularity*MINCIRCULARITY_RANGE));

	//SimpleBlobDetector::Params params;
	//params.minArea = minSize;
	//params.maxArea = maxSize;
	//params.filterByArea = true;
	//params.filterByCircularity = true;
	//params.minCircularity = minCircularity;
	//params.filterByInertia = false;
	//params.filterByConvexity = false;
	//params.minDistBetweenBlobs = 20;
	//params.maxThreshold = 255;
	//params.minThreshold = 50;
}
