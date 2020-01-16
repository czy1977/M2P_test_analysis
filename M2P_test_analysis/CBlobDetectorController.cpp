#include "pch.h"
#include "CBlobDetectorController.h"
#include <opencv2/highgui.hpp>

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
	cv::namedWindow(windowName, cv::WINDOW_GUI_EXPANDED);
	//cv::resizeWindow(windowName, 960, 540);
	cv::setWindowTitle(windowName, name);

	cv::createTrackbar("filterByArea", windowName, &filterByArea, 1, [](int pos, void* pdata)-> void {
		CBlobDetectorController * pInstance = static_cast <CBlobDetectorController*>(pdata);
		if (NULL != pInstance) 
		{
			pInstance->targetParameter->filterByArea = (pos == 1) ? true : false;
			pInstance->dirty();
		}
	}, this);


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
