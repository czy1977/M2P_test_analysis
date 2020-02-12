#include "pch.h"
#include "COpenCVVideoControlBar.h"
#define TRACKBAR_NAME ("_VideoController")

void COpenCVVideoControlBar::dirty()
{
	needUpdate = true;
}

COpenCVVideoControlBar::COpenCVVideoControlBar(std::string masterWindowName):
	windowName(masterWindowName), 
	firstInit(true),
	updating(false)
{
	cv::createTrackbar(TRACKBAR_NAME, masterWindowName, &position, 100, [](int pos, void* pdata)-> void {
		COpenCVVideoControlBar * pInstance = static_cast <COpenCVVideoControlBar*>(pdata);
		if (NULL != pInstance)
		{
			if (!pInstance->updating) 
			{
				pInstance->position = pos;
				pInstance->dirty();
			}
			pInstance->updating = false;
		}

	}, this);

}


COpenCVVideoControlBar::~COpenCVVideoControlBar()
{
}

void COpenCVVideoControlBar::UpdateStatus(cv::VideoCapture & cap)
{
	if (needUpdate) {
		cap.set(CV_CAP_PROP_POS_FRAMES,(double)position);	
		needUpdate = false;
	}
	else if(firstInit)
	{

		count = cap.get(CV_CAP_PROP_FRAME_COUNT);
		position = cap.get(CV_CAP_PROP_POS_FRAMES);
		updating = true;
		cv::setTrackbarMax(TRACKBAR_NAME, windowName, count);
		cv::setTrackbarPos(TRACKBAR_NAME, windowName, position);
		firstInit = false;

	}
	else
	{
		updating = true;
		position = cap.get(CV_CAP_PROP_POS_FRAMES);
		cv::setTrackbarPos(TRACKBAR_NAME, windowName, position);
	}

}
