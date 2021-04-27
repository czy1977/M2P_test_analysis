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
	updating(false),
	position(0),
	count(0),
	needUpdate(false)
{
  static int tmp;
  cv::createTrackbar(TRACKBAR_NAME, masterWindowName, &tmp, 100, [](int pos, void* pdata)-> void {
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
	cap.set(cv::VideoCaptureProperties::CAP_PROP_POS_FRAMES,position);
		needUpdate = false;
	}
	else if(firstInit)
	{

    count = cap.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
    position = cap.get(cv::VideoCaptureProperties::CAP_PROP_POS_FRAMES);
		updating = true;
		cv::setTrackbarMax(TRACKBAR_NAME, windowName, count);
		cv::setTrackbarPos(TRACKBAR_NAME, windowName, position);
		firstInit = false;

	}
	else
	{
		updating = true;
	position = cap.get(cv::VideoCaptureProperties::CAP_PROP_POS_FRAMES);
		cv::setTrackbarPos(TRACKBAR_NAME, windowName, position);
	}

}

void COpenCVVideoControlBar::UpdateStatus(std::map<double,std::string> const& imageList, std::map<double,std::string>::const_iterator& imageIndex)
{
  if (imageIndex == imageList.end())
    return;

  if (needUpdate) {
    imageIndex = imageList.lower_bound(position);
    needUpdate = false;
  }
  else if(firstInit)
  {
    count = imageList.rbegin()->first+1;
    position = imageIndex->first;
    updating = true;
    cv::setTrackbarMax(TRACKBAR_NAME, windowName, count);
    cv::setTrackbarPos(TRACKBAR_NAME, windowName, position);
    firstInit = false;
  }
  else
  {
    updating = true;
    position = imageIndex->first;
    cv::setTrackbarPos(TRACKBAR_NAME, windowName, position);
  }
}
