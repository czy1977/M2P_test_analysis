#pragma once
#include <opencv2/opencv.hpp>
class COpenCVVideoControlBar
{
private:
	std::string windowName;
	bool needUpdate;
	bool firstInit;
	bool updating;
	void dirty();

public:
  double position; // position (in ms) in the video
	int count;
	COpenCVVideoControlBar(std::string masterWindowName);
	virtual ~COpenCVVideoControlBar();

	void UpdateStatus(cv::VideoCapture & cap);

  void UpdateStatus(std::vector<std::string>& imageList, long& imageIndex);

  void UpdateStatus(std::map<double,std::string> const& imageList, std::map<double,std::string>::const_iterator& imageIndex);

};

