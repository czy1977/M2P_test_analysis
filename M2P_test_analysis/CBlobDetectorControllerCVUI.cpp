#include "pch.h"
#include "CBlobDetectorControllerCVUI.h"
#define CVUI_IMPLEMENTATION
#include "cvui.h"
#include <iostream>
#include <opencv2/core/persistence.hpp>

#define MAX_AREA_RANGE 10000
#define MAX_DISTENCE 100
#define MINCIRCULARITY_RANGE 100

bool CBlobDetectorControllerCVUI::isInit = false;



void CBlobDetectorControllerCVUI::save()
{
	cv::FileStorage fs(cv::String(getConfigFileName()), cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON, "UTF-8");
	try
	{
		targetParameter->write(fs);
	}
	catch (const cv::Exception & e)
	{
		std::cout << "write " << windowName << " failed, file save error. " << e.err << std::endl;
	}
	
}

void CBlobDetectorControllerCVUI::load()
{
	cv::FileStorage fs(cv::String(getConfigFileName()), cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON, "UTF-8");
	if (fs.isOpened()) {
		cv::FileNode n = fs.root();
		try
		{
			targetParameter->read(n);
		}
		catch (const cv::Exception & e)
		{
			std::cout << "read " << windowName << " failed, parse error. "<<e.err << std::endl;
		}
		
	}
	else
	{
		std::cout << "read " << windowName << " failed, file not exist. " << std::endl;
	}
}

CBlobDetectorControllerCVUI::CBlobDetectorControllerCVUI()
{
}


CBlobDetectorControllerCVUI::~CBlobDetectorControllerCVUI()
{
}

void CBlobDetectorControllerCVUI::open(std::shared_ptr<cv::SimpleBlobDetector::Params> parameters, std::string name, bool isLoadFromFile )
{
	targetParameter = parameters;

	windowName = name + "_config";
	frame = cv::Mat(850, 500, CV_8UC3);
	cv::namedWindow(windowName);
	if (isLoadFromFile) {
		load();
	}
	if (!isInit) {
		cvui::init(windowName, -1, false);
		isInit = true;
	}
	else
	{
		cvui::watch(windowName);
	}
	
	

}
 std::string CBlobDetectorControllerCVUI::getConfigFileName() {
	 return "./" + windowName + "_save.json";
}
void CBlobDetectorControllerCVUI::loop()
{

	int baseX = 10;
	int baseY = 50;
	int setpX = 150;
	int setpY = 40;
	int textOffset = 15;
	int trackbarWidth = 300;
	frame = cv::Scalar(49, 52, 49);
	cvui::context(windowName);
	if (cvui::button(frame, 50, 20, "Save config")) {
		std::cout << "Save  button click" << std::endl;
		save();
	}
	if (cvui::button(frame, 200, 20, "Load config")) {
		std::cout << "Load  button click" << std::endl;
		load();
	}
	int row = 0;

	cvui::checkbox(frame, baseX , baseY + setpY* row + textOffset, "filterByArea", &targetParameter->filterByArea);

	row++;
	cvui::text(frame, baseX, baseY + setpY * row + textOffset, "minArea");
	cvui::trackbar<float>(frame, baseX+ setpX, baseY+ setpY* row, trackbarWidth, &targetParameter->minArea,0, MAX_AREA_RANGE);

	row++;
	cvui::text(frame, baseX, baseY + setpY * row + textOffset, "maxArea");
	cvui::trackbar<float>(frame, baseX + setpX, baseY + setpY * row, trackbarWidth, &targetParameter->maxArea, 0, MAX_AREA_RANGE);

	row++;
	cvui::checkbox(frame, baseX, baseY + setpY * row + textOffset, "filterByCircularity", &targetParameter->filterByCircularity);

	row++;
	cvui::text(frame, baseX, baseY + setpY * row + textOffset, "minCircularity");
	cvui::trackbar<float>(frame, baseX + setpX, baseY + setpY * row, trackbarWidth, &targetParameter->minCircularity, 0, 1);

	row++;
	cvui::text(frame, baseX, baseY + setpY * row + textOffset, "maxCircularity");
	cvui::trackbar<float>(frame, baseX + setpX, baseY + setpY * row, trackbarWidth, &targetParameter->maxCircularity, 0, 1);


	row++;
	cvui::checkbox(frame, baseX, baseY + setpY * row + textOffset, "filterByConvexity", &targetParameter->filterByConvexity);

	row++;
	cvui::text(frame, baseX, baseY + setpY * row + textOffset, "minConvexity");
	cvui::trackbar<float>(frame, baseX + setpX, baseY + setpY * row, trackbarWidth, &targetParameter->minConvexity, 0, 1);

	row++;
	cvui::text(frame, baseX, baseY + setpY * row + textOffset, "maxConvexity");
	cvui::trackbar<float>(frame, baseX + setpX, baseY + setpY * row, trackbarWidth, &targetParameter->maxConvexity, 0, 1);

	row++;
	cvui::checkbox(frame, baseX, baseY + setpY * row + textOffset, "filterByInertia", &targetParameter->filterByInertia);

	row++;
	cvui::text(frame, baseX, baseY + setpY * row + textOffset, "minInertiaRatio");
	cvui::trackbar<float>(frame, baseX + setpX, baseY + setpY * row, trackbarWidth, &targetParameter->minInertiaRatio, 0, 1);

	row++;
	cvui::text(frame, baseX, baseY + setpY * row + textOffset, "maxInertiaRatio");
	cvui::trackbar<float>(frame, baseX + setpX, baseY + setpY * row, trackbarWidth, &targetParameter->maxInertiaRatio, 0, 1);

	row++;
	cvui::text(frame, baseX, baseY + setpY * row + textOffset, "minDistBetweenBlobs");
	cvui::trackbar<float>(frame, baseX + setpX, baseY + setpY * row, trackbarWidth, &targetParameter->minDistBetweenBlobs, 0, MAX_DISTENCE);

	row++;
	cvui::text(frame, baseX, baseY + setpY * row + textOffset, "thresholdStep");
	cvui::trackbar<float>(frame, baseX + setpX, baseY + setpY * row, trackbarWidth, &targetParameter->thresholdStep, 0, 100);

	row++;
	cvui::text(frame, baseX, baseY + setpY * row + textOffset, "minThreshold");
	cvui::trackbar<float>(frame, baseX + setpX, baseY + setpY * row, trackbarWidth, &targetParameter->minThreshold, 0, 255);

	row++;
	cvui::text(frame, baseX, baseY + setpY * row + textOffset, "maxThreshold");
	cvui::trackbar<float>(frame, baseX + setpX, baseY + setpY * row, trackbarWidth, &targetParameter->maxThreshold, 0, 255);

	row++;
	cvui::text(frame, baseX, baseY + setpY * row + textOffset, "minRepeatability");
	cvui::trackbar<size_t>(frame, baseX + setpX, baseY + setpY * row, trackbarWidth, &targetParameter->minRepeatability, 0, 100);

	row++;
	cvui::checkbox(frame, baseX, baseY + setpY * row + textOffset, "filterByColor", &targetParameter->filterByColor);

	row++;
	cvui::text(frame, baseX, baseY + setpY * row + textOffset, "blobColor");
	cvui::trackbar<uchar>(frame, baseX + setpX, baseY + setpY * row, trackbarWidth, &targetParameter->blobColor, 0, 255);
	//cvui::update(windowName);
	cvui::imshow(windowName, frame);
}
