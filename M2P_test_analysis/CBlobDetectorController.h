#pragma once
#include <opencv.hpp>
#include <memory>
class CBlobDetectorController
{
	friend void _CBlobDetectorController_UpdateCallBack(int pos, void* pdata);
protected:
	std::string windowName;
	bool needUpdate;
	void dirty();

	


	// the field used by blob parameter


	int filterByColor;
	uchar blobColor;

	int filterByArea;
	int minArea, maxArea;

	int filterByCircularity;
	int minCircularity, maxCircularity;

	int filterByInertia;
	int minInertiaRatio, maxInertiaRatio;

	int filterByConvexity;
	int minConvexity, maxConvexity;

	 /*
	 float thresholdStep;
	 float minThreshold;
	 float maxThreshold;
	 size_t minRepeatability;
	 float minDistBetweenBlobs;

	 bool filterByColor;
	 uchar blobColor;

	 bool filterByArea;
	 float minArea, maxArea;

	 bool filterByCircularity;
	 float minCircularity, maxCircularity;

	 bool filterByInertia;
	 float minInertiaRatio, maxInertiaRatio;

	 bool filterByConvexity;
	 float minConvexity, maxConvexity;
	 */
public:
	CBlobDetectorController();
	~CBlobDetectorController();
	void open(std::shared_ptr<cv::SimpleBlobDetector::Params> parameters,std::string name);
	std::shared_ptr<cv::SimpleBlobDetector::Params> targetParameter;
	bool isDirty() { return needUpdate; }

};

