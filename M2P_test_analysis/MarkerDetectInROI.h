#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>

using namespace std;
using namespace cv;

class MarkerDetectInROI
{
public:
	vector<Point2f> corners;
	int cornerNum;
	MarkerDetectInROI();
	~MarkerDetectInROI();
	void InitBlobParams(std::shared_ptr<cv::SimpleBlobDetector::Params> params, float minSize, float maxSize, float minCircularity);
	bool FindMarkersInWholeImg(std::shared_ptr<cv::SimpleBlobDetector::Params> params, Mat src);
	bool FindMarkersInROI(std::shared_ptr<cv::SimpleBlobDetector::Params> params1, 
		std::shared_ptr<cv::SimpleBlobDetector::Params> params2, 
		Mat src, int roiSize, int marginSize);
	static void DecetROI(const cv::Mat & srcImg, const vector<Point2f> & corners, int i, int roiSize, const cv::SimpleBlobDetector::Params& parameters, vector<Point2f> & candidatePts, vector<bool> & foundMark);
	bool FindMarkers(std::shared_ptr<cv::SimpleBlobDetector::Params> params1,
		std::shared_ptr<cv::SimpleBlobDetector::Params> params2,
		Mat src, int roiSize, int marginSize);

	
	void OrderCorners(vector<Point2f> orgCorners, vector<Point2f> &orderedCorners);
	bool IsRec(vector<Point2f> &orderedCorners, float thd);
	static void GetROI(Mat src, Mat &ROI, Point2f pt, int roiSize);
	bool FindCenterMarker(std::shared_ptr<cv::SimpleBlobDetector::Params> params, int thd,
		Mat src, vector<Point2f> pts, Point2f &centerPt);
};

