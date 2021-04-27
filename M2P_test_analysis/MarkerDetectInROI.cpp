#include "pch.h"
#include "MarkerDetectInROI.h"
#include "COpenCVShowManyImages.h"
#include <thread>
//#define DEBUG
//#define SHOW_DEBUG_IMAGE

#define RUN_MULTI_THREAD 1
//#define DEBUG_ROI

MarkerDetectInROI::MarkerDetectInROI()
{
	cornerNum = 0;
}

void MarkerDetectInROI::InitBlobParams(std::shared_ptr<cv::SimpleBlobDetector::Params> params,
	float minSize, float maxSize, float minCircularity)
{
  params->minArea = minSize;
  params->maxArea = maxSize;
	params->filterByArea = true;
	params->filterByCircularity = true;
	params->minCircularity = minCircularity;
	params->filterByInertia = false;
	params->filterByConvexity = false;
  params->minDistBetweenBlobs = 20;
  params->maxThreshold = 255;
  params->minThreshold = 50;
}

bool MarkerDetectInROI::FindMarkersInWholeImg(std::shared_ptr<cv::SimpleBlobDetector::Params> params,
	Mat src) {
	corners.clear();
	vector<KeyPoint> tempKpt;
	vector<Point2f> pt;
	float isFoundFlag;
	Mat gray;
	cvtColor(src, gray, cv::ColorConversionCodes::COLOR_RGB2GRAY);
	//GaussianBlur(img, img, Size(), 0.5, 0.5);
	Mat invImg = 255 * Mat::ones(gray.size(), CV_8U);
	invImg = invImg - gray;
	//GaussianBlur(invImg, invImg, Size(), 0.5, 0.5);
  Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(*params);
	// std::vector<KeyPoint> corners;
	detector->detect(invImg, tempKpt);
	KeyPoint::convert(tempKpt, pt);
	//cout << corners.size() << endl;
#ifdef DEBUG
	Mat im_with_corners;
	drawcorners(invImg, corners, im_with_corners, Scalar(255, 0, 0));
	imshow("keypoint", im_with_corners);
	waitKey(0);
#endif // DEBUG

	int tempCornerNum = (int)pt.size();
	if (tempCornerNum == 4) {
		vector<int> hullID;
		convexHull(pt, hullID, false, false);
		if (hullID.size() == 4) {
			vector<Point2f> orderedPt;
			OrderCorners(pt, orderedPt);
			if (IsRec(orderedPt, 20)) {
				isFoundFlag = true;
				cornerNum = 4;
				corners = orderedPt;
			}
			else {
				corners.clear();
				isFoundFlag = false;
				cornerNum = 0;
			}
		}
		else {
			corners.clear();
			isFoundFlag = false;
			cornerNum = 0;
		}

	}
	else if (tempCornerNum == 5){
		vector<int> hullID;
		convexHull(pt, hullID, false, false);
		if (hullID.size() == 4) {
			vector<Point2f> orderedPt;
			OrderCorners(pt, orderedPt);
			if (IsRec(orderedPt, 20)) {
				int centerID = 10 - hullID[0] - hullID[1] - hullID[2] - hullID[3];
				orderedPt.push_back(pt[centerID]);
				isFoundFlag = true;
				cornerNum = 5;
				corners = orderedPt;
			}
			else {
				corners.clear();
				isFoundFlag = false;
				cornerNum = 0;
			}
		}
		else {
			corners.clear();
			isFoundFlag = false;
			cornerNum = 0;
		}
	}
	else
	{
		corners.clear();
		isFoundFlag = false;
		cornerNum = 0;
	}

	return isFoundFlag;
}

bool MarkerDetectInROI::FindMarkersInROI(std::shared_ptr<cv::SimpleBlobDetector::Params> params1,
	std::shared_ptr<cv::SimpleBlobDetector::Params> params2,
	Mat src, int roiSize, int marginSize) {

	vector<Point2f> candidatePts(4);
	bool foundMark = false;
	vector<bool> foundMarkFlags(4);
	vector<Mat> tempMat(4);

	


#if RUN_MULTI_THREAD
	vector<std::thread> threads(4);
	for (int i = 0; i < 4; i++) {
		threads[i] = std::thread(MarkerDetectInROI::DecetROI, ref(src),ref( corners), i, roiSize, ref(*params1), ref(candidatePts), ref(foundMarkFlags));
	}
	for (int i = 0; i < 4; i++) {
		threads[i].join();
	}
#else
	for (int i = 0; i < 4; i++) {
		DecetROI(src,corners, i, roiSize, *params1, candidatePts, foundMarkFlags);
	}
#endif // RUN_MULTI_THREAD

	
	if (std::all_of(foundMarkFlags.begin(), foundMarkFlags.end(), [](bool value) {return value; })) {
		corners=candidatePts;
		Point2f centerPt;
		bool flag = FindCenterMarker(params2, marginSize, src, candidatePts, centerPt);
		if (flag) {
			cornerNum = 5;
			corners.push_back(centerPt);
			return true;
		}
		else {
			cornerNum = 4;
			return true;
		}
		
	}
	else {
		corners.clear();
		cornerNum = 0;
		return false;
	}
	
}

void MarkerDetectInROI::DecetROI(const cv::Mat & srcImg, const vector<Point2f> & corners, int i, int roiSize, const cv::SimpleBlobDetector::Params& parameters, vector<Point2f> & candidatePts, vector<bool> & foundMark)
{
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(parameters);
	vector<KeyPoint> tempKpt;
	vector<Point2f> pt;
	Mat tempMatColor, tempMat;
	GetROI(srcImg, tempMatColor, corners[i], roiSize);

	if (tempMatColor.rows<10 || tempMatColor.cols<10) {
		std::cout << "too small image: " << tempMatColor.rows << "x" << tempMatColor.cols << std::endl;
		foundMark[i] = false;
		return;
	}

	Mat gray;
	cvtColor(tempMatColor, gray, cv::ColorConversionCodes::COLOR_RGB2GRAY);
	tempMat = 255 - gray;
	detector->detect(tempMat, tempKpt);
	KeyPoint::convert(tempKpt, pt);
	if (pt.size() == 1) {

		pt[0].x = (int)(corners[i].x) + pt[0].x - roiSize / 2;
		pt[0].y = (int)(corners[i].y) + pt[0].y - roiSize / 2;
		candidatePts[i] = pt[0];
		foundMark[i] = true;
#ifdef DEBUG_ROI
		cout << i << " roi center:" << corners[i].x << ", " << corners[i].y << endl;
		cout << i << " point:" << pt[0].x << ", " << pt[0].y << endl;
#endif
	}
	else {
		foundMark[i] = false;
	}
}

bool MarkerDetectInROI::FindMarkers(std::shared_ptr<cv::SimpleBlobDetector::Params> params1,
	std::shared_ptr<cv::SimpleBlobDetector::Params> params2,
	Mat src, int roiSize, int marginSize) {
	if (corners.size() == 0) {
		FindMarkersInWholeImg(params1, src);
	}
	else {
		FindMarkersInROI(params1, params2, src, roiSize, marginSize);
		if (corners.size() == 0) {
			FindMarkersInWholeImg(params1, src);
		}
	}
	return true;
}

MarkerDetectInROI::~MarkerDetectInROI()
{
	corners.clear();
	cout << "the marker detector is released." << endl;
}

void MarkerDetectInROI::OrderCorners(vector<Point2f> orgCorners, vector<Point2f> &orderedCorners) {
	float aveX = 0;
	float aveY = 0;
	for (int i = 0; i < orgCorners.size(); i++) {
		aveX += orgCorners[i].x;
		aveY += orgCorners[i].y;
	}
	aveX = aveX / orgCorners.size();
	aveY = aveY / orgCorners.size();
	for (int i = 0; i < orgCorners.size(); i++) {
		if (orgCorners[i].x >= aveX && orgCorners[i].y <= aveY) {
			orderedCorners.push_back(orgCorners[i]);
			break;
		}
	}
	for (int i = 0; i < orgCorners.size(); i++) {
		if (orgCorners[i].x <= aveX && orgCorners[i].y <= aveY) {
			orderedCorners.push_back(orgCorners[i]);
			break;
		}
	}
	for (int i = 0; i < orgCorners.size(); i++) {
		if (orgCorners[i].x <= aveX && orgCorners[i].y >= aveY) {
			orderedCorners.push_back(orgCorners[i]);
			break;
		}
	}
	for (int i = 0; i < orgCorners.size(); i++) {
		if (orgCorners[i].x >= aveX && orgCorners[i].y >= aveY) {
			orderedCorners.push_back(orgCorners[i]);
			break;
		}
	}
}

bool MarkerDetectInROI::IsRec(vector<Point2f> &orderedCorners, float thd) {
	float dx = orderedCorners[0].x - orderedCorners[1].x;
	float dy = orderedCorners[0].y - orderedCorners[1].y;
	float w1 = sqrt(dx*dx + dy*dy);
	dx = orderedCorners[2].x - orderedCorners[3].x;
	dy = orderedCorners[2].y - orderedCorners[3].y;
	float w2 = sqrt(dx*dx + dy*dy);
	dx = orderedCorners[0].x - orderedCorners[2].x;
	dy = orderedCorners[0].y - orderedCorners[2].y;
	float h1 = sqrt(dx*dx + dy*dy);
	dx = orderedCorners[1].x - orderedCorners[3].x;
	dy = orderedCorners[1].y - orderedCorners[3].y;
	float h2 = sqrt(dx*dx + dy*dy);

	if (abs(w1 - w2) < thd && abs(h1 - h2) < thd) {
		return true;
	}
	else {
		return false;
	}
}

void MarkerDetectInROI::GetROI(Mat src, Mat &ROI, Point2f pt, int roiSize) {
	auto s = roiSize/2;
	cv::Range cols(std::max(0, (int)(pt.x - s)), std::min(src.cols, (int)(pt.x + s + 1)));
	cv::Range rows(std::max(0, (int)(pt.y - s)), std::min(src.rows, (int)(pt.y + s + 1)));


	if (cols.start>cols.end) cols.start = cols.end;
	if (rows.start>rows.end) rows.start = rows.end;

	std::cout << "GetROI: " << cols.start << ":" << cols.end << " " << rows.start << ":" << rows.end << std::endl;


	ROI = src(rows, cols);
//  ROI = src(cv::Rect((int)(pt.x - roiSize / 2), (int)(pt.y - roiSize / 2), roiSize, roiSize));
}

bool MarkerDetectInROI::FindCenterMarker(std::shared_ptr<cv::SimpleBlobDetector::Params> params,
	int thd, Mat src, vector<Point2f> pts, Point2f &centerPt) {
	vector<KeyPoint> tempKpt;
	vector<Point2f> pt;
	Mat gray;
	//GaussianBlur(invImg, invImg, Size(), 0.5, 0.5);
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(*params);
	//float thd = 100;
  int p1y = max(pts[0].y+thd, pts[1].y+thd);
	int p1x = max(pts[1].x+thd, pts[2].x+thd);
	int p2y = min(pts[2].y-thd, pts[3].y-thd);
	int p2x = min(pts[0].x-thd, pts[3].x-thd);

	if (p1x<0 || p1x>=src.cols || p1y<0 || p1y>=src.rows || p2x <= p1x ||  p2y <= p1y) {
		std::cout << "wrong ROI: " << p1x << ","<< p1y << " " << p2x << "," << p2y << " " << src.cols << "x" << src.rows	<< std::endl;
		return false;
	}


	Mat srcROI = src(cv::Rect(p1x, p1y, p2x - p1x, p2y - p1y));


	cvtColor(srcROI, gray, cv::ColorConversionCodes::COLOR_RGB2GRAY);
	Mat ROI = 255 - gray;;
	detector->detect(ROI, tempKpt);
	KeyPoint::convert(tempKpt, pt);

	if (pt.size() == 1) {
		pt[0].x = pt[0].x + p1x;
		pt[0].y = pt[0].y + p1y;
		centerPt = pt[0];
		return true;
	}
	else {
		return false;
	}
	
}
