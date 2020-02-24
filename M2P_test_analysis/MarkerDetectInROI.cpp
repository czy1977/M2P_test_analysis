#include "pch.h"
#include "MarkerDetectInROI.h"
#include "COpenCVShowManyImages.h"
//#define DEBUG
//#define SHOW_DEBUG_IMAGE

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
	cvtColor(src, gray, CV_RGB2GRAY);
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

bool MarkerDetectInROI::FindMarkersInROI(std::shared_ptr<cv::SimpleBlobDetector::Params> params, 
	Mat src, int roiSize) {
	vector<KeyPoint> tempKpt;
	vector<Point2f> pt;

	Mat gray;
	cvtColor(src, gray, CV_RGB2GRAY);
	//GaussianBlur(img, img, Size(), 0.5, 0.5);
	Mat invImg = 255 * Mat::ones(gray.size(), CV_8U);
	invImg = invImg - gray;
	//GaussianBlur(invImg, invImg, Size(), 0.5, 0.5);
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(*params);
	vector<Point2f> candidatePts;
	vector<Mat> tempMat(4);
	for (int i = 0; i < 4; i++) {
		
		GetROI(invImg, tempMat[i], corners[i], roiSize);

		detector->detect(tempMat[i], tempKpt);
#ifdef SHOW_DEBUG_IMAGE
		//static const char* debug_window_name[] = {"MarkerDetectInROI_0","MarkerDetectInROI_1","MarkerDetectInROI_2","MarkerDetectInROI_3" };
		//imshow(debug_window_name[i], tempMat);
#endif
		KeyPoint::convert(tempKpt, pt);
		if (pt.size() == 1) {
			pt[0].x = corners[i].x + pt[0].x - roiSize / 2;
			pt[0].y = corners[i].y + pt[0].y - roiSize / 2;
			candidatePts.push_back(pt[0]);
		}
		else {
			candidatePts.clear();
			break;
		}
	}
#ifdef SHOW_DEBUG_IMAGE
	COpenCVShowManyImages::ShowManyImages(tempMat,Size(roiSize, roiSize));
#endif
	
	if (candidatePts.size() == 4) {
		
		corners=candidatePts;

		cornerNum = 4;
		return true;
	}
	else {
		corners.clear();
		cornerNum = 0;
		return false;
	}
	
}

bool MarkerDetectInROI::FindMarkers(std::shared_ptr<cv::SimpleBlobDetector::Params> params, Mat src) {
	if (corners.size() == 0) {
		FindMarkersInWholeImg(params, src);
	}
	else {
		FindMarkersInROI(params, src, 80);
		if (corners.size() == 0) {
			FindMarkersInWholeImg(params, src);
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
	ROI = src(cv::Rect(pt.x - roiSize / 2, pt.y - roiSize / 2, roiSize, roiSize));
}

bool MarkerDetectInROI::FindCenterMarker(std::shared_ptr<cv::SimpleBlobDetector::Params> params,
	Mat src, vector<Point2f> pts, Point2f &centerPt) {
	
	
	return true;
}
