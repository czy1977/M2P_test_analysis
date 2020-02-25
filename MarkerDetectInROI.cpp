#include "MarkerDetectInROI.h"

//#define DEBUG

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

	int tempCornerNum = pt.size();
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
	//float thd = 10;
	vector<KeyPoint> tempKpt;
	vector<Point2f> pt;
	float isFoundFlag;
	Mat gray;
	cvtColor(src, gray, CV_RGB2GRAY);
	//GaussianBlur(img, img, Size(), 0.5, 0.5);
	Mat invImg = 255 * Mat::ones(gray.size(), CV_8U);
	invImg = invImg - gray;
	//GaussianBlur(invImg, invImg, Size(), 0.5, 0.5);
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(*params1);
	vector<Point2f> candidatePts;
	for (int i = 0; i < 4; i++) {
		Mat tempMat;
		GetROI(invImg, tempMat, corners[i], roiSize);
		//imshow("1", tempMat);
		//waitKey(0);
		detector->detect(tempMat, tempKpt);
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
	
	if (candidatePts.size() == 4) {
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
	ROI = src(cv::Rect(pt.x - roiSize / 2, pt.y - roiSize / 2, roiSize, roiSize));
}

bool MarkerDetectInROI::FindCenterMarker(std::shared_ptr<cv::SimpleBlobDetector::Params> params,
	int thd, Mat src, vector<Point2f> pts, Point2f &centerPt) {
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
	//float thd = 100;
	int p1y = max(pts[0].y+thd,pts[1].y+thd);
	int p1x = max(pts[1].x+thd, pts[2].x+thd);
	int p2y = min(pts[2].y-thd, pts[3].y-thd);
	int p2x = min(pts[0].x-thd, pts[3].x-thd);
	Mat ROI = invImg(cv::Rect(p1x, p1y, p2x - p1x, p2y - p1y));
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
