#include "pch.h"
#include "markerDetect.h"
//#define DEBUG


bool FindWhiteInBlackCircleGrid(Mat src, float maxSize, float minSize, float minCircularity, vector<KeyPoint> &keypoints) {
	float isFoundFlag;
	Mat gray;
	cvtColor(src, gray, CV_RGB2GRAY);
	SimpleBlobDetector::Params params;
	params.minArea = minSize;
	params.maxArea = maxSize;
	params.filterByArea = true;
	params.filterByCircularity = true;
	params.minCircularity = minCircularity;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.minDistBetweenBlobs = 20;
	params.maxThreshold = 255;
	params.minThreshold = 50;
	//GaussianBlur(img, img, Size(), 0.5, 0.5);

	Mat invImg = 255 * Mat::ones(gray.size(), CV_8U);
	invImg = invImg - gray;
	//GaussianBlur(invImg, invImg, Size(), 0.5, 0.5);


	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	// std::vector<KeyPoint> keypoints;
	detector->detect(invImg, keypoints);
	
	cout << keypoints.size() << endl;

#ifdef DEBUG
	Mat im_with_keypoints;
	drawKeypoints(invImg, keypoints, im_with_keypoints, Scalar(255, 0, 0));
	imshow("keypoint", im_with_keypoints);
	waitKey(0);
#endif // DEBUG

	if (keypoints.size() == 4 && keypoints.size() == 5) {

		isFoundFlag = true;
	}
	else
	{
		isFoundFlag = false;
	}

	return isFoundFlag;
}

