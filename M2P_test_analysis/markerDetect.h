#pragma once
#include <opencv.hpp>
#include <iostream>
#include <memory>

using namespace std;
using namespace cv;

void InitBlobParams(std::shared_ptr<cv::SimpleBlobDetector::Params> params, float minSize, float maxSize, float minCircularity);

bool FindWhiteInBlackCircleGrid(std::shared_ptr<cv::SimpleBlobDetector::Params> params, Mat src, vector<KeyPoint> &corners);

