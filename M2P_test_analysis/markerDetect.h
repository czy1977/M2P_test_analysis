#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>


void InitBlobParams(std::shared_ptr<cv::SimpleBlobDetector::Params> params, float minSize, float maxSize, float minCircularity);

bool FindWhiteInBlackCircleGrid(std::shared_ptr<cv::SimpleBlobDetector::Params> params, cv::Mat src, std::vector<cv::KeyPoint> &corners);

