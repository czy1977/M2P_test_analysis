#pragma once
#include <opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

bool FindWhiteInBlackCircleGrid(Mat src, float maxSize, float minSize, float minCircularity, vector<KeyPoint> &corners);

