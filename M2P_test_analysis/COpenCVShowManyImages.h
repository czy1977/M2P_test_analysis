#pragma once
#include <vector>
#include <opencv.hpp>
class COpenCVShowManyImages
{
public:
	COpenCVShowManyImages();
	virtual ~COpenCVShowManyImages();
	static void ShowManyImages(const std::vector<cv::Mat>&srcImages, cv::Size imageSize);
};

