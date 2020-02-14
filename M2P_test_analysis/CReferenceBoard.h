#pragma once
#include "opencv2/opencv.hpp"
#include <vector>
class CReferenceBoard
{
public:
	CReferenceBoard();
	virtual ~CReferenceBoard();
	std::vector<cv::Point2f> refPoints;
	// get homography matrix  from a perspective projected points
	cv::Matx33f GetTransformation(std::vector<cv::Point2f> input);
	void UpdateCurrentTransform(std::vector<cv::Point2f> input);
	cv::Size2f GetSize();
	// get a UV value from a peojected 2d point based on the . 
	cv::Point2f GetUVCoordinate(std::vector<cv::Point2f> projectedPoints, cv::Point2f referencePoint);
	cv::Point2f GetUVCoordinate(cv::Matx33f homography, cv::Point2f referencePoint);
	cv::Point2f GetReprojectedCoordinate(cv::Matx33f homography, cv::Point2f uv);

	cv::Point2f GetUVCoordinate( cv::Point2f referencePoint);
	cv::Point2f GetReprojectedCoordinate( cv::Point2f uv);

private:
	void InitRefPoints();
	cv::Matx33f homographyMatrix;
};

