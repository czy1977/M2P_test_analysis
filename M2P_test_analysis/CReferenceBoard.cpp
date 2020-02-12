#include "pch.h"
#include "CReferenceBoard.h"
using namespace cv;
using namespace std;
#define BOARD_WIDTH 250.0
#define BOARD_HEIGHT 150.0

CReferenceBoard::CReferenceBoard()
{
	InitRefPoints();
}


CReferenceBoard::~CReferenceBoard()
{
}

cv::Matx33f CReferenceBoard::GetTrans(std::vector<cv::Point2f> input)
{
	//return findHomography(refPoints, input);

	return findHomography(input,refPoints );
	
}

cv::Size2f CReferenceBoard::GetSize()
{
	return cv::Size2f(BOARD_WIDTH, BOARD_HEIGHT);
}

cv::Point2f CReferenceBoard::GetUVCoordinate(std::vector<cv::Point2f> projectedPoints, cv::Point2f referencePoint)
{
	cv::Matx33f m = GetTrans(projectedPoints);
	return GetUVCoordinate(m, referencePoint);
}

cv::Point2f CReferenceBoard::GetUVCoordinate(cv::Matx33f homography, cv::Point2f referencePoint)
{
	cv::Matx33f  trans(homography);
	vector<cv::Point2f> inputArr(1); 
	inputArr[0] = referencePoint;
	vector<cv::Point2f> outputArr(1);
	
	cv::perspectiveTransform(inputArr, outputArr, homography);

	

	return cv::Point2f(outputArr[0].x / BOARD_WIDTH, outputArr[0].y/ BOARD_HEIGHT);
}



void CReferenceBoard::InitRefPoints()
{
	//250mm x 150mm
	refPoints.resize(4);

	refPoints[0] = Point2f(0, 0);
	
	refPoints[1] = Point2f(BOARD_WIDTH, 0);
	refPoints[2] = Point2f(BOARD_WIDTH, BOARD_HEIGHT);

	refPoints[3] = Point2f(0, BOARD_HEIGHT);

}

