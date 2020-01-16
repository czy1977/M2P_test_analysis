#pragma once

#include <opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

namespace ocvi {
	void Reproject3Dto2D(Mat pt3D, Mat R, Mat T, Mat intriMat, Mat &outputPt2D);

	void PoseEstFor3Dto3D(Mat pt3D1, Mat pt3D2, Mat &R, Mat &T);
	void TriangulatePoints(Mat intriMatL, Mat intriMatR, Mat R, Mat T,
		Mat leftCorners, Mat rightCorners, Mat &pt3D);

	void PlotPts(Mat src, Mat pt2D, Scalar color, float radius);
	Mat ConvertPt3D2Mat(vector<Point3f> pt3D);
	Mat ConvertPt2D2Mat(vector<Point2f> pt2D);
	void NormInRows(Mat m1, Mat m2, vector<float> &dist);
	void FindHomoGraphyInStereo(Mat intriMatL, Mat intriMatR, Mat R, Mat T,
		Mat pt2DL, Mat pt2DR, Mat orgPt2D, Mat &H);
	void FindHomoGraphy(Mat pt2DL, Mat pt2DR, Mat &H);

	void Reproject2Dto2D(Mat pt2D, Mat H, Mat &outputPt2D);
}