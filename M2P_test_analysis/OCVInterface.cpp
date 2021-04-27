#include "pch.h"
#include "OCVInterface.h"

using namespace cv;

void ocvi::Reproject3Dto2D(Mat pt3D, Mat R, Mat T, Mat intriMat, Mat &outputPt2D) {
	// This function is to reproject 3d point in camera coordinate system (CCS) to 2d point in image coordinate system (ICS).
	// parameters:
	// pt3D is n*3 matrix which represents n 3d points
	// R is a 3*3 rotation matrix for initial camera coordiante system to current camera coordiante system
	// T is a 1*3 translation matrix for initial camera coordiante system to current camera coordiante system
	// intriMat is a 3*3 intrinsic matrix
	// outputPt2D is a n*2 matrix which represents n reprojected 2d points

	R.convertTo(R, CV_32F);
	T.convertTo(T, CV_32F);
	Mat homoPtICS = pt3D.clone();
	for (int i = 0; i < pt3D.rows; i++) {
		Mat pt3D_ = pt3D.row(i);
		pt3D_ = intriMat*(R* pt3D_.t() + T);
		homoPtICS.row(i) = pt3D_.t();
	}
  std::vector<Point2f> point2D;
	convertPointsFromHomogeneous(homoPtICS, point2D);
	outputPt2D = Mat(point2D).clone();
}

void ocvi::PlotPts(Mat src, Mat pt2D, Scalar color, float radius) {
	// This function is plot 2d points on image.
	// parameters:
	// src is an input image
	// pt2D is n*2 matrix for 2D points
	pt2D.convertTo(pt2D, CV_32F);
	for (int i = 0; i < pt2D.rows; i++) {
		circle(src, Point2f(pt2D.at<float>(i,0), pt2D.at<float>(i, 1)), radius, color, 3);
	}
}

Mat ocvi::ConvertPt3D2Mat(std::vector<Point3f> pt3D) {
	// This function is to convert pt3D to Mat.
	// parameters:
	// pt3D is a Point3f vector
	// outputPt3D is a output mat

	Mat outputPt3D(Size(3, pt3D.size()), CV_32FC1);
	for (int i = 0; i < pt3D.size(); i++) {
		outputPt3D.at<float>(i, 0) = pt3D[i].x;
		outputPt3D.at<float>(i, 1) = pt3D[i].y;
		outputPt3D.at<float>(i, 2) = pt3D[i].z;
	}
	return outputPt3D;
}

Mat ocvi::ConvertPt2D2Mat(std::vector<Point2f> pt2D) {
// This function is to convert pt2D to Mat.
// parameters:
// pt2D is a Point3f vector
// outputPt2D is a output mat

	Mat outputPt2D(Size(3, pt2D.size()), CV_32FC1);
	for (int i = 0; i < pt2D.size(); i++) {
		outputPt2D.at<float>(i, 0) = pt2D[i].x;
		outputPt2D.at<float>(i, 1) = pt2D[i].y;
	}
	return outputPt2D;
}

void ocvi::PoseEstFor3Dto3D(Mat pt3D1, Mat pt3D2, Mat &R, Mat &T){
	// This function is to estimate Pose according to 3D point to 3D point.
	// parameters:
	// pt3D1 is a n*3 matrix for 3D point
	// pt3D2 is a n*3 matrix for 3D point (each pair of pt3D1 and pt3D2 is matched.)
	// R is the rotation from pt3D1 to pt3D2
	// T is the translation from pt3D1 to pt3D2

	Mat pt1MassCenter, pt2MassCenter;
	reduce(pt3D1, pt1MassCenter, 0, cv::ReduceTypes::REDUCE_AVG);
	reduce(pt3D2, pt2MassCenter, 0, cv::ReduceTypes::REDUCE_AVG);



	Mat zeroDriftPt1 = pt3D1.clone();
	for (int i = 0; i < pt3D1.rows; i++) {
		zeroDriftPt1.row(i) = pt3D1.row(i) - pt1MassCenter;
	}

	Mat zeroDriftPt2 = pt3D2.clone();
	for (int i = 0; i < pt3D2.rows; i++) {
		zeroDriftPt2.row(i) = pt3D2.row(i) - pt2MassCenter;
	}

	Mat S, U, V;
	SVD::compute(zeroDriftPt1.t()*zeroDriftPt2, S, U, V, 0);
	S = Mat::diag(S);
	Mat H = U * S * V.t();
	Mat X = V * U.t();
	if (abs(determinant(X) + 1) < 0.0001) {
		V.col(2) = -V.col(2);
		R = V * U.t();
	}
	else {
		R = X;
	}
	T = pt2MassCenter.t() - R * pt1MassCenter.t();
}

void ocvi::TriangulatePoints(Mat intriMatL, Mat intriMatR, Mat R, Mat T,
	Mat pt2DL, Mat pt2DR, Mat &pt3D) {
	// This function is to use trangulatePoint in OCV to calculate pt3D.
	// parameters:
	// intriMatL is a 3*3 matrix for camera intrinsic
	// intriMatR is a 3*3 matrix for camera intrinsic
	// R is the 3*3 rotation matrix from pt3D1 to pt3D2
	// T is the 3*1 translation from pt3D1 to pt3D2
	// pt2DL is a n*2 2D points in left Image Coordinate system 
	// pt2DR is a n*2 2D points in right Image Coordinate system 
	// pt3D is a n*3 3D points in left camera coordinate system

	Mat projMatrixL = intriMatL*Mat::eye(3, 4, CV_32FC1);
	Mat projMatrixR;
	hconcat(R, T, projMatrixR);
	projMatrixR = intriMatR*projMatrixR;

	/*
	// orb trangulate points
	cv::Mat A(4, 4, CV_32F);
	vector<Point3f> point3DVec;
	Mat temp(pt2DL.rows, 3, CV_32F);
	
	for (int i = 0; i < pt2DL.rows; i++) {
		A.row(0) = pt2DL.at<float>(i, 0)*projMatrixL.row(2) - projMatrixL.row(0);

		A.row(1) = pt2DL.at<float>(i, 1)*projMatrixL.row(2) - projMatrixL.row(1);

		A.row(2) = pt2DR.at<float>(i, 0)*projMatrixR.row(2) - projMatrixR.row(0);

		A.row(3) = pt2DR.at<float>(i, 1)*projMatrixR.row(2) - projMatrixR.row(1);

		cout << A << endl;

		cv::Mat u, w, vt;

		cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

		Mat x3D = vt.row(3).t();
		
		cout << x3D.rowRange(0, 3) << endl;
		cout << x3D.at<float>(3) << endl;

		x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);

		cout << x3D << endl;
		temp.row(i) = x3D.t();
		cout << temp.row(i) << endl;
	}
	pt3D = temp.clone();
	*/

	
	// opencv method
	Mat point4DMat;
	triangulatePoints(projMatrixL, projMatrixR, pt2DL, pt2DR, point4DMat);
  std::vector<Point3f> point3DVec;
	convertPointsFromHomogeneous(point4DMat.t(), point3DVec);
	pt3D = ocvi::ConvertPt3D2Mat(point3DVec);
	
}

void ocvi::NormInRows(Mat m1, Mat m2, std::vector<float> &dist) {
	// This function is to calculate the distance between two n*2 or n*3 matrix, which will output an n*1 diatance list .
	// parameters:
	// m1 is a n*2 or n*3 matrix
	// m2 is a n*2 or n*3 matrix
	// dist is an n*1 diatance vector
	dist.clear();
	Mat temp = m1 - m2;
	for (int i = 0; i < temp.rows; i++) {
		dist.push_back(norm(temp.row(i)));
	}
}

void ocvi::FindHomoGraphyInStereo(Mat intriMatL, Mat intriMatR, Mat R, Mat T,
	Mat pt2DL, Mat pt2DR, Mat pt3D, Mat &H) {

}

void ocvi::FindHomoGraphy(Mat pt2DL, Mat pt2DR, Mat &H) {
	H = findHomography(pt2DL, pt2DR, cv::RANSAC);

	
}

void ocvi::Reproject2Dto2D(Mat pt2D, Mat H, Mat &outputPt2D) {
	Mat nonHomoPt2D(pt2D.rows, 3, CV_32F);
	H.convertTo(H, CV_32F);
	for (int i = 0; i < pt2D.rows; i++) {
		Mat temp(3, 1, CV_32F);
		temp.at<float>(0) = pt2D.at<float>(i, 0);
		temp.at<float>(1) = pt2D.at<float>(i, 1);
		temp.at<float>(2) = 1;
		//cout << temp << endl;
		//cout << H << endl;
		Mat temp0 = H * temp;
		//cout << temp0 << endl;
		nonHomoPt2D.row(i) = temp0.t();
	}

  std::vector<Point2f> point2D;
	convertPointsFromHomogeneous(nonHomoPt2D, point2D);
	outputPt2D = Mat(point2D).clone();
}
