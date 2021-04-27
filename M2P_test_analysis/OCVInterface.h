#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>


namespace ocvi {
  void Reproject3Dto2D(cv::Mat pt3D, cv::Mat R, cv::Mat T, cv::Mat intriMat, cv::Mat &outputPt2D);

  void PoseEstFor3Dto3D(cv::Mat pt3D1, cv::Mat pt3D2, cv::Mat &R, cv::Mat &T);
  void TriangulatePoints(cv::Mat intriMatL, cv::Mat intriMatR, cv::Mat R, cv::Mat T,
    cv::Mat leftCorners, cv::Mat rightCorners, cv::Mat &pt3D);

  void PlotPts(cv::Mat src, cv::Mat pt2D, cv::Scalar color, float radius);
  cv::Mat ConvertPt3D2Mat(std::vector<cv::Point3f> pt3D);
  cv::Mat ConvertPt2D2Mat(std::vector<cv::Point2f> pt2D);
  void NormInRows(cv::Mat m1, cv::Mat m2, std::vector<float> &dist);
  void FindHomoGraphyInStereo(cv::Mat intriMatL, cv::Mat intriMatR, cv::Mat R, cv::Mat T,
    cv::Mat pt2DL, cv::Mat pt2DR, cv::Mat orgPt2D, cv::Mat &H);
  void FindHomoGraphy(cv::Mat pt2DL, cv::Mat pt2DR, cv::Mat &H);

  void Reproject2Dto2D(cv::Mat pt2D, cv::Mat H, cv::Mat &outputPt2D);
}
