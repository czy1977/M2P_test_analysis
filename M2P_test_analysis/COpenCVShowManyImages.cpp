#include "pch.h"
#include "COpenCVShowManyImages.h"

using namespace std;
using namespace cv;

COpenCVShowManyImages::COpenCVShowManyImages()
{
}


COpenCVShowManyImages::~COpenCVShowManyImages()
{
}

void COpenCVShowManyImages::ShowManyImages(const std::vector<cv::Mat>&srcImages, cv::Size imageSize) {
	int nNumImages = srcImages.size();
	Size nSizeWindows;
	if (nNumImages > 12) {
		cout << "no more tha 12 images" << endl;
		return;
	}
	//根据图片序列数量来确定分割小窗口的形态
	switch (nNumImages) {
	case 1:nSizeWindows = Size(1, 1); break;
	case 2:nSizeWindows = Size(2, 1); break;
	case 3:
	case 4:nSizeWindows = Size(2, 2); break;
	case 5:
	case 6:nSizeWindows = Size(3, 2); break;
	case 7:
	case 8:nSizeWindows = Size(4, 2); break;
	case 9:nSizeWindows = Size(3, 3); break;
	default:nSizeWindows = Size(4, 3);
	}
	//设置小图像尺寸，间隙，边界
	int nShowImageSize = 200;
	int nSplitLineSize = 15;
	int nAroundLineSize = 50;
	//创建输出图像，图像大小根据输入源来确定
	const int imagesHeight = nShowImageSize *
		nSizeWindows.width + nAroundLineSize +
		(nSizeWindows.width - 1)*nSplitLineSize;
	const int imagesWidth = nShowImageSize *
		nSizeWindows.height + nAroundLineSize +
		(nSizeWindows.height - 1)*nSplitLineSize;
	cout << imagesWidth << "  " << imagesHeight << endl;
	Mat showWindowsImages(imagesWidth, imagesHeight, CV_8UC3, Scalar(0, 0, 0));
	//提取对应小图像的左上角坐标x，y
	int posX = (showWindowsImages.cols - (nShowImageSize*nSizeWindows.width +
		(nSizeWindows.width - 1)*nSplitLineSize)) / 2;
	int posY = (showWindowsImages.rows - (nShowImageSize*nSizeWindows.height +
		(nSizeWindows.height - 1)*nSplitLineSize)) / 2;
	cout << posX << "  " << posY << endl;
	int tempPosX = posX;
	int tempPosY = posY;
	//将每一幅小图像整合成一幅大图像
	for (int i = 0; i < nNumImages; i++) {
		//小图像坐标转换
		if ((i%nSizeWindows.width == 0) && (tempPosX != posX)) {
			tempPosX = posX;;
			tempPosY += (nSplitLineSize + nShowImageSize);
		}
		//利用Rect区域将小图像置于大图像的相应区域
		Mat tempImage = showWindowsImages
		(Rect(tempPosX, tempPosY, nShowImageSize, nShowImageSize));
		//利用resize函数实现图像缩放
		resize(srcImages[i], tempImage,
			Size(nShowImageSize, nShowImageSize));
		tempPosX += (nSplitLineSize + nShowImageSize);
	}
	imshow("showWindowImages", showWindowsImages);
}

