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
	//����ͼƬ����������ȷ���ָ�С���ڵ���̬
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
	//����Сͼ��ߴ磬��϶���߽�
	int nShowImageSize = 200;
	int nSplitLineSize = 15;
	int nAroundLineSize = 50;
	//�������ͼ��ͼ���С��������Դ��ȷ��
	const int imagesHeight = nShowImageSize *
		nSizeWindows.width + nAroundLineSize +
		(nSizeWindows.width - 1)*nSplitLineSize;
	const int imagesWidth = nShowImageSize *
		nSizeWindows.height + nAroundLineSize +
		(nSizeWindows.height - 1)*nSplitLineSize;
	cout << imagesWidth << "  " << imagesHeight << endl;
	Mat showWindowsImages(imagesWidth, imagesHeight, CV_8UC3, Scalar(0, 0, 0));
	//��ȡ��ӦСͼ������Ͻ�����x��y
	int posX = (showWindowsImages.cols - (nShowImageSize*nSizeWindows.width +
		(nSizeWindows.width - 1)*nSplitLineSize)) / 2;
	int posY = (showWindowsImages.rows - (nShowImageSize*nSizeWindows.height +
		(nSizeWindows.height - 1)*nSplitLineSize)) / 2;
	cout << posX << "  " << posY << endl;
	int tempPosX = posX;
	int tempPosY = posY;
	//��ÿһ��Сͼ�����ϳ�һ����ͼ��
	for (int i = 0; i < nNumImages; i++) {
		//Сͼ������ת��
		if ((i%nSizeWindows.width == 0) && (tempPosX != posX)) {
			tempPosX = posX;;
			tempPosY += (nSplitLineSize + nShowImageSize);
		}
		//����Rect����Сͼ�����ڴ�ͼ�����Ӧ����
		Mat tempImage = showWindowsImages
		(Rect(tempPosX, tempPosY, nShowImageSize, nShowImageSize));
		//����resize����ʵ��ͼ������
		resize(srcImages[i], tempImage,
			Size(nShowImageSize, nShowImageSize));
		tempPosX += (nSplitLineSize + nShowImageSize);
	}
	imshow("showWindowImages", showWindowsImages);
}

