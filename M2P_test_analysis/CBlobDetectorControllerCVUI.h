#pragma once
class CBlobDetectorControllerCVUI
{
protected:
	std::string windowName;
	bool needUpdate;
	cv::Mat frame;
	void save();
	void load();
	std::string getConfigFileName();
	static bool isInit;


public:
	CBlobDetectorControllerCVUI();
	virtual ~CBlobDetectorControllerCVUI();
	void open(std::shared_ptr<cv::SimpleBlobDetector::Params> parameters, std::string name, bool isLoadFromFile = true);

	

	std::shared_ptr<cv::SimpleBlobDetector::Params> targetParameter;
	bool isDirty() { return needUpdate; }
	void loop();
};

