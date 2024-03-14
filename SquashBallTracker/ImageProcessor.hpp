#pragma once
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/cudafilters.hpp>

class ImageProcessor
{
	cv::VideoCapture capture;

	cv::Ptr<cv::cuda::Filter> gausFilter = cv::cuda::createGaussianFilter(16, 16, cv::Size(3, 3), 0);

	cv::Mat frame1, frame2, frame3;

	cv::cuda::GpuMat delta1, delta2, delta3;
	cv::cuda::GpuMat gFrame1, gFrame2, gFrame3;
	cv::cuda::GpuMat ballProcessed, playerProcessed;

public:
	
	void Init(std::string& videoPath);
	void AmplifyDifference(cv::cuda::GpuMat& returnImage, int threshLower);
	void UploadAndBlur();
	void GetFrame(cv::Mat& mat) const;
};