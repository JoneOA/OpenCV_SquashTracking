#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>

#include "ImageProcessor.hpp"

void ImageProcessor::Init(std::string& videoPath)
{
	capture.open(videoPath);
}

void ImageProcessor::AmplifyDifference(cv::cuda::GpuMat& returnImage, int threshLower)
{
	cv::cuda::subtract(gFrame2, gFrame1, delta1);
	cv::cuda::subtract(gFrame3, gFrame2, delta2);
	cv::cuda::subtract(gFrame1, gFrame3, delta3);

	cv::cuda::threshold(delta1, delta1, threshLower, 255, cv::THRESH_BINARY);
	cv::cuda::threshold(delta2, delta2, threshLower, 255, cv::THRESH_BINARY);
	cv::cuda::threshold(delta3, delta3, threshLower, 255, cv::THRESH_BINARY);

	cv::cuda::add(delta1, delta2, returnImage);
	cv::cuda::add(delta3, returnImage, returnImage);

	cv::cuda::cvtColor(returnImage, returnImage, cv::COLOR_BGR2GRAY);
}

void ImageProcessor::UploadAndBlur()
{
	capture >> frame1;
	capture >> frame2;
	capture >> frame3;

	gFrame1.upload(frame1);
	gFrame1.adjustROI(0, -50, -50, -50);
	gFrame2.upload(frame2);
	gFrame2.adjustROI(0, -50, -50, -50);
	gFrame3.upload(frame3);
	gFrame3.adjustROI(0, -50, -50, -50);

	gausFilter->apply(gFrame1, gFrame1);
	gausFilter->apply(gFrame2, gFrame2);
	gausFilter->apply(gFrame3, gFrame3);
}

void ImageProcessor::GetFrame(cv::Mat& mat) const
{
	mat = frame1;
}