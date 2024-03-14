#pragma once

#include <opencv2/opencv.hpp>
#include <string>

#include "ImageProcessor.hpp"
#include "ObjectDetector.hpp"

class App
{
private:

	ImageProcessor m_ImProc;
	ObjectDetector m_objDetector;

	cv::cuda::GpuMat m_BallProcessedG, m_PlayerProcessedG;
	cv::Mat m_BallProcessedC, m_PlayerProcessedC;

	int m_FramesChecked;

public:
	void Init(std::string& videoPath);
	void Run();
};