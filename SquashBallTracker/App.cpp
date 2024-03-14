#include <iostream>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>

#include "App.hpp"

void App::Init(std::string& videoPath)
{
	m_ImProc.Init(videoPath);

	std::cout << "App initalised" << std::endl;
}

void App::Run()
{
	m_ImProc.UploadAndBlur();

	m_ImProc.AmplifyDifference(m_BallProcessedG, 5);
	m_ImProc.AmplifyDifference(m_PlayerProcessedG, 45);

	m_BallProcessedG.download(m_BallProcessedC);
	m_PlayerProcessedG.download(m_PlayerProcessedC);

	std::vector<cv::Rect> ballRects;

	m_objDetector.FindObjects(m_BallProcessedC, ballRects, 50, 500);
	m_objDetector.GroupNearObjects(ballRects, 1);
	m_objDetector.TrackObjects(ballRects);

	std::vector<cv::Point> bestPath;
	m_objDetector.GetBestPath(bestPath);

	cv::Mat frame;
	m_ImProc.GetFrame(frame);
	
	for (size_t i = 0; i < bestPath.size(); i++)
	{
		cv::circle(frame, { bestPath[i].x + 50, bestPath[i].y }, 2, cv::Scalar(100), -1);
		if (i < bestPath.size() - 1)
			cv::line(frame, { bestPath[i].x + 50, bestPath[i].y }, { bestPath[i + 1].x + 50, bestPath[i + 1].y }, cv::Scalar(0, 100, 0), 1);
	}

	cv::imshow("Balls", frame);

	//std::cout << "App running" << std::endl;
}

