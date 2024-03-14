#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <unordered_map>
#include "ProjectileMaths.hpp"

constexpr int HISTORY_SIZE = 5;

class ObjectDetector
{
private:
	//Maps an ID to a specific object
	std::unordered_map<int, cv::Point> m_objectIDs;

	//Mapping objects to other close objects
	std::unordered_map<int, std::vector<int>> m_linkedObjects;
	
	//Storing which objects were found in previous frames
	std::vector<std::vector<int>> m_objectHistory;

	//Current search path for ball
	std::vector<int> m_currentPath;

	std::vector<int> m_bestPath;
	double m_lowestDelta;

	ProjectileMaths m_pM;

	int m_nextFreeId = 0;

public:
	void FindObjects(cv::Mat& proccessedImage, std::vector<cv::Rect>& foundRectangles, int minSize, int maxSize);
	void GroupNearObjects(std::vector<cv::Rect>& foundRectangles, float groupSize);
	cv::Rect increaseRectSize(cv::Rect& boundingBox, int groupSize);
	void TrackObjects(std::vector<cv::Rect>& foundRectangles);
	void ClassifyObjects(std::vector<cv::Rect>& foundRectangles);
	void DistanceCheck(std::vector<cv::Rect>& foundRectangles);
	void PathCheck();
	void FindNextObject(int ID);
	void GetBestPath(std::vector<cv::Point>& path);

	void FollowsProjectile();
};