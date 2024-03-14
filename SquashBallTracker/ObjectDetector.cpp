#include "ObjectDetector.hpp"
#include "ProjectileMaths.hpp"
#include <cmath>

void ObjectDetector::FindObjects(cv::Mat& proccessedImage, std::vector<cv::Rect>& objRect, int minSize, int maxSize)
{
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(proccessedImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	for (size_t i = 0; i < contours.size(); i++)
	{
		double area = cv::contourArea(contours[i]);
		if (area > minSize && area < maxSize)
			objRect.push_back(cv::boundingRect(contours[i]));
	}
}

void ObjectDetector::GroupNearObjects(std::vector<cv::Rect>& foundRectangles, float groupSize)
{
	for (int i = 0; i < foundRectangles.size(); i++) {
		cv::Rect obj1 = increaseRectSize(foundRectangles[i], groupSize);
		for (int j = i + 1; j < foundRectangles.size(); j++) {
			cv::Rect obj2 = increaseRectSize(foundRectangles[j], groupSize);
			if ((obj1 & obj2).area() != 0) {
				foundRectangles[i].x = MIN(obj1.x, obj2.x);
				foundRectangles[i].y = MIN(obj1.y, obj2.y);
				foundRectangles[i].width = obj1.br().x > obj2.br().x ? obj1.br().x - obj1.x : obj2.br().x - obj1.x;
				foundRectangles[i].height = obj1.br().y > obj2.br().y ? obj1.br().y - obj1.y : obj2.br().y - obj1.y;

				foundRectangles.erase(foundRectangles.begin() + j);
				j--;
			}
		}
	}
}

cv::Rect ObjectDetector::increaseRectSize(cv::Rect& boundingBox, int groupSize)
{
	cv::Rect newBox = boundingBox;
	newBox.x -= (boundingBox.width / 2) * groupSize;
	newBox.y -= (boundingBox.height / 2) * groupSize;
	newBox.width += boundingBox.width * groupSize;
	newBox.height += boundingBox.height * groupSize;

	return newBox;
}

void ObjectDetector::TrackObjects(std::vector<cv::Rect>& foundRectangles)
{
	m_lowestDelta = (double)INT_MAX;
	ClassifyObjects(foundRectangles);
	DistanceCheck(foundRectangles);
	PathCheck();
}

void ObjectDetector::ClassifyObjects(std::vector<cv::Rect>& foundRectangles)
{
	//Adding to history
	std::vector<int> objectsInFrame;
	for (size_t i = 0; i < foundRectangles.size(); i++)
	{
		cv::Rect& rect = foundRectangles[i];
		m_objectIDs[m_nextFreeId] = {rect.x + (rect.width / 2), rect.y + (rect.height / 2)};

		objectsInFrame.push_back(m_nextFreeId);
		m_nextFreeId++;
	}
	m_objectHistory.push_back(objectsInFrame);
	
	//Removing oldest from history
	if (m_objectHistory.size() > HISTORY_SIZE)
	{
		for (size_t i = 0; i < m_objectHistory[0].size(); i++)
		{
			m_objectIDs.erase(m_objectHistory[0][i]);
			m_linkedObjects.erase(m_objectHistory[0][i]);
		}
		m_objectHistory.erase(m_objectHistory.begin());
	}
}

void ObjectDetector::DistanceCheck(std::vector<cv::Rect>& foundRectangles)
{
	int lastFrame = m_objectHistory.size() - 1;

	if (lastFrame == 0)
		return;

	for (size_t i = 0; i < m_objectHistory[lastFrame - 1].size(); i++)
	{
		for (size_t j = 0; j < m_objectHistory[lastFrame].size(); j++)
		{
			int object1ID = m_objectHistory[lastFrame - 1][i];
			cv::Point& point1 = m_objectIDs[object1ID];
			
			int object2ID = m_objectHistory[lastFrame][j];
			cv::Point& point2 = m_objectIDs[object2ID];

			float distance = std::hypotf(point2.x - point1.x, point2.y - point1.y);

			if (distance < 300)
			{
				m_linkedObjects[object1ID].push_back(object2ID);
			}
		}
	}
}

void ObjectDetector::PathCheck()
{
	for (size_t i = 0; i < m_objectHistory[0].size(); i++)
	{
		int objectID = m_objectHistory[0][i];
		m_currentPath.push_back(objectID);
		for (size_t j = 0; j < m_linkedObjects[objectID].size(); j++)
		{
			m_currentPath.push_back(m_linkedObjects[objectID][j]);
			FindNextObject(m_linkedObjects[objectID][j]);
			m_currentPath.pop_back();
		}
		m_currentPath.pop_back();
	}
}

void ObjectDetector::FindNextObject(int ID)
{
	if (m_linkedObjects.count(ID) == 0)
	{
		FollowsProjectile();
		return;
	}

	for (size_t i = 0; i < m_linkedObjects[ID].size(); i++)
	{
		m_currentPath.push_back(m_linkedObjects[ID][i]);
		FindNextObject(m_linkedObjects[ID][i]);
		m_currentPath.pop_back();
	}
}

void ObjectDetector::GetBestPath(std::vector<cv::Point>& path)
{
	for (size_t i = 0; i < m_bestPath.size(); i++)
		path.push_back(m_objectIDs[m_bestPath[i]]);
}

void ObjectDetector::FollowsProjectile()
{
	std::vector<double> polynomialCoeff;
	std::vector<cv::Point> pointList;
	cv::Point nextDirection;
	float polyFit;
	float delta = 0;

	if (m_currentPath.size() >= 4)
	{
		for (int i = 0; i < m_currentPath.size(); i++)
			pointList.push_back(m_objectIDs[m_currentPath[i]]);
		//polynomialCoeff = m_pM.CalculateCoefficients(pointList);
	}
	else 
		return;

	//TODO: Check that points are along the arc in the correct order!
	if (m_currentPath.size() >= 4) {
		polynomialCoeff = m_pM.CalculateCoefficients(pointList);
		for (int i = 0; i < m_currentPath.size(); i++) {
			int positionID = m_currentPath[i];
			polyFit = (polynomialCoeff[2] * powf(m_objectIDs[positionID].x, 2)) + (m_objectIDs[positionID].x * polynomialCoeff[1]) + polynomialCoeff[0];
			delta += abs(polyFit - m_objectIDs[positionID].y);
		}

		if (delta < m_lowestDelta) {
			m_bestPath = m_currentPath;
		}	
	}

	return;
}
