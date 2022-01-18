#include <iostream>
#include "Tracker.hpp"
#include "TrajectoryPreditor.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace sbt;
bool sbt::SBTracker::isWithinBounds(cv::Point detectedObj, cv::Point ClassifiedObj, int uncertainty)
{
	if (detectedObj.x - ClassifiedObj.x < uncertainty && detectedObj.x - ClassifiedObj.x > -uncertainty) {
		if (detectedObj.y - ClassifiedObj.y < uncertainty && detectedObj.y - ClassifiedObj.y > -uncertainty) {
			return true;
		}
	}
	return false;
}

cv::Rect sbt::SBTracker::findNextLink(int objectId, int a)
{
	cv::Rect currentObject;
	cv::Rect nextObject;
	if (a < linkedDetections.size()) {
		for (int l = 0; l < linkedDetections[a][objectId].size(); l++) {
			findNextLink(linkedDetections[a][objectId][l], a + 1);
		}
	}
	return nextObject;
}

std::vector<SBTracker::TrackedObj> SBTracker::distanceTracker(std::vector<cv::Rect> detectedObjects) {
	int lengthOfHistory = detectionHistory.size() - 1;
	std::vector<std::vector<int>> closeObjectList;
	if (lengthOfHistory >= 0) {
		for (int i = 0; i < detectionHistory[lengthOfHistory].size(); i++) {
			std::vector<int> closeObjects;
			for (int j = 0; j < detectedObjects.size(); j++) {
				cv::Rect detObj = detectedObjects[j];
				if (hypot(detObj.x - detectionHistory[lengthOfHistory][i].x, detObj.y - detectionHistory[lengthOfHistory][i].y) < 50) {
					closeObjects.push_back(j);
				}
			}
			closeObjectList.push_back(closeObjects);
		}
		linkedDetections.push_back(closeObjectList);
	}

	detectionHistory.push_back(detectedObjects);

	if (linkedDetections.size() > 2) {
		cv::Rect currentObject;
		cv::Rect nextObject;

		for (int j = 0; j < linkedDetections.size(); j++) {
			std::cout << "-----------------------------------------------------------------------------\n Frame " << j << "\n";
			for (int k = 0; k < linkedDetections[j].size(); k++) {
				std::cout << "Object: " << k << " - ";
				for (int l = 0; l < linkedDetections[j][k].size(); l++) {
					
					findNextLink(linkedDetections[j][k][l], j + 1);

				}
				std::cout << "\b\b\n";
			}
		}
	}
	return newObjects;
} 