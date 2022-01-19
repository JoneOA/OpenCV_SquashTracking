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

std::vector<SBTracker::TrackedObj> SBTracker::distanceTracker(std::vector<cv::Rect> detectedObjects) {
	
	newObjects.clear();

	for (cv::Rect detObj : detectedObjects) {

		bool newObject = true;
		direction d;

		TrackedObj t = { detObj, identifier, 0, d, newObject };

		float cenX = detObj.x + (detObj.width / 2);
		float cenY = detObj.y + (detObj.height / 2);

		for (int index = 0; index < classifiedObjects.size(); index++) {
			
			float pCenX = classifiedObjects[index].position.x + (classifiedObjects[index].position.width / 2);
			float pCenY = classifiedObjects[index].position.y + (classifiedObjects[index].position.height / 2);
			
			d.x = pCenX - cenX;
			d.y = pCenY - cenY;

			t.dir = d;

			double dis = hypot(t.dir.x, t.dir.y);

			if (dis < 50) 
			{
				newObject = false;
				TrackedObj temp = { detObj, classifiedObjects[index].id, dis, t.dir, newObject };
				newObjects.push_back(temp);
				classifiedObjects.erase(classifiedObjects.begin() + index);
				std::cout << "ID: " << temp.id << " Dis: " << dis << " Direction: [" << t.dir.x << ", " << t.dir.y << "]\n";
				index--;
				break;
			}
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