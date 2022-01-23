#include <iostream>
#include "Tracker.hpp"
#include "TrajectoryPreditor.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace sbt;

int sbt::SBTracker::findNextLink(int objectId, int j)
{
	int ballId = -1;
	if (j < linkedDetections.size()) {
		for (int k = 0; k < linkedDetections[j][objectId].size(); k++) {
			objectPath.push_back(linkedDetections[j][objectId][k]);
			linkedObjects.push_back(detectionHistory[j + 1][linkedDetections[j][objectId][k]]);
			ballId = findNextLink(linkedDetections[j][objectId][k], j + 1);
			objectPath.pop_back();
			linkedObjects.pop_back();
		}
		//std::cout << "Path: ";
		//for (int a = 0; a < objectPath.size(); a++) {
		//	std::cout << objectPath[a] << ", ";
		//}
		//std::cout << "\n";
		if (tpd::TrajectoryPredictor::nextPosition(linkedObjects)) {
			fullGraph.push_back(linkedObjects);
			ballId = objectId;
			//std::cout << ballId << "\n";
		}
	}
	return ballId;
}

//OPTIMISATION

void sbt::SBTracker::PathSearch(int& ballId)
{
	cv::Rect currentObject;
	cv::Rect nextObject;
	int j = 0;
	for (int k = 0; k < linkedDetections[j].size(); k++) {
		objectPath.push_back(k);
		linkedObjects.push_back(detectionHistory[j][k]);
		for (int l = 0; l < linkedDetections[j][k].size(); l++) {
			objectPath.push_back(linkedDetections[j][k][l]);
			linkedObjects.push_back(detectionHistory[j + 1][linkedDetections[j][k][l]]);
			ballId = findNextLink(linkedDetections[j][k][l], j + 1);
			objectPath.pop_back();
			linkedObjects.pop_back();
		}
		objectPath.pop_back();
		linkedObjects.pop_back();
	}
}

std::vector<std::vector<cv::Rect>> SBTracker::distanceTracker(std::vector<cv::Rect> detectedObjects) {
	fullGraph.clear();
	int lengthOfHistory = detectionHistory.size() - 1;
	int ballId = -1;
	std::vector<std::vector<int>> closeObjectList;
	if (lengthOfHistory >= 0) {
		for (int i = 0; i < detectionHistory[lengthOfHistory].size(); i++) {
			std::vector<int> closeObjects;
			for (int j = 0; j < detectedObjects.size(); j++) {
				cv::Rect detObj = detectedObjects[j];
				if (hypot(detObj.x - detectionHistory[lengthOfHistory][i].x, detObj.y - detectionHistory[lengthOfHistory][i].y) < 300) {
					closeObjects.push_back(j);
				}
			}
			closeObjectList.push_back(closeObjects);
		}
		linkedDetections.push_back(closeObjectList);
		if (linkedDetections.size() > 4) {
			linkedDetections.erase(linkedDetections.begin());
			detectionHistory.erase(detectionHistory.begin());
		}
	}

	detectionHistory.push_back(detectedObjects);

	if (linkedDetections.size() > 2) {
		PathSearch(ballId);
	}
	return fullGraph;
}