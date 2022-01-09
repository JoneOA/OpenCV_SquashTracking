#include <iostream>
#include "Tracker.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>


using namespace sbt;

std::vector<SBTracker::TrackedObj> SBTracker::distanceTracker(std::vector<cv::Rect> detectedObjects) {
	
	newObjects.clear();

	for (cv::Rect detObj : detectedObjects) {

		TrackedObj t = { detObj, identifier };

		bool sameObj = false;


		int cenX = detObj.x + (detObj.width / 2);
		int cenY = detObj.y + (detObj.height / 2);

		for (int index = 0; index < classifiedObjects.size(); index++) {
			int pCenX = classifiedObjects[index].position.x + (classifiedObjects[index].position.width / 2);
			int pCenY = classifiedObjects[index].position.y + (classifiedObjects[index].position.height / 2);

			double dis = hypot(cenX - pCenX, cenY - pCenY);

			if (dis < 200) 
			{
				TrackedObj temp = { detObj, classifiedObjects[index].id };
				newObjects.push_back(temp);
				sameObj = true;
				break;
			}
		}

		if( !sameObj ){
			newObjects.push_back(t);
			identifier++;
		}
	}

	tempObj.clear();

	for (int i = 0; i < newObjects.size(); i++) {
		int eyeDee = newObjects[i].id;
		cv::Rect rectan;
		if (classifiedObjects.size() != 0) {
			for (int j = 0; j < classifiedObjects.size(); j++) {
				if (classifiedObjects[j].id == eyeDee) {
					rectan = classifiedObjects[j].position;
					tempObj.push_back({ rectan, eyeDee });
					break;
				}
			}
		}
		else {
			rectan = newObjects[i].position;
			tempObj.push_back({ rectan, eyeDee });
		}

	}
	classifiedObjects = tempObj;

	return classifiedObjects;
}

SBTracker::TrackedObj SBTracker::createTrackedObject(cv::Rect TrackedObject) {
	return{ TrackedObject, identifier };
}

	


