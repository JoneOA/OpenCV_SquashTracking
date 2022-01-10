#include <iostream>
#include "Tracker.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace sbt;

std::vector<SBTracker::TrackedObj> SBTracker::distanceTracker(std::vector<cv::Rect> detectedObjects) {
	
	newObjects.clear();

	for (cv::Rect detObj : detectedObjects) {

		TrackedObj t = { detObj, identifier, 0};

		bool sameObj = false;

		int cenX = detObj.x + (detObj.width / 2);
		int cenY = detObj.y + (detObj.height / 2);

		for (int index = 0; index < classifiedObjects.size(); index++) {
			int pCenX = classifiedObjects[index].position.x + (classifiedObjects[index].position.width / 2);
			int pCenY = classifiedObjects[index].position.y + (classifiedObjects[index].position.height / 2);

			double dis = abs(hypot(cenX - pCenX, cenY - pCenY));

			if (dis < 50) 
			{
				TrackedObj temp = { detObj, classifiedObjects[index].id, dis };
				newObjects.push_back(temp);
				classifiedObjects.erase(classifiedObjects.begin() + index);
				sameObj = true;
				index--;
				break;
			}
		}

		if( !sameObj ){
			newObjects.push_back(t);
			identifier++;
		}
	}

	classifiedObjects = newObjects;

	return classifiedObjects;
}

SBTracker::TrackedObj SBTracker::createTrackedObject(cv::Rect TrackedObject) {
	return{ TrackedObject, identifier };
}

	


