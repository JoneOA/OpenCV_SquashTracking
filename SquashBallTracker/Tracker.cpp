#include <iostream>
#include "Tracker.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace sbt;

std::vector<SBTracker::TrackedObj> SBTracker::distanceTracker(std::vector<cv::Rect> detectedObjects) {
	
	newObjects.clear();

	for (cv::Rect detObj : detectedObjects) {

		bool isNewObject = true;
		direction direction;

		TrackedObj t = { detObj, identifier, 0, direction, isNewObject };

		float cenX = detObj.x + (detObj.width / 2);
		float cenY = detObj.y + (detObj.height / 2);

		for (int index = 0; index < classifiedObjects.size(); index++) {

			float pCenX = classifiedObjects[index].position.x + (classifiedObjects[index].position.width / 2);
			float pCenY = classifiedObjects[index].position.y + (classifiedObjects[index].position.height / 2);

			direction.x = pCenX - cenX;
			direction.y = pCenY - cenY;
			t.dir = direction;

			double dis = hypot(t.dir.x, t.dir.y);

			if (dis < 50)
			{
				isNewObject = false;
				TrackedObj temp = { detObj, classifiedObjects[index].id, dis, t.dir, isNewObject };
				newObjects.push_back(temp);
				classifiedObjects.erase(classifiedObjects.begin() + index);
				index--;
				break;
			}
		}
		if( isNewObject ){
			newObjects.push_back(t);
			identifier++;
		}
	}
	classifiedObjects = newObjects;
	return classifiedObjects;
}

	


