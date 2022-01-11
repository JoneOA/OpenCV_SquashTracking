#include <iostream>
#include "Tracker.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace sbt;

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

		if( newObject ){
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

	


