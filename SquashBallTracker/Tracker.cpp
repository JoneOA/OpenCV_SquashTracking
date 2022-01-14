#include <iostream>
#include "Tracker.hpp"
#include "TrajectoryPreditor.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace sbt;

std::vector<SBTracker::TrackedObj> SBTracker::distanceTracker(std::vector<cv::Rect> detectedObjects) {
	newObjects.clear();

	for (int indexOfObject = 0; indexOfObject < detectedObjects.size(); indexOfObject++) {
		cv::Rect currentObj = detectedObjects[indexOfObject];

		std::vector<int> pTO; //Potential Tracking Objects

		//Setting the features of object passed in from main.cpp
		float xPosD = currentObj.x + (currentObj.width / 2); //The x position at the centre of the detected object
		float yPosD = currentObj.y + (currentObj.height / 2); //The y position at the centre of the detected object
		cv::Point position(xPosD, yPosD);

		//Initalising the prediction of where the classified object could be, to check against the position of the detected objects that have been passed in
		cv::Point nextPos;

		//Variables that need setting to return to main.cpp
		int closestObjId = -1;
		float closestObjectDistance = 50;
		float closestObjXDir;
		float closestObjYDir;

		//Looping over all of the already classified objects from previous frames that have been passed in
		for (int iOfClassified = 0; iOfClassified < classifiedObjects.size(); iOfClassified++) {
			SBTracker::TrackedObj curClassObj = classifiedObjects[iOfClassified];

			//Setting the features of the already classified object detected in previous iterations
			float xPosC = curClassObj.positions.at(curClassObj.positions.size() - 1).x; //The x position at the center of the already classified object
			float yPosC = curClassObj.positions.at(curClassObj.positions.size() - 1).y; //The y position at the centre of the alreadt classified object

			//Calulating the euclidian distance between the classified object and the deteced object
			float distanceChange = hypot(xPosD - xPosC, yPosD - yPosC);

			int uncertainty = 10; //Defines the area around the "nextPoint" to search for an object

			/*
			If the current object doesn't have a long enough history to predict a path then any object in the new frame could be it. The possible object are set and the 
			path between them are used to predict a point where their hypothetical path may lead to. If this is close to the detected object then they are taken as the 
			same object.
			*/
			if (curClassObj.positions.size() < 2 && !curClassObj.possibleTrackedObjs.empty()) {

				uncertainty = 30;
				//std::cout << "ObjID: " << curClassObj.id << " IDs: ";
				for (int i = 0; i < curClassObj.possibleTrackedObjs.size(); i++) {
					
					//std::cout << curClassObj.possibleTrackedObjs[i] << ", ";

					int possibleObjId = curClassObj.possibleTrackedObjs[i];
					cv::Point possibleObjPosition;
					
					std::vector<cv::Point> constructedHistory;

					for (int j = 0; j < classifiedObjects.size(); j++) {
						if (classifiedObjects[i].id == possibleObjId) {
							possibleObjPosition = classifiedObjects[i].positions[classifiedObjects[i].positions.size() - 1];
						}
					}

					constructedHistory.push_back(curClassObj.positions[0]);
					constructedHistory.push_back(possibleObjPosition);
					nextPos = tpd::TrajectoryPredictor::nextPosition(constructedHistory);
					//potential matches are being set, figure out what is happening with them

					if (position.x - nextPos.x < uncertainty && position.x - position.x > -uncertainty && distanceChange < closestObjectDistance) {
						if (position.y - nextPos.y < uncertainty && position.y - position.y > -uncertainty) {
							closestObjId = iOfClassified;
							closestObjectDistance = distanceChange;
						}
					}
				}
				//std::cout << "\n";
			}
			//Finding the possible next point if the object has as history of point
			else if (curClassObj.positions.size() >= 2) {
				nextPos = tpd::TrajectoryPredictor::nextPosition(curClassObj.positions);
				if (curClassObj.positions.size() == 2) uncertainty = 20;
				//Using the next predicted point to find the object which is closest to it
				if (position.x - nextPos.x < uncertainty && position.x - position.x > -uncertainty && distanceChange < closestObjectDistance) {
					if (position.y - nextPos.y < uncertainty && position.y - position.y > -uncertainty) {
						closestObjId = iOfClassified;
						closestObjectDistance = distanceChange;
					}
				}
			}
			else {
				if (distanceChange < 50) {
					pTO.push_back(curClassObj.id);
				}
			}
		
		}
		//If an object has been found then it is added to the list of new objects and removed from the persisting list. It will be readded later on
		if (closestObjId >= 0) {
			classifiedObjects[closestObjId].positions.push_back(nextPos);
			SBTracker::TrackedObj tempObj = { classifiedObjects[closestObjId].positions, classifiedObjects[closestObjId].id, currentObj.width, currentObj.height, 0, std::vector<int>() };
			newObjects.push_back(tempObj);
			classifiedObjects.erase(classifiedObjects.begin() + closestObjId);
		}
		//If the detected object has no suitable match then it is added as a new object with the IDs of the possible objects appended
		else {
			std::vector<cv::Point> tp;
			tp.push_back(position);
			SBTracker::TrackedObj tempObj = { tp, identifier, currentObj.width, currentObj.height, 0, pTO };
			newObjects.push_back(tempObj);
			identifier++;
		}
	}
	for (int i = 0; i < classifiedObjects.size(); i++) {
		classifiedObjects[i].framesMissing++;
	}

	classifiedObjects.insert(classifiedObjects.end(), newObjects.begin(), newObjects.end());

	return classifiedObjects;
}