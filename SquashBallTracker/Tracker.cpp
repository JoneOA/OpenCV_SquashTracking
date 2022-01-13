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

		float xPosD = currentObj.x + (currentObj.width / 2); //The x position at the centre of the detected object
		float yPosD = currentObj.y + (currentObj.height / 2); //The y position at the centre of the detected object

		cv::Point position(xPosD, yPosD);
		cv::Point nextPos;

		int framesMissing = 0;
		bool newObj = true;
		bool isBall = false;

		//TODO: Each object should temporarily store all the possible objects in the new frame that could be them. 
		//----> This could either be done by storing the possible objects onto the object or by storing them temporarily on a vector in this class.
		//----> It could be done in real time with the right conditions but for the time being do it separatly. Make it easy for yourself fool

		for (int indexOfClassified = 0; indexOfClassified < classifiedObjects.size(); indexOfClassified++) {
			SBTracker::TrackedObj curClassObj = classifiedObjects[indexOfClassified];

			float xPosC = curClassObj.positions.at(curClassObj.positions.size() - 1).x; //The x position at the center of the already classified object
			float yPosC = curClassObj.positions.at(curClassObj.positions.size() - 1).y; //The y position at the centre of the alreadt classified object

			float distanceChange = hypot(xPosD - xPosC, yPosD - yPosC);
			float xDir = xPosD - xPosC;
			float yDir = yPosD - yPosC;

			float xDirChange = curClassObj.xDir - xDir;
			float yDirChange = curClassObj.yDir - yDir;
			if (curClassObj.positions.size() >= 2){

				nextPos = tpd::TrajectoryPredictor::nextPosition(curClassObj);

				newObj = false;

				if(nextPos.x - position.x < 25 && nextPos.x - position.x > 25) {
					if (nextPos.y - position.y < 25 && nextPos.y - position.y > 25) {
						std::cout << "--=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=--=- 1 \n";
						framesMissing = 0;
						curClassObj.positions.push_back(position);
						newObjects.push_back({ curClassObj.positions, curClassObj.id, xDir, yDir, currentObj.width, currentObj.height, framesMissing });
						classifiedObjects.erase(classifiedObjects.begin() + indexOfClassified);
						indexOfClassified--;
						break;
					}
				}
			}
			else {
				if (xPosD < xPosC + curClassObj.xDir + 50 && xPosD > xPosC + curClassObj.xDir - 50) {
					if (yPosD < yPosC + curClassObj.yDir + 50 && xPosD > yPosC + curClassObj.yDir - 50) {
						std::cout << "--=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=--=- 2 \n";
						framesMissing = 0;
						curClassObj.positions.push_back(position);
						newObjects.push_back({ curClassObj.positions, curClassObj.id, xDir, yDir, currentObj.width, currentObj.height, framesMissing });
						classifiedObjects.erase(classifiedObjects.begin() + indexOfClassified);
						indexOfClassified--;
						break;
					}
				}
			}
		}
		if ( newObj ) {
			std::vector<cv::Point> positions;
			positions.push_back(position);
			newObjects.push_back({ positions, identifier, 0, 0, currentObj.width, currentObj.height, 0 });
			identifier++;
		}
	}

	for (int i = 0; i < classifiedObjects.size(); i++) {
		classifiedObjects[i].framesMissing++;
	}

	classifiedObjects.insert(classifiedObjects.end(), newObjects.begin(), newObjects.end());

	return classifiedObjects;
}