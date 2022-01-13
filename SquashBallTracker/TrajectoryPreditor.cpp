#include <iostream>

#include <opencv2/opencv.hpp>
#include "Tracker.hpp"
#include "TrajectoryPreditor.hpp"

cv::Point tpd::TrajectoryPredictor::nextPosition(sbt::SBTracker::TrackedObj object)
{
	//TODO: PLAN WHAT THIS IS SUPPOSED TO BE. THINK ABOUT IT MAN

	bool isFollowingArc = true;
	std::vector<cv::Point> posHistory = object.positions;
	std::vector<cv::Point> pointDiff;
	cv::Point rateOfChange;
	cv::Point nextPoint;

	if (posHistory.size() >= 3) {
		for (int i = 1; i < posHistory.size(); i++) {
			int changeX = posHistory[i].x - posHistory[i - 1].x;
			int changeY = posHistory[i].y - posHistory[i - 1].y;
			pointDiff.push_back({ changeX, changeY });
		}
		if (pointDiff.size() >= 2) {
			int tempXDif = pointDiff.at(pointDiff.size() - 1).x - pointDiff.at(pointDiff.size() - 2).x;
			int tempYDif = pointDiff.at(pointDiff.size() - 1).y - pointDiff.at(pointDiff.size() - 2).y;

			for (int j = 3; j < pointDiff.size(); j++) {
				int xCheck = pointDiff.at(pointDiff.size() - j).x - pointDiff.at(pointDiff.size() - (j + 1)).x;
				int yCheck = pointDiff.at(pointDiff.size() - j).y - pointDiff.at(pointDiff.size() - (j + 1)).y;

				if (tempXDif - xCheck > 5 && tempXDif - xCheck < -5) {
					if (tempYDif - yCheck > 5 && tempYDif - yCheck < -5) {
						return cv::Point(-1, -1);
					}
				}

				tempXDif = xCheck;
				tempYDif = yCheck;

				rateOfChange = { xCheck, yCheck };
			}
		}
	}

	nextPoint = posHistory[posHistory.size() - 1]+ rateOfChange;

	return nextPoint;
}