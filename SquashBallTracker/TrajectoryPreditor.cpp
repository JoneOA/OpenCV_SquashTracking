#include <iostream>

#include <opencv2/opencv.hpp>
#include "Tracker.hpp"
#include "TrajectoryPreditor.hpp"

cv::Point tpd::TrajectoryPredictor::nextPosition(std::vector<cv::Point>	posHistory)
{
	std::vector<cv::Point> pointDiff;
	cv::Point rateOfChange = 0;
	cv::Point nextDirection;
	cv::Point nextPoint;

	if (posHistory.size() >= 3) {
		for (int i = 1; i < posHistory.size(); i++) {
			pointDiff.push_back(posHistory[i] - posHistory[i - 1]);
		}
		for (int i = 1; i < pointDiff.size(); i++) {
			cv::Point newRateOfChange = pointDiff[i] - pointDiff[i - 1];
			rateOfChange = (rateOfChange * (i - 1) + newRateOfChange) / (i);
		}
		nextDirection = rateOfChange + pointDiff[pointDiff.size() - 1];
		nextPoint = posHistory[posHistory.size() - 1] + nextDirection;
	}
	else if (posHistory.size() == 2) {
		nextPoint = posHistory[1] + (posHistory[1] - posHistory[0]);
	}
	return nextPoint;
}