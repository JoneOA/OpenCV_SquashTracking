#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "Tracker.hpp"

namespace tpd {

	class TrajectoryPredictor {
	public:

		static cv::Point nextPosition(std::vector<cv::Point>);

	};
}