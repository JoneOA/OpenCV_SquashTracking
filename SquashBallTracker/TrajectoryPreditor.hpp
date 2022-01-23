#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "Tracker.hpp"

namespace tpd {

	class TrajectoryPredictor {
	public:

		static bool nextPosition(std::vector<cv::Rect>);
	

	private:

		static std::vector<double> polynomialPath(std::vector<cv::Rect>);
		static long long calcDeterminant(std::vector<std::vector<unsigned long long>>);
	};


}