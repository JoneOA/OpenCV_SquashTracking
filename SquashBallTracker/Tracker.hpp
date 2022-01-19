#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
 
namespace sbt {

	class SBTracker {
		public:	
			struct TrackedObj
			{
				cv::Point location;
				int width;
				int height;
			};

			std::vector<TrackedObj> distanceTracker(std::vector<cv::Rect> detectedObjs);

		private:

			bool isWithinBounds(cv::Point detectedObj, cv::Point ClassifiedObj, int uncertainty);
			cv::Rect findNextLink(int objectId, int j);
			std::vector<TrackedObj> newObjects;
			std::vector<std::vector<cv::Rect>> detectionHistory;
			std::vector<std::vector<std::vector<int>>> linkedDetections;
			std::vector<TrackedObj> tempObj;
			int identifier = 1;
	};


}