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

			void PathSearch(int& ballId);

			std::vector<std::vector<cv::Rect>> distanceTracker(std::vector<cv::Rect> detectedObjs);

		private:
			
			int findNextLink(int objectId, int j);
			void PathSearch();
			std::vector<int> objectPath;
			std::vector<cv::Rect> linkedObjects;
			std::vector<TrackedObj> newObjects;
			std::vector<std::vector<cv::Rect>> detectionHistory;
			std::vector<std::vector<std::vector<int>>> linkedDetections;
			std::vector<TrackedObj> tempObj;
			std::vector<std::vector<cv::Rect>> fullGraph;
			int identifier = 1;
	};


}