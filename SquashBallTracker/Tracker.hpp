#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
 
namespace sbt {

	class SBTracker {
		public:	
			struct TrackedObj
			{
				std::vector<cv::Point> positions;
				int id;
				float xDir;
				float yDir;
				int width;
				int height;
				int framesMissing;
				bool isBall;
			};

			std::vector<TrackedObj> distanceTracker(std::vector<cv::Rect> detectedObjs);

		private:

			std::vector<TrackedObj> newObjects;
			std::vector<TrackedObj> classifiedObjects;
			std::vector<TrackedObj> tempObj;
			int identifier = 1;
	};


}