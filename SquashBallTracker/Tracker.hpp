#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
 
namespace sbt {

	class SBTracker {
		public:	
			struct TrackedObj
			{
				cv::Rect position;
				int id;
			};

			std::vector<TrackedObj> distanceTracker(std::vector<cv::Rect> detectedObjs);
			TrackedObj createTrackedObject(cv::Rect trackedObjects);

		private:

			std::vector<TrackedObj> newObjects;
			std::vector<TrackedObj> classifiedObjects;
			std::vector<TrackedObj> tempObj;
			int identifier = 1;
	};


}