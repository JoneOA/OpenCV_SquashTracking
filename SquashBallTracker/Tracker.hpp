#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
 
namespace sbt {

	class SBTracker {
		public:	
			struct direction 
			{
				float x = 0;
				float y = 0;
			};
			struct TrackedObj
			{
				cv::Rect position;
				int id;
				double distanceMoved;
				direction dir;
				bool newObject;
			};

			std::vector<TrackedObj> distanceTracker(std::vector<cv::Rect> detectedObjs);

		private:

			std::vector<TrackedObj> newObjects;
			std::vector<TrackedObj> classifiedObjects;
			std::vector<TrackedObj> tempObj;
			int identifier = 1;
	};


}