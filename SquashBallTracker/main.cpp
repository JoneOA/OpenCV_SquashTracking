#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaobjdetect.hpp>
#include <opencv2/cudabgsegm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/cudaarithm.hpp>
#include "Tracker.hpp"

int videoAnalysisV1() {
	//Adding path of video and capturing the frames using VideoCapture
	std::string videoPath = "E:\\Documents\\Projects\\opencv\\SquashFootage.avi";
	cv::VideoCapture cap(videoPath);

	cv::Mat cFr1, cFr2, cFr3, cProc;
	cv::cuda::GpuMat gFr1, gFr2, gFr3;

	cv::cuda::GpuMat d1, d2, d3, t1;	//Delta Frames (Differences between capture frames)
	cv::cuda::GpuMat b1, b2, b3, bc;	//Frames optimised for ball tracking

	//Adding the relevant CUDA methods to manipulate each frame with the GPU
	cv::Ptr<cv::cuda::Filter> gausFilter = cv::cuda::createGaussianFilter(16, 16, cv::Size(13, 13), 0);

	cv::namedWindow("cFr1", cv::WINDOW_KEEPRATIO);
	cv::namedWindow("Threshold", cv::WINDOW_GUI_NORMAL);

	int threshL = 3;
	int threshH = 255;
	int weight = 95;

	cv::createTrackbar("LowerVal", "Threshold", &threshL, 255);
	cv::createTrackbar("HigherVal", "Threshold", &threshH, 255);

	std::vector<std::vector<cv::Point>> locations;
	std::vector<cv::Rect> possiblePeople, possibleBall;
	std::vector<sbt::SBTracker::TrackedObj> personIds, objIds;

	double area;

	//Creating the tracker and the variable type for object tracking between frames;
	sbt::SBTracker tracker;

	possibleBall.clear();
	possiblePeople.clear();

	while (true) {

		possibleBall.clear();
		cap >> cFr1;
		cap >> cFr2;
		cap >> cFr3;

		gFr1.upload(cFr1);
		gFr2.upload(cFr2);
		gFr3.upload(cFr3);

		gausFilter->apply(gFr1, gFr1);
		gausFilter->apply(gFr2, gFr2);
		gausFilter->apply(gFr3, gFr3);

		cv::cuda::subtract(gFr2, gFr1, d1);
		cv::cuda::subtract(gFr3, gFr2, d2);
		cv::cuda::subtract(gFr3, gFr1, d3);

		cv::cuda::threshold(d1, d1, 3, 255, cv::THRESH_BINARY);
		cv::cuda::threshold(d2, d2, 3, 255, cv::THRESH_BINARY);
		cv::cuda::threshold(d3, d3, 3, 255, cv::THRESH_BINARY);
		
		cv::cuda::add(d1, d2, t1);
		cv::cuda::add(t1, d3, t1);

		cv::cuda::cvtColor(t1, t1, cv::COLOR_BGR2GRAY);
		cv::cuda::threshold(t1, t1, 240, 255, cv::THRESH_BINARY);

		t1.download(cProc);
		
		//TODO: Implement longer history into tracker so full path of objects can be taken into account

		cv::findContours(cProc, locations, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i = 0; i < locations.size(); i++) {
			area = cv::contourArea(locations[i]);
			if (area > 20 && area < 300) {
				possibleBall.push_back(boundingRect(locations[i]));
			}
		}

		//Tracking the objects in the by possible people and possible balls
		objIds = tracker.distanceTracker(possibleBall);

		for (sbt::SBTracker::TrackedObj obj : objIds) {
			if (!obj.newObject) {
				cv::rectangle(cFr1, obj.position, cv::Scalar(0, 255, 0));
				cv::putText(cFr1, std::to_string(obj.id), cv::Point(obj.position.x, obj.position.y),0, 1, cv::Scalar(255, 0, 0));
				cv::line(cFr1, cv::Point(obj.position.x + (obj.position.width / 2), obj.position.y + (obj.position.height / 2)), cv::Point(obj.position.x + (obj.position.width / 2) - obj.dir.x, obj.position.y + (obj.position.height / 2) - obj.dir.y), cv::Scalar(0, 0, 255));
			}
		}

		cv::imshow("cFr1", cFr1);

		if (cv::waitKey(30) > 0) {
			break;
		}
	}
	cv::destroyAllWindows();
	return 0;
}

int main(int argc, char* argv[]) {
	videoAnalysisV1();
	return 0;
}