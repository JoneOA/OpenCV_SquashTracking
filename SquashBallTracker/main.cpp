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

	cv::Mat cFr1, cFr2, cFr3;			//Capture Frames
	cv::Mat ball, person;

	cv::cuda::GpuMat d1, d2, d3;		//Delta Frames (Differences between capture frames)
	cv::cuda::GpuMat b1, b2, b3, bc;	//Frames optimised for ball tracking
	cv::cuda::GpuMat p1, p2, p3, pc;	//Frames optimised for player tracking
	cv::cuda::GpuMat gFr1, gFr2, gFr3;	//GPU Frames
	int erosionSize = 2;
	int dilationSize = 3;

	//Adding the relevant CUDA methods to manipulate each frame with the GPU
	cv::Ptr<cv::BackgroundSubtractor> BSM = cv::cuda::createBackgroundSubtractorMOG2(1000);
	cv::Ptr<cv::cuda::Filter> gausFilter = cv::cuda::createGaussianFilter(16, 16, cv::Size(3, 3), 0);

	cv::Mat eroElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1, 1));
	cv::Mat dilElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1));
	cv::Ptr<cv::cuda::Filter> dilFilter = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, dilElement);
	cv::Ptr<cv::cuda::Filter> eroFilter = cv::cuda::createMorphologyFilter(cv::MORPH_ERODE, CV_8UC1, eroElement);

	cv::namedWindow("cFr1", cv::WINDOW_KEEPRATIO);
	cv::namedWindow("Threshold", cv::WINDOW_GUI_NORMAL);

	int threshL = 167;
	int threshH = 255;
	int weight = 95;

	cv::createTrackbar("LowerVal", "Threshold", &threshL, 255);
	cv::createTrackbar("HigherVal", "Threshold", &threshH, 255);
	cv::createTrackbar("Weight", "Threshold", &weight, 100);

	std::vector<std::vector<cv::Point>> ballLocations, personLocations;
	std::vector<cv::Rect> possiblePeople, possibleBall;
	std::vector<sbt::SBTracker::TrackedObj> personIds, objIds;
	double area;
	bool intersect;

	//Creating the tracker and the variable type for object tracking between frames;
	sbt::SBTracker ballTracker;
	sbt::SBTracker shorbaggyIdentifier;
	sbt::SBTracker::TrackedObj t;

	while (true) {
		possibleBall.clear();
		possiblePeople.clear();

		//Capturing the frame and uploading it to the GPU
		cap >> cFr1;
		cap >> cFr2;
		cap >> cFr3;

		gFr1.upload(cFr1);
		gFr2.upload(cFr2);
		gFr3.upload(cFr3);

		gausFilter->apply(gFr1, gFr1);
		gausFilter->apply(gFr2, gFr2);
		gausFilter->apply(gFr3, gFr3);

		//Calculating the deltas (Differences) between each frame
		cv::cuda::subtract(gFr2, gFr1, d1);
		cv::cuda::subtract(gFr3, gFr1, d2);
		cv::cuda::subtract(gFr3, gFr2, d3);

		//Processing the image to better show the movement of the players
		cv::cuda::threshold(d1, p1, 45, 255, cv::THRESH_BINARY);
		cv::cuda::threshold(d2, p2, 45, 255, cv::THRESH_BINARY);
		cv::cuda::threshold(d3, p3, 45, 255, cv::THRESH_BINARY);
		cv::cuda::add(p1, p2, pc);
		cv::cuda::add(p3, pc, pc);
		cv::cuda::cvtColor(pc, pc, cv::COLOR_BGR2GRAY);

		//Processing the image to better show the movement of the ball
		cv::cuda::threshold(d1, b1, 5, 255, cv::THRESH_BINARY);
		cv::cuda::threshold(d2, b2, 5, 255, cv::THRESH_BINARY);
		cv::cuda::threshold(d3, b3, 5, 255, cv::THRESH_BINARY);
		cv::cuda::add(b1, b2, bc);
		cv::cuda::add(bc, b3, bc);
		cv::cuda::cvtColor(bc, bc, cv::COLOR_BGR2GRAY);

		//TODO: Ignore any object that is around the ElShorbaggy's
		//TODO: CONVERT CPU TASKS TO GPU TASKS FOR OPTIMISATION

		//gFr1.upload(t1);
		//Running a Gaussian Blur and removing static background
		//gausFilter->apply(gFr1, gFr2);
		//BSM->apply(gFr2, gFr2);
		//eroFilter->apply(gFr1, gFr2);
		//dilFilter->apply(gFr2, gFr2);
		//gFr2.download(t1);

		pc.download(person);
		bc.download(ball);

		//Finding the contours of the image and passing it to the simple traker

		//TODO: Combine elements on the person to create a large boundingbox. If the boxes overlap then combine them
		cv::findContours(person, personLocations, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i = 0; i < personLocations.size(); i++) {
			area = cv::contourArea(personLocations[i]);
			if (area > 500 && area < 5000) {
				possiblePeople.push_back(boundingRect(personLocations[i]));
			}
		}

		cv::findContours(ball, ballLocations, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i = 0; i < ballLocations.size(); i++) {
			area = cv::contourArea(ballLocations[i]);
			if (area > 20 && area < 500) {
				possibleBall.push_back(boundingRect(ballLocations[i]));
			}
		}

		//TODO: Draw bounding box around objects to screen
		//personIds = shorbaggyIdentifier.distanceTracker(possiblePeople);
		objIds = ballTracker.distanceTracker(possibleBall);

		for (sbt::SBTracker::TrackedObj per : personIds) {
		cv::rectangle(cFr1, cv::Rect(per.positions.at(per.positions.size() - 1).x - per.width / 2, per.positions.at(per.positions.size() - 1).y - per.height / 2, per.width, per.height), cv::Scalar(0, 0, 255), 3);
		cv::putText(cFr1, std::to_string(per.id), cv::Point(per.positions.at(per.positions.size() - 1).x - per.width / 2, per.positions.at(per.positions.size() - 1).y - per.height / 2), 1, 1, cv::Scalar(255, 255, 0));
		}

		for (sbt::SBTracker::TrackedObj obj : objIds) {
			if (!obj.possibleTrackedObjs.empty()) {
				cv::rectangle(cFr1, cv::Rect(obj.positions.at(obj.positions.size() - 1).x - obj.width / 2, obj.positions.at(obj.positions.size() - 1).y - obj.height / 2, obj.width, obj.height), cv::Scalar(0, 255, 0), 3);
				cv::putText(cFr1, std::to_string(obj.id), cv::Point(obj.positions.at(obj.positions.size() - 1).x - obj.width / 2, obj.positions.at(obj.positions.size() - 1).y - obj.height / 2), 1, 1, cv::Scalar(255, 255, 0));
			}
		}

		cv::imshow("cFr1", cFr1);

		if (cv::waitKey(50) > 0) {
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