#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaobjdetect.hpp>
#include <opencv2/cudabgsegm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/cudaarithm.hpp>
#include "Tracker.hpp"
#include "TrajectoryPreditor.hpp"

std::vector<cv::Rect> groupNearRects(std::vector<cv::Rect> rectangles, float groupSize) {
	for (int i = 0; i < rectangles.size(); i++) {
		cv::Rect obj1 = rectangles[i];
		obj1.x -= obj1.width / (2 * groupSize);
		obj1.y -= obj1.height / (2 * groupSize);
		obj1.width += obj1.width;
		obj1.height += obj1.height;

		for (int j = i + 1; j < rectangles.size(); j++) {
			cv::Rect obj2 = rectangles[j];
			obj2.x -= obj2.width / (2 * groupSize);
			obj2.y -= obj2.height / (2 * groupSize);
			obj2.width += obj2.width;
			obj2.height += obj2.height;

			if ((obj1 & obj2).area() > 0) {
				obj1.width = obj1.x + obj1.width > obj2.x + obj2.width ? obj1.width : obj2.x + obj2.width - obj1.x;
				obj1.height = obj1.y + obj1.height > obj2.y + obj2.height ? obj1.height : obj2.y + obj2.height - obj1.y;
				obj1.x = MIN(obj1.x, obj2.x);
				obj1.y = MIN(obj1.y, obj2.y);

				rectangles.erase(rectangles.begin() + j);
				j--;
			}
		}
	}
	return rectangles;
}

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

	int threshL = 5;
	int threshH = 255;
	int weight = 95;

	cv::createTrackbar("LowerVal", "Threshold", &threshL, 255);
	cv::createTrackbar("HigherVal", "Threshold", &threshH, 255);
	cv::createTrackbar("Weight", "Threshold", &weight, 100);

	std::vector<std::vector<cv::Point>> ballLocations, personLocations;
	std::vector<cv::Rect> possiblePeople, possibleBall;
	std::vector<cv::Rect> personIds, objIds;
	double area;
	bool intersect;

	//Creating the tracker and the variable type for object tracking between frames;
	sbt::SBTracker ballTracker;
	sbt::SBTracker shorbaggyIdentifier;
	sbt::SBTracker::TrackedObj t;

	std::vector<cv::Rect> foundProjectiles;
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

		gFr1.adjustROI(0, -50, -50, -50);
		gFr2.adjustROI(0, -50, -50, -50);
		gFr3.adjustROI(0, -50, -50, -50);

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

		pc.download(person);
		bc.download(ball);

		cFr1.adjustROI(0, -50, -50, -50);

		cv::findContours(ball, ballLocations, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i = 0; i < ballLocations.size(); i++) {
			area = cv::contourArea(ballLocations[i]);
			if (area > 50 && area < 500) {
				possibleBall.push_back(boundingRect(ballLocations[i]));
			}
		}

		cv::findContours(person, personLocations, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i = 0; i < personLocations.size(); i++) {
			area = cv::contourArea(personLocations[i]);
			if (area > 20 && area < 10000) {
				possiblePeople.push_back(boundingRect(personLocations[i]));
			}
		}

		std::cout << possiblePeople.size() << " Size Before \n";

		possiblePeople = groupNearRects(possiblePeople, 0.01);

		std::cout << possiblePeople.size() << " Size After \n -----------------------\n";

		possibleBall = groupNearRects(possibleBall, 1);

		for (int j = 0; j < possiblePeople.size(); j++) {
			for (int i = 0; i < possibleBall.size(); i++) {
				if ((possibleBall[i] & possiblePeople[j]).area() > 0) {
					possibleBall.erase(possibleBall.begin() + i);
					i--;
				}
			}
		}

		std::vector<std::vector<cv::Rect>> obj = ballTracker.distanceTracker(possibleBall);

		for (int i = 0; i < possiblePeople.size(); i++) {
			cv::rectangle(cFr1, possiblePeople[i], cv::Scalar(0, 0, 255));
			//cv::circle(cFr1, cv::Point(possiblePeople[i].x + (possiblePeople[i].width / 2), possiblePeople[i].y + (possiblePeople[i].height / 2)), 50, cv::Scalar(0,255,255));
			cv::putText(cFr1, std::to_string(i), cv::Point(possiblePeople[i].x, possiblePeople[i].y), 0, 1, cv::Scalar(0, 0, 255));
		}
		////std::cout << "NEW FRAME ---------------------------------------------------------------------\n";
		for (int i = 0; i < possibleBall.size(); i++) {
		cv::rectangle(cFr1, possibleBall[i], cv::Scalar(0, 255, 0));
			//cv::circle(cFr1, cv::Point(possibleBall[i].x + (possibleBall[i].width / 2), possibleBall[i].y + (possibleBall[i].height / 2)), 50, cv::Scalar(0,255,255));
			cv::putText(cFr1, std::to_string(i), cv::Point(possibleBall[i].x, possibleBall[i].y), 0, 1, cv::Scalar(255, 0, 0));
		}
		
		for (int i = 0; i < obj.size(); i++) {
			for(int j = 1; j < obj[i].size(); j++) {
				cv::line(cFr1, cv::Point(obj[i][j - 1].x + (obj[i][j - 1].width / 2), obj[i][j - 1].y + (obj[i][j - 1].height / 2)), cv::Point(obj[i][j].x + (obj[i][j].width / 2), obj[i][j].y + (obj[i][j].height / 2)), cv::Scalar(i*46%255, i * 72%255, i * 113%255));
				cv::rectangle(cFr1, obj[i][j], cv::Scalar((100 * i) % 255, (25 * i ) % 255, (100 * i) % 255));
			}
		}
		

		cv::imshow("cFr1", cFr1);

		if (cv::waitKey(1) > 0) {
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