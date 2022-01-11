#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaobjdetect.hpp>
#include <opencv2/cudabgsegm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/video/tracking.hpp>
#include "Tracker.hpp"



int videoAnalysisV1() {
	//Adding path of video and capturing the frames using VideoCapture
	std::string videoPath = "E:\\Documents\\Projects\\opencv\\SquashFootage.avi";
	cv::VideoCapture cap(videoPath);

	cv::Mat cFr1, cFr2, ave, dif;
	cv::cuda::GpuMat gFr1, gFr2;
	int erosionSize = 2;
	int dilationSize = 3;

	//Adding the relevant CUDA methods to manipulate each frame with the GPU
	cv::Mat eroElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1));
	cv::Mat dilElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1));

	cv::Ptr<cv::BackgroundSubtractor> BSM = cv::cuda::createBackgroundSubtractorMOG2();
	cv::Ptr<cv::cuda::Filter> gausFilter = cv::cuda::createGaussianFilter(16, 16, cv::Size(13, 13), 0);
	cv::Ptr<cv::cuda::Filter> dilFilter = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, dilElement);
	cv::Ptr<cv::cuda::Filter> eroFilter = cv::cuda::createMorphologyFilter(cv::MORPH_ERODE, CV_8UC1, eroElement);

	std::vector<std::vector<cv::Point>> locations;
	std::vector<cv::Rect> ContourBoxTracker;
	std::vector<sbt::SBTracker::TrackedObj> objIds;
	double area;
	
	//Creating the tracker and the variable type for object tracking between frames;
	sbt::SBTracker tracker;
	sbt::SBTracker::TrackedObj t;

	cv::namedWindow("Squash ball detection", cv::WINDOW_NORMAL);

	while (true) {
		
		ContourBoxTracker.clear();

		//Capturing the frame and uploading it to the GPU
		cap >> cFr1;

		//cv::cvtColor(cFr1, cFr2, cv::COLOR_BGR2GRAY);

		gFr1.upload(cFr1);

		//TODO:Research how to differentiate motion blur of a small object from shadows in the image
		//TODO:Ignore any object that is around the ElShorbaggy's

		//Running a Gaussian Blur and removing static background
		gausFilter->apply(gFr1, gFr1);
		BSM->apply(gFr1, gFr2);
		eroFilter->apply(gFr2, gFr2);
		dilFilter->apply(gFr2, gFr2);
		gFr2.download(cFr2);

		//if (ave.empty()) {
		//	cFr2.copyTo(ave);
		//}


		//TODO: Figure out how to get this accumulateWeighted to work
		//TODO: Implement longer history into tracker so full path of objects can be taken into account

		//cv::accumulateWeighted(cFr2, ave, 0.05 cv::Mat());
		//cv::absdiff(cFr2, ave, dif);

		//cv::threshold(dif, dif, 125, 255, cv::THRESH_BINARY);
		//cv::absdiff(cFr2, ave, dif);
		//Finding the "objects" in the frame that are between the size of 20 an 300 and appending them to a vector for tracking to take place
	
		cv::findContours(cFr2, locations, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i = 0; i < locations.size(); i++) {
			area = cv::contourArea(locations[i]);
			if (area > 100 && area < 300) {
				ContourBoxTracker.push_back(boundingRect(locations[i]));
			}
		}

		objIds = tracker.distanceTracker(ContourBoxTracker);

		for (int index = 0; index < objIds.size(); index++) {
			if (!objIds[index].newObject) {
				cv::rectangle(cFr1, objIds[index].position, cv::Scalar(0, 255, 0), 2);
				cv::line(cFr1, cv::Point2f(objIds[index].position.x, objIds[index].position.y), cv::Point2f(objIds[index].position.x - objIds[index].dir.x, objIds[index].position.y - objIds[index].dir.y), cv::Scalar(0, 0, 255), 2);
				cv::putText(cFr1, std::to_string(objIds[index].id), cv::Point(objIds[index].position.x, objIds[index].position.y), 1, 1, cv::Scalar(255, 0, 0));
			}
		}
		
		cv::imshow("Squash ball detection", cFr1);

		if (cv::waitKey(30) > 0) {
			break;
		}
	}
	cv::destroyWindow("Squash ball detection");
	return 0;
}

void pictureAnalysis() {
	cv::Mat src_host, baw_img, edge_host, bin_img, can_mat;
	cv::cuda::GpuMat image, edges, houghOut, canOut;
	std::string srcWin = "Source";
	std::string edgeWin = "Edges";
	cv::namedWindow(srcWin);
	cv::namedWindow(edgeWin);

	//Creating the detector objects
	//TODO: Tweak the values so the correct number of circles can be displayed
	cv::Ptr<cv::cuda::CannyEdgeDetector> can = cv::cuda::createCannyEdgeDetector(100, 200, 3, false);
	cv::Ptr<cv::cuda::HoughCirclesDetector> circ = cv::cuda::createHoughCirclesDetector(2, 1000, 1000, 10, 200, 500);

	//Importing the picture into the program
	std::string picturePath = "E:\\Documents\\Projects\\opencv\\circle.png";
	src_host = cv::imread(picturePath);

	//Preparing the image for object detection
	cv::cvtColor(src_host, baw_img, cv::COLOR_RGB2GRAY, 0); //Converting colour to grayscale
	GaussianBlur(baw_img, baw_img, cv::Size(3, 3), 0, 0);	//Denoising the photo
	threshold(baw_img, bin_img, 128, 255, cv::THRESH_OTSU); //Creating a black and white photo based on the white values in the image

	//Loading the image into the GPU
	image.upload(bin_img);

	//Edge detection first to find the edges in the photo, followed by Hough Circle Detection to find the circles that are present
	can->detect(image, edges);
	circ->detect(edges, houghOut);

	//Retreiving the object detected images from the GPU
	edges.download(can_mat);		//The image with the edges being displayed as white lines on black background
	houghOut.download(edge_host);	//The values of the circles found, returns as floats in sets of 3 -> (x pos, y pos, radius)

	//Drawing the circles to the source image, for comparison of position
	cv::Size size = edge_host.size();
	for (int i = 0; i < size.area();) {
		float centre[2] = { edge_host.at<float>(0, i), edge_host.at<float>(0, i++) };
		//cv::Point sendIt(centre[0], centre[1]);
		//circle(src_host, sendIt, edge_host.at<float>(0, i++), cv::Scalar(0, 255, 0), 1);
	}

	//Output the images
	imshow(srcWin, src_host);
	imshow(edgeWin, can_mat);

	//Keeps the images displayed until a key is pressed
	while (true) {
		if (cv::waitKey(30) >= 0) {
			break;
		}
	}
	cv::destroyAllWindows();
}

int main(int argc, char* argv[]){
	videoAnalysisV1();
	return 0;
}
