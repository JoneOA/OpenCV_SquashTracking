#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaobjdetect.hpp>
#include <opencv2/cudabgsegm.hpp>
#include <opencv2/cudafilters.hpp>
#include "Tracker.hpp"

void videoAnalysis() {
	//Adding path of video and capturing the frames using VideoCapture
	std::string videoPath = "E:\\Documents\\Projects\\opencv\\SquashFootage.avi";
	cv::VideoCapture cap(videoPath);

	//Adding the relevant CUDA methods to manipulate each frame with the GPU
	cv::Ptr<cv::BackgroundSubtractor> BSM = cv::cuda::createBackgroundSubtractorMOG2();
	cv::Ptr<cv::cuda::Filter> filter = cv::cuda::createGaussianFilter(16, 16, cv::Size(9, 9), 0);

	cv::Mat cFr1, cFr2;
	cv::cuda::GpuMat gFr1, gFr2;

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
		gFr1.upload(cFr1);

		//Running a Gaussian Blur and removing static background
		filter->apply(gFr1, gFr1);
		BSM->apply(gFr1, gFr2);

		gFr2.download(cFr2);

		//Finding the "objects" in the frame that are between the size of 20 an 300 and appending them to a vector for tracking to take place
		cv::findContours(cFr2, locations, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i = 0; i < locations.size(); i++) {
			area = cv::contourArea(locations[i]);
			if (area > 20 && area < 300) {
				//cv::rectangle(cFr1, cv::boundingRect(locations[i]), cv::Scalar(0, 255, 0), 2);
				ContourBoxTracker.push_back(boundingRect(locations[i]));
			}
		}

		//TODO: Bounding rectangles are not having their positions updated
		//TODO: Contour Box Traker is having repeated values inserted

		objIds = tracker.distanceTracker(ContourBoxTracker);

		for (sbt::SBTracker::TrackedObj obj : objIds) {
			cv::rectangle(cFr1, obj.position, cv::Scalar(0, 255, 0), 2);
			cv::putText(cFr1, std::to_string(obj.id), cv::Point(obj.position.x, obj.position.y), 1, 1, cv::Scalar(255, 0, 0));
		}

		cv::imshow("Squash ball detection", cFr1);

		if (cv::waitKey(1) > 0) {
			break;
		}
	}

	cv::destroyWindow("Squash ball detection");
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
		cv::Point sendIt(centre[0], centre[1]);
		circle(src_host, sendIt, edge_host.at<float>(0, i++), cv::Scalar(0, 255, 0), 1);
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
	videoAnalysis();
	return 0;
}
