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
	cap >> cFr1;
	int dilationSize = 3;
	int erosionSize = 2;

	//Adding the relevant CUDA methods to manipulate each frame with the GPU
	cv::Mat eroElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1));
	cv::Mat dilElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1));

	cv::Ptr<cv::BackgroundSubtractor> BSM = cv::cuda::createBackgroundSubtractorMOG2(1000);
	cv::Ptr<cv::cuda::Filter> gausFilter = cv::cuda::createGaussianFilter(16, 16, cv::Size(13, 13), 0);

	cv::namedWindow("cFr1", cv::WINDOW_KEEPRATIO);
	cv::namedWindow("Threshold", cv::WINDOW_GUI_NORMAL);

	int threshL = 3;
	int threshH = 255;
	int weight = 95;

	cv::createTrackbar("LowerVal", "Threshold", &threshL, 255);
	cv::createTrackbar("HigherVal", "Threshold", &threshH, 255);
	cv::createTrackbar("Weight", "Threshold", &weight, 100);

	std::vector<std::vector<cv::Point>> locations;
	std::vector<cv::Rect> possiblePeople, possibleBall;
	std::vector<sbt::SBTracker::TrackedObj> personIds, objIds;
	cv::createTrackbar("Weight", "Threshold", &weight, 100);

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

int main(int argc, char* argv[]) {
	videoAnalysisV1();
	return 0;
}