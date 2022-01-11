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

	cv::Mat cFr1, cFr2, cFr3, d1, d2, d3, t1;
	cv::cuda::GpuMat gFr1, gFr2;
	int erosionSize = 2;
	int dilationSize = 3;

	cap >> cFr1;

	//Adding the relevant CUDA methods to manipulate each frame with the GPU
	cv::Mat eroElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1));
	cv::Mat dilElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1));

	cv::Ptr<cv::BackgroundSubtractor> BSM = cv::cuda::createBackgroundSubtractorMOG2(1000);
	cv::Ptr<cv::cuda::Filter> gausFilter = cv::cuda::createGaussianFilter(16, 16, cv::Size(13, 13), 0);
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

	std::vector<std::vector<cv::Point>> locations;
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

		d1 = cFr2 - cFr1;
		d2 = cFr3 - cFr1;
		d3 = cFr3 - cFr2;

		cv::threshold(d1, d1, 10, 255, cv::THRESH_BINARY);
		cv::threshold(d2, d2, 10, 255, cv::THRESH_BINARY);
		cv::threshold(d3, d3, 10, 255, cv::THRESH_BINARY);

		t1 = d1 + d2 + d3;

		cv::cvtColor(t1, t1, cv::COLOR_BGR2GRAY);
		
		gFr1.upload(cFr1);

		//TODO: Ignore any object that is around the ElShorbaggy's

		//TODO: CONVERT CPU TASKS TO GPU TASKS FOR OPTIMISATION
		//Running a Gaussian Blur and removing static background
		//gausFilter->apply(gFr1, gFr2);
		//BSM->apply(gFr2, gFr2);
		//eroFilter->apply(gFr2, gFr2);
		//dilFilter->apply(gFr2, gFr2);
		//gFr2.download(cFr2);

		//TODO: Implement longer history into tracker so full path of objects can be taken into account
		
		//Finding the potential "Players" in the frame that are between the size of 1000 an 10000 and potential "Balls" that are between 20 and 300
		cv::findContours(t1, locations, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i = 0; i < locations.size(); i++) {
			area = cv::contourArea(locations[i]);
			if (area > 1000 && area < 10000) {
				possiblePeople.push_back(boundingRect(locations[i]));
			}
			else if (area > 20 && area < 300) {
				possibleBall.push_back(boundingRect(locations[i]));
			}
		}

		//Tracking the objects in the by possible people and possible balls
		personIds = shorbaggyIdentifier.distanceTracker(possiblePeople);
		objIds = ballTracker.distanceTracker(possibleBall);

		for (int indexBall = 0; indexBall < objIds.size(); indexBall++) {
			intersect = false;
			for (int indexPers = 0; indexPers < personIds.size(); indexPers++) {
				if (!intersect) {
					intersect = ((objIds[indexBall].position & personIds[indexPers].position).area() > 0) ? true : false;
				}
				cv::rectangle(cFr1, cv::Rect(personIds[indexPers].position), cv::Scalar(255, 255, 0));
			}
			if (!objIds[indexBall].newObject && !intersect) {
				cv::rectangle(cFr1, objIds[indexBall].position, cv::Scalar(0, 255, 0), 2);
				//cv::rectangle(cFr1, cv::Rect(objIds[indexBall].position.x + objIds[indexBall].position.width/2 - 50, objIds[indexBall].position.y + objIds[indexBall].position.height/2 - 50, 100, 100), cv::Scalar(0, 255, 255), 2);
				cv::line(cFr1, cv::Point2f(objIds[indexBall].position.x + objIds[indexBall].position.width / 2, objIds[indexBall].position.y + objIds[indexBall].position.height / 2), cv::Point2f(objIds[indexBall].position.x + objIds[indexBall].position.width / 2 - objIds[indexBall].dir.x, objIds[indexBall].position.y + objIds[indexBall].position.height / 2 - objIds[indexBall].dir.y), cv::Scalar(0, 0, 255), 2);
				cv::putText(cFr1, std::to_string(objIds[indexBall].id), cv::Point(objIds[indexBall].position.x, objIds[indexBall].position.y), 1, 1, cv::Scalar(255, 0, 0));
			}
		}
		
		cv::imshow("cFr1", d1);

		if (cv::waitKey(1) > 0) {
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

int main(int argc, char* argv[]){
	videoAnalysisV1();
	return 0;
}
