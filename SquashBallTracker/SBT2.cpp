#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

#include "App.hpp"

int main()
{  
	std::string videoPath = "E:/Documents/Projects/opencv/SquashFootage.avi";
	App app;

	app.Init(videoPath);

	while (true)
	{
		app.Run();
		if (cv::waitKey(1) > 0)
			break;
	}
}