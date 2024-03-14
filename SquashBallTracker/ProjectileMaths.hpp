#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class ProjectileMaths
{
public:
	std::vector<double> CalculateCoefficients(std::vector<cv::Point>& path);
	long long CalculateMatrixDeterminant(std::vector<std::vector<unsigned long long>>& m);

private:

};