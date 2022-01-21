#include <iostream>

#include <opencv2/opencv.hpp>
#include "Tracker.hpp"
#include "TrajectoryPreditor.hpp"

bool tpd::TrajectoryPredictor::nextPosition(std::vector<cv::Rect> posHistory)
{
	std::vector<cv::Point> pointDiff;
	std::vector<float> polynomialCoeff;
	cv::Point rateOfChange = 0;
	cv::Point nextDirection;
	bool followsProjectile = false;
	float polyFit;
	float uncertaintly = 50;


	if (posHistory.size() >= 4) {
		polynomialCoeff = polynomialPath(posHistory);
		for (int i = 0; i < posHistory.size(); i++) {
			polyFit = (polynomialCoeff[2] * powf(posHistory[i].x, 2)) + (posHistory[i].x * polynomialCoeff[1]) + polynomialCoeff[0];
			if (abs(polyFit - posHistory[i].y) < uncertaintly)
			{
				followsProjectile = true;
			}
			else 
			{
				followsProjectile = false;
				break;
			}
		}
	}
	if (followsProjectile) {
		//std::cout << "PROJECTILE! - ";
	}
	return followsProjectile;
}

//DEBUG HERE VALUES AREN'T exactly as expected

std::vector<float> tpd::TrajectoryPredictor::polynomialPath(std::vector<cv::Rect> posHistory)
{
	float xij, yi;
	std::vector<float> coeff;

	std::vector<std::vector<float>> M = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
	std::vector<std::vector<float>> Mi = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
	std::vector<float> B = {0, 0, 0};
	for (int i = 0; i < M.size(); i++) {
		for (int j = 0; j < 3; j++) {
			yi = 0;
			xij = 0;
			for (int k = 0; k < posHistory.size(); k++) {
				xij += powf(posHistory[k].x, i + j);
				yi += powf(posHistory[k].x, i) * posHistory[k].y;
			}
			M[i][j] = xij;
		}
		B[i] = yi;
	}

	for (int i = 0; i < M.size(); i++) {
		Mi = M;
		for (int j = 0; j < M[i].size(); j++) {
			Mi[j][i] = B[j];
			//std::cout << M[i][j] << ", ";
		}
		//std::cout << "\n";
		coeff.push_back((calcDeterminant(Mi) / calcDeterminant(M)));
	}

	return coeff;
}

float tpd::TrajectoryPredictor::calcDeterminant(std::vector<std::vector<float>> M)
{	
	float determinant = ((M[0][0] * M[1][1] * M[2][2]) + (M[0][1] * M[1][2] * M[2][0]) + (M[1][0] * M[2][1] * M[0][2])) - ((M[0][2] * M[1][1] * M[2][0]) + (M[0][1] * M[1][0] * M[2][2]) + (M[0][0] * M[1][2] * M[2][1]));

	return determinant;
}