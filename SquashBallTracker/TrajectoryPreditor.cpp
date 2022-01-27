#include <iostream>

#include <opencv2/opencv.hpp>
#include "Tracker.hpp"
#include "TrajectoryPreditor.hpp"

bool tpd::TrajectoryPredictor::nextPosition(std::vector<cv::Rect> posHistory)
{
	std::vector<cv::Point> pointDiff;
	std::vector<double> polynomialCoeff;
	cv::Point rateOfChange = 0;
	cv::Point nextDirection;
	bool followsProjectile = false;
	float polyFit;
	float uncertaintly = 5;

	//TODO: Check that points are along the arc in the correct order!
	if (posHistory.size() >= 4) {
		int xDir = posHistory[0].x - posHistory[1].x > 0 ? 1 : -1;
		int yDir = posHistory[0].y - posHistory[1].y > 0 ? 1 : -1;
		polynomialCoeff = polynomialPath(posHistory);
		for (int i = 0; i < posHistory.size(); i++) {
			polyFit = (polynomialCoeff[2] * powf(posHistory[i].x, 2)) + (posHistory[i].x * polynomialCoeff[1]) + polynomialCoeff[0];
			if (abs(polyFit - posHistory[i].y) < uncertaintly) {
				if (i + 1 < posHistory.size()) {
					if (posHistory[i].x - posHistory[i + 1].x > 0 && xDir > 0 || posHistory[i].x - posHistory[i + 1].x < 0 && xDir < 0) {
						followsProjectile = true;
					}
				}
			}
				else
				{
					followsProjectile = false;
					break;
				}


		}
	}
	return followsProjectile;
}

std::vector<double> tpd::TrajectoryPredictor::polynomialPath(std::vector<cv::Rect> posHistory)
{
	unsigned long long xij, yi;
	std::vector<double> coeff;

	std::vector<std::vector<unsigned long long>> M = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
	std::vector<std::vector<unsigned long long>> Mi = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
	std::vector<unsigned long long> B = {0, 0, 0};
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
	long long detM = calcDeterminant(M);
	for (int i = 0; i < M.size(); i++) {
		Mi = M;
		Mi[i] = B;
		long long detMi = calcDeterminant(Mi);
		coeff.push_back(((double)detMi / (double)detM));
	}

	return coeff;
}

long long tpd::TrajectoryPredictor::calcDeterminant(std::vector<std::vector<unsigned long long>> M)
{	
	long long t1 = M[0][0] * M[1][1] * M[2][2] - M[0][0] * M[1][2] * M[2][1];
	long long t2 = M[0][1] * M[1][2] * M[2][0] - M[0][1] * M[1][0] * M[2][2];
	long long t3 = M[0][2] * M[1][0] * M[2][1] - M[0][2] * M[1][1] * M[2][0];
	long long determinant = t1 + t2 + t3;

	return determinant;
}