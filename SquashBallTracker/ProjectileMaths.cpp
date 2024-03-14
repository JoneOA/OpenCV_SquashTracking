#include "ProjectileMaths.hpp"

std::vector<double> ProjectileMaths::CalculateCoefficients(std::vector<cv::Point>& path)
{
	unsigned long long xij, yi;
	std::vector<double> coeff;

	std::vector<std::vector<unsigned long long>> M = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
	std::vector<std::vector<unsigned long long>> Mi = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
	std::vector<unsigned long long> B = {0, 0, 0};

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			yi = 0;
			xij = 0;
			for (int k = 0; k < path.size(); k++) {
				xij += powf(path[k].x, i + j);
				yi += powf(path[k].x, i) * path[k].y;
			}
			M[i][j] = xij;
		}
		B[i] = yi;
	}
	long long detM = CalculateMatrixDeterminant(M);
	for (int i = 0; i < M.size(); i++) {
		Mi = M;
		Mi[i] = B;
		long long detMi = CalculateMatrixDeterminant(Mi);
		coeff.push_back(((double)detMi / (double)detM));
	}

	return coeff;
}

long long ProjectileMaths::CalculateMatrixDeterminant(std::vector<std::vector<unsigned long long>>& M)
{
	long long t1 = M[0][0] * M[1][1] * M[2][2] - M[0][0] * M[1][2] * M[2][1];
	long long t2 = M[0][1] * M[1][2] * M[2][0] - M[0][1] * M[1][0] * M[2][2];
	long long t3 = M[0][2] * M[1][0] * M[2][1] - M[0][2] * M[1][1] * M[2][0];
	long long determinant = t1 + t2 + t3;

	return determinant;
}
