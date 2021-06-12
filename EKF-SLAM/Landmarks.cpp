#include "Landmarks.h"

#include "Robot.h"

double RandomFloat() {
	return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

double distance(const std::pair<double, double>& a, const std::pair<double, double>& b) {
	return sqrt((a.first - b.first) * (a.first - b.first) +
		(a.second - b.second) * (a.second - b.second));
}

std::vector<std::pair<double, double> > GenerateLandmarks(
	double x_min, double x_max, double y_min, double y_max, int n) {
	double h = x_max - x_min;
	double v = y_max - y_min;

	std::vector<std::pair<double, double> > res;
	for (int i = 0; i < n; i++) {
		double x = x_min + RandomFloat() * h;
		double y = y_min + RandomFloat() * v;
		res.push_back(std::make_pair(x, y));
	}

	return res;
}

std::vector<int> FindLandmarks(double x, double y, double r, const std::vector<std::pair<double, double> >& landmarks) {
	std::vector<int> res;
	for (int i = 0; i < landmarks.size(); i++) {
		if (distance(std::make_pair(x, y), landmarks[i]) <= r) {
			res.push_back(i);
		}
	}

	return res;
}

ObservationModel CreateObservationModel() {
	std::function<Vector(const Vector&, const Vector&)> H = [](const Vector& x, const Vector& xl) {
		Vector m = Vector(2);
		m[0] = sqrt((xl[0] - x[0]) * (xl[0] - x[0]) +
				(xl[1] - x[1]) * (xl[1] - x[1]));
		m[1] = atan2((xl[1] - x[1]), (xl[0] - x[0]));
		return m;
	};

	std::function<Matrix(const Vector&, const Vector&)> H_x = [](const Vector& x, const Vector& xl) {
		Matrix m = Matrix(2, 4);
		m[0][0] = -(xl[0] - x[0]) /
			sqrt((xl[0] - x[0]) * (xl[0] - x[0]) +
				(xl[1] - x[1]) * (xl[1] - x[1]));
		m[0][1] = -(xl[1] - x[1]) /
			sqrt((xl[0] - x[0]) * (xl[0] - x[0]) +
				(xl[1] - x[1]) * (xl[1] - x[1]));
		m[1][0] = (xl[1] - x[1]) /
			((xl[0] - x[0]) * (xl[0] - x[0]) +
				(xl[1] - x[1]) * (xl[1] - x[1]));
		m[1][1] = -(xl[0] - x[0]) /
			((xl[0] - x[0]) * (xl[0] - x[0]) +
				(xl[1] - x[1]) * (xl[1] - x[1]));
		return m;
	};

	std::function<Matrix(const Vector&, const Vector&)> H_l = [](const Vector& x, const Vector& xl) {
		Matrix m = Matrix(2, 2);
		m[0][0] = (xl[0] - x[0]) /
			sqrt((xl[0] - x[0]) * (xl[0] - x[0]) +
				(xl[1] - x[1]) * (xl[1] - x[1]));
		m[0][1] = (xl[1] - x[1]) /
			sqrt((xl[0] - x[0]) * (xl[0] - x[0]) +
				(xl[1] - x[1]) * (xl[1] - x[1]));
		m[1][0] = -(xl[1] - x[1]) /
			((xl[0] - x[0]) * (xl[0] - x[0]) +
				(xl[1] - x[1]) * (xl[1] - x[1]));
		m[1][1] = (xl[0] - x[0]) /
			((xl[0] - x[0]) * (xl[0] - x[0]) +
				(xl[1] - x[1]) * (xl[1] - x[1]));
		return m;
	};

	std::function<Vector(const Vector&, const Vector&)> G = [](const Vector& x, const Vector& y) {
		Vector m = Vector(2);
		m[0] = x[0] + y[0] * cos(y[1]);
		m[1] = x[1] + y[0] * sin(y[1]);
		return m;
	};

	std::function<Matrix(const Vector&, const Vector&)> G_x = [](const Vector& x, const Vector& y) {
		Matrix m = Matrix(2, 4);
		m[0][0] = 1;
		m[1][1] = 1;
		return m;
	};

	std::function<Matrix(const Vector&, const Vector&)> G_y = [](const Vector& x, const Vector& y) {
		Matrix m = Matrix(2, 2);
		m[0][0] = cos(y[1]);
		m[0][1] = -y[0]*sin(y[1]);
		m[1][0] = sin(y[1]);
		m[1][1] = y[0] * cos(y[1]);
		return m;
	};

	return ObservationModel(H, H_x, H_l, G, G_x, G_y);
}