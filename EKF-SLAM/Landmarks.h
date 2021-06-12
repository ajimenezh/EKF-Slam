#pragma once

#include <vector>
#include <functional>

#include "Robot.h"

std::vector<std::pair<double, double> > GenerateLandmarks(
	double x_min, double x_max, double y_min, double y_max, int n);

std::vector<int> FindLandmarks(double x, double y, double r, const std::vector<std::pair<double, double> >& landmarks);

class ObservationModel {
public:
	ObservationModel(std::function<Vector(const Vector&, const Vector&)> H, std::function<Matrix(const Vector&, const Vector&)> H_x,
		std::function<Matrix(const Vector&, const Vector&)> H_l,
		std::function<Vector(const Vector&, const Vector&)> G, 
		std::function<Matrix(const Vector&, const Vector&)> G_x,
		std::function<Matrix(const Vector&, const Vector&)> G_y) : H_(H), H_x_(H_x), H_l_(H_l), G_(G), G_x_(G_x), G_y_(G_y) {}

	Vector H(const Vector& x, const Vector& xl) {
		return H_(x, xl);
	}

	Matrix H_x(const Vector& x, const Vector& xl) {
		return H_x_(x, xl);
	}

	Matrix H_l(const Vector& x, const Vector& xl) {
		return H_l_(x, xl);
	}

	Vector G(const Vector& x, const Vector& y) {
		return G_(x, y);
	}

	Matrix G_x(const Vector& x, const Vector& y) {
		return G_x_(x, y);
	}

	Matrix G_y(const Vector& x, const Vector& y) {
		return G_y_(x, y);
	}

private:
	std::function<Vector(const Vector&, const Vector&)> H_;
	std::function<Matrix(const Vector&, const Vector&)> H_x_;
	std::function<Matrix(const Vector&, const Vector&)> H_l_;

	std::function<Vector(const Vector&, const Vector&)> G_;
	std::function<Matrix(const Vector&, const Vector&)> G_x_;
	std::function<Matrix(const Vector&, const Vector&)> G_y_;
};

ObservationModel CreateObservationModel();