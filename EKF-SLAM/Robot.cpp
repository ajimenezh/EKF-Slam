#include "Robot.h"

#include "math.h"

Vector ToUnitVector(const Vector& v) {
	double norm = sqrt(v[0] * v[0] + v[1] * v[1]);
	return v * (1.0 / norm);
}

Vector operator-(const Vector& l, const Vector& r) {
	Vector res = l;
	for (int i = 0; i < r.size(); i++) {
		res[i] -= r[i];
	}
	return res;
}

Matrix MatrixUnion(const Matrix& a, const Matrix& b) {
	Matrix res(a.N() + b.N(), a.M());
	for (int i = 0; i < a.N(); i++) {
		for (int j = 0; j < a.M(); j++) {
			res[i][j] = a[i][j];
		}
	}
	for (int i = 0; i < b.N(); i++) {
		for (int j = 0; j < b.M(); j++) {
			res[i + a.N()][j] = b[i][j];
		}
	}
	return res;
}

Robot CreateRobot2D(const Vector& x) {
	std::function<Vector(const Vector&, const Vector&)> F = [](const Vector& x, const Vector& u) {
		Vector res(4);
		res[2] = u[0];
		res[3] = u[1];
		res[0] = x[2];
		res[1] = x[3];
		return res;
	};

	std::function<Matrix(const Vector&)> F_x = [](const Vector& x) {
		Matrix m = Matrix(4, 4);
		m[0][2] = 1;
		m[1][3] = 1;
		return m;
	};

	std::function<Matrix(const Vector&)> F_n = [](const Vector& x) {
		Matrix m = Matrix(4, 2);
		m[2][0] = 1;
		m[3][1] = 1;
		return m;
	};

	return Robot(x, F, F_x, F_n);
}

Robot CreateRobotDirectional(const Vector& x) {
	std::function<Vector(const Vector&, const Vector&)> F = [](const Vector& x, const Vector& u) {
		Vector res(4);
		res[2] = u[0];
		res[3] = u[1];
		res[0] = x[2] * cos(x[3]);
		res[1] = x[2] * sin(x[3]);
		return res;
	};

	std::function<Matrix(const Vector&)> F_x = [](const Vector& x) {
		Matrix m = Matrix(4, 4);
		m[0][2] = cos(x[3]);
		m[0][3] = -x[2] * sin(x[3]);
		m[1][2] = sin(x[3]);
		m[1][3] = x[2] * cos(x[3]);
		return m;
	};

	std::function<Matrix(const Vector&)> F_n = [](const Vector& x) {
		Matrix m = Matrix(4, 4);
		m[2][0] = 1;
		m[3][1] = 1;
		return m;
	};

	return Robot(x, F, F_x, F_n);
}