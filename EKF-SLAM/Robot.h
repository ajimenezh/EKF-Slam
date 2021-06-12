#pragma once

#include <vector>
#include <functional>

const double pi = acos(-1.0);

class Vector : public std::vector<double> {
public:
	Vector() {}
	Vector(int n) : std::vector<double>(n) {}
	Vector(double x, double y) : std::vector<double>({ x, y }) {}
	Vector(std::vector<double> v) : std::vector<double>(v) {}

	Vector operator+(const Vector& v) const{
		Vector res = (*this);
		for (int i = 0; i < v.size(); i++) {
			res[i] += v[i];
		}
		return res;
	}

	void operator+=(const Vector& v) {
		for (int i = 0; i < v.size(); i++) {
			(*this)[i] += v[i];
		}
	}

	Vector operator*(double f) const {
		Vector res = (*this);
		for (int i = 0; i < res.size(); i++) {
			res[i] *= f;
		}
		return res;
	}

	Vector Resize(int n) {
		Vector res = (*this);
		res.resize(n);
		return res;
	}

	void Append(const Vector& other) {
		int n = this->size();
		this->resize(n + other.size());
		for (int i = 0; i < other.size(); i++) {
			(*this)[n + i] = other[i];
		}
	}

	Vector Subarray(int i, int n) {
		Vector res(n);
		for (int j = 0; j < n; j++) {
			res[j] = (*this)[i + j];
		}
		return res;
	}

	void Set(int i, const Vector& other) {
		for (int j = 0; j < other.size(); j++) {
			(*this)[i + j] = other[j];
		}
	}
};

Vector operator-(const Vector& l, const Vector& r);

class Matrix : public std::vector<std::vector<double> > {
public:
	Matrix(int n, int m) {
		this->resize(n);
		for (int i = 0; i < n; i++) {
			(*this)[i].resize(m, 0.0);
		}
	}

	Matrix() {}

	Matrix Transpose() {
		Matrix m(M(), N());
		for (int i = 0; i < N(); i++) {
			for (int j = 0; j < M(); j++) {
				m[j][i] = (*this)[i][j];
			}
		} 
		return m;
	}

	Matrix Inverse() const {
		Matrix m = (*this);

		// Calculate the inverse of the determinant of m.
		int n = N();
		double det = calculateDeterminant(n);
		double inverseDet = 1.0f / det;

		Matrix result(n, n);

		for (int j = 0; j < n; j++) {
			for (int i = 0; i < n; i++) {
				// Get minor of element (j, i) - not (i, j) because
				// this is where the transpose happens.
				double minor = calculateMinor(n, j, i);

				// Multiply by (−1)^{i+j}
				double factor = ((i + j) % 2 == 1) ? -1.0 : 1.0;
				double cofactor = minor * factor;

				result[i][j] = inverseDet * cofactor;
			}
		}

		return result;
	}

	Matrix Submatrix(int x, int y, int n, int m) {
		Matrix res(n, m);
		for (int i = 0; i < n; i++) {
			for (int j = 0; j <m; j++) {
				res[i][j] = (*this)[x + i][y + j];
			}
		}
		return res;
	}

	double calculateDeterminant(int n) const {
		double det = 0.0;

		if (n == 1) return (*this)[0][0];

		if (n == 2) {
			return (*this)[0][0] * (*this)[1][1] -
				(*this)[0][1] * (*this)[1][0];
		}

		for (int i = 0; i < n; i++) {
			// Get minor of element (0, i)
			double minor = calculateMinor(n, 0, i);

			// If this is an odd-numbered row, negate the value.
			double factor = (i % 2 == 1) ? -1.0 : 1.0;

			det += factor * (*this)[0][i] * minor;
		}

		return det;
	}

	double calculateMinor(int n, int row, int col) const {
		auto minorSubmatrix = getMinor(n, row, col);
		return minorSubmatrix.calculateDeterminant(n - 1);
	}

	Matrix getMinor(int n, int row, int col) const {
		int colCount = 0, rowCount = 0;

		Matrix dest(n - 1, n - 1);
		for (int i = 0; i < n; i++) {
			if (i != row) {
				colCount = 0;
				for (int j = 0; j < n; j++) {
					if (j != col) {
						dest[rowCount][colCount] = (*this)[i][j];
						colCount++;
					}
				}
				rowCount++;
			}
		}

		return dest;
	}

	Matrix getSubmatrix(int n, int row, int col) const {
		Matrix dest(std::min(n, N()), std::min(n, N()));
		for (int i = row; i < std::min(row + n, N()); i++) {
			for (int j = col; j < std::min(col + n, N()); j++) {
				dest[i - row][j - col] = (*this)[i][j];
			}
		}

		return dest;
	}

	int N() const {
		return this->size();
	}

	int M() const {
		return N() == 0 ? 0 : (*this)[0].size();
	}

	Matrix operator*(const Matrix& other) const {
		Matrix res(N(), other.M());
		for (int i = 0; i < N(); i++) {
			for (int j = 0; j < M(); j++) {
				for (int k = 0; k < other.M(); k++) {
					res[i][k] += (*this)[i][j] * other[j][k];
				}
			}
		}
		return res;
	}

	Matrix operator*(double val) const {
		Matrix res = (*this);
		for (int i = 0; i < N(); i++) {
			for (int j = 0; j < M(); j++) {
				res[i][j] *= val;
			}
		}
		return res;
	}

	void Expand(const Matrix& m1) {
		int n = N();
		int m = M();
		for (int i = 0; i < m1.N(); i++) {
			(*this)[i].resize(m + m1.M());
			for (int j = 0; j < m1.M(); j++) {
				(*this)[i][j + m] = m1[i][j];
			}
		}
	}

	void Expand(const Matrix& m1, const Matrix& m2) {
		int n = N();
		int m = M();
		(*this).resize(n + m1.size());
		for (int i = 0; i < N(); i++) {
			(*this)[i].resize(m + m2.size());
		}
		for (int i = 0; i < m1.M(); i++) {
			for (int j = 0; j < m1.N(); j++) {
				(*this)[i][j + m] = m1[j][i];
			}
		}
		for (int i = 0; i < m1.N(); i++) {
			for (int j = 0; j < m1.M(); j++) {
				(*this)[i + n][j] = m1[i][j];
			}
		}
		for (int i = 0; i < m2.N(); i++) {
			for (int j = 0; j < m2.M(); j++) {
				(*this)[i + n][j + m] = m2[i][j];
			}
		}
	}
};

inline Matrix operator+(const Matrix& a, const Matrix& b) {
	Matrix res(a.N(), a.M());
	for (int i = 0; i < a.N(); i++) {
		for (int j = 0; j < a.M(); j++) {
			res[i][j] = a[i][j] + b[i][j];
		}
	}
	return res;
}

inline Matrix operator-(const Matrix& a, const Matrix& b) {
	Matrix res(a.N(), a.M());
	for (int i = 0; i < a.N(); i++) {
		for (int j = 0; j < a.M(); j++) {
			res[i][j] = a[i][j] - b[i][j];
		}
	}
	return res;
}

inline Vector operator*(const Matrix& a, const Vector& b) {
	Vector res(a.N());
	for (int i = 0; i < a.N(); i++) {
		for (int j = 0; j < a.M(); j++) {
			res[i] += a[i][j] * b[j];
		}
	}
	return res;
}

Matrix MatrixUnion(const Matrix& a, const Matrix& b);

class Robot {
public:
	Robot(const Vector& x, 
		const std::function<Vector(const Vector&, const Vector&)>& F,
		const std::function<Matrix(const Vector&)>& F_x,
		const std::function<Matrix(const Vector&)>& F_n) :
		x_(x), F_(F), F_x_(F_x), F_n_(F_n) {
	}

	void Move(Vector u, double delta_t) {
		x_ += F_(x_, u) * delta_t;
	}

	Vector Pose() const {
		return Vector(x_[0], x_[1]);
	}

	const Vector& State() const {
		return x_;
	}

	Matrix F_x() {
		return F_x_(x_);
	}

	void UpdateState(const Vector& x) {
		x_ = x;
	}

	void Normalize() {
		if (x_[3] > pi) {
			x_[3] -= 2 * pi;
		}
		if (x_[3] < -pi) {
			x_[3] += 2 * pi;
		}
	}

private:
	Vector x_;
	std::function<Vector(const Vector&, const Vector&)> F_;
	std::function<Matrix(const Vector&)> F_x_;
	std::function<Matrix(const Vector&)> F_n_;
};

Vector ToUnitVector(const Vector& v);

Robot CreateRobot2D(const Vector& x);

Robot CreateRobotDirectional(const Vector& x);
