#include "EkfSlam.h"

#include "math.h"
#include <sstream>
#include <iostream>
#include <map>

#include "Robot.h"
#include "Landmarks.h"
#include "../LibPngUtils/PngImage.h"

double UniformRand() {
	return (double)rand() / (RAND_MAX);
}

void ApplyRandomNoise(Vector& obs, const std::vector<double>& s) {
	obs[0] += s[0] * (2 * UniformRand() - 1.0);
	obs[1] += s[1] * (2 * UniformRand() - 1.0);
}

double distance2d(const Vector& a, const Vector& b) {
	return sqrt((a[0] - b[0])* (a[0] - b[0]) + (a[1] - b[1])* (a[1] - b[1]));
}

void DrawImage(int i, const std::vector<Vector>& robot, const std::vector<Vector>& sim_robot, 
	const std::vector<Vector>& naive_robot_positions, 
	const std::vector < std::pair<double, double> >& landmarks, double detection_radius) {
	int width = 500;
	int height = 500;
	PngImage image = createWhite(width, height);
	image.SetDimensions(0.0, 10.0, 0.0, 10.0);

	for (auto landmark : landmarks) {
		image.DrawPoint(Point(landmark.first + 5.0, landmark.second + 5.0), 3, 0, 0, 0);
	}

	for (int i = 1; i < robot.size(); i++) {
		image.DrawSegment(Point(robot[i][0] + 5.0, robot[i][1] + 5.0),
			Point(robot[i - 1][0] + 5.0, robot[i - 1][1] + 5.0), 5, 255, 0, 0);
		image.DrawSegment(Point(sim_robot[i][0] + 5.0, sim_robot[i][1] + 5.0),
			Point(sim_robot[i - 1][0] + 5.0, sim_robot[i - 1][1] + 5.0), 5, 0, 0, 0);
		image.DrawSegment(Point(naive_robot_positions[i][0] + 5.0, naive_robot_positions[i][1] + 5.0),
			Point(naive_robot_positions[i - 1][0] + 5.0, naive_robot_positions[i - 1][1] + 5.0), 3, 0, 0, 255);
	}
	int idx = robot.size() - 1;

	image.DrawCircle(Point(sim_robot[idx][0] + 5.0, sim_robot[idx][1] + 5.0), detection_radius, 0, 0, 0);

	std::vector<int> lids = FindLandmarks(sim_robot[idx][0], sim_robot[idx][1], detection_radius, landmarks);

	for (int i = 0; i < lids.size(); i++) {
		image.DrawSegment(Point(sim_robot[idx][0] + 5.0, sim_robot[idx][1] + 5.0),
			Point(landmarks[lids[i]].first + 5.0, landmarks[lids[i]].second + 5.0), 20, 0, 0, 255);
	}

	std::stringstream ss;
	ss << "./out/test_" << i << ".png";
	int result = image.writeImage(_strdup(ss.str().c_str()), _strdup(""));
}

void EkfSlam::Solve() {
	double r0 = 4.0;
	double u = 0.5;
	double v0 = r0 * sqrt(u / r0);
	//Robot robot = CreateRobot2D(Vector({ 0.0, r0, v0, 0.0 }));
	//Robot sim_robot = CreateRobot2D(Vector({ 0.0, r0, v0, 0.0 }));

	Robot robot = CreateRobotDirectional(Vector({ 0.0, r0, v0, 0.0 }));
	Robot sim_robot = CreateRobotDirectional(Vector({ 0.0, r0, v0, 0.0 }));
	Robot naive_robot = CreateRobotDirectional(Vector({ 0.0, r0, v0, 0.0 }));

	double detection_radius = 1.0;
	auto landmarks = GenerateLandmarks(-5.0, 5.0, -5.0, 5.0, 100);

	int n_dim = 4;
	int n_landmarks = landmarks.size();

	Matrix P_xx(n_dim, n_dim);
	Matrix P_xm(n_dim, n_landmarks);
	Matrix P_mx(n_landmarks, n_dim);

	// System noise: Gaussian
	std::vector<double> q = { 0.1, 0.2 };
	Matrix Q(4, 4);
	Q[0][0] = q[0] * q[0];
	Q[1][1] = q[1] * q[1];
	Q[2][2] = q[0] * q[0];
	Q[3][3] = q[1] * q[1];

	// Measurement noise
	std::vector<double>s = { 0.01, 0.01 };
	Matrix S(4, 4);
	S[0][0] = s[0] * s[0];
	S[1][1] = s[1] * s[1];
	S[2][2] = s[0] * s[0];
	S[3][3] = s[1] * s[1];

	//Matrix H_x(2, 4);
	//H_x[0][0] = -1;
	//H_x[1][1] = -1;

	ObservationModel observation_model = CreateObservationModel();

	Matrix R(2, 2);

	std::vector<Vector> robot_positions({ robot.Pose() });
	std::vector<Vector> sim_robot_positions({ sim_robot.Pose() });
	std::vector<Vector> naive_robot_positions({ sim_robot.Pose() });

	double delta_t = 0.02;
	for (int it = 0; it < 1000; it++) {
		std::cout << "Iteration: " << it << std::endl;
		Vector last_sim_pose = sim_robot.Pose();
		Vector last_pose = robot.Pose();

		// Simulator
		// Motion
		//Vector a = ToUnitVector(sim_robot.Pose()) * (-u);
		//Vector a(std::vector<double>({ 0.0, -u }));
		//sim_robot.Move(a, delta_t);

		Vector a = Vector(std::vector<double>({ 0.0, -u }));
		ApplyRandomNoise(a, q);
		sim_robot.Move(a, delta_t);
		sim_robot.Normalize();

		//image.DrawSegment(Point(robot.Pose().x_ + 5.0, robot.Pose().y_ + 5.0),
		//	Point(last_pose.x_ + 5.0, last_pose.y_ + 5.0), 5, 255, 0, 0);
	
		// P_xx = robot.F_x * P_xx * robot.F_x.Transpose() + Q;

		// Observations
		std::vector<int> lids = FindLandmarks(sim_robot.Pose()[0], sim_robot.Pose()[1], detection_radius, landmarks);
		std::vector<Vector> observations;
		for (int i = 0; i < lids.size(); i++) {
			Vector obs = observation_model.H(sim_robot.Pose(), Vector(landmarks[lids[i]].first, landmarks[lids[i]].second));
			ApplyRandomNoise(obs, s);
			observations.push_back(obs);
		}
	
		// Estimator
		// Predicion - robot motion
		//a = ToUnitVector(robot.Pose()) * (-u);
		a = Vector(std::vector<double>({ 0.0, -u }));
		//ApplyRandomNoise(a, q);
		robot.Move(a, delta_t);
		robot.Normalize();

		P_xm = robot.F_x() * P_xm;
		P_mx = P_xm.Transpose();
		P_xx = robot.F_x() * P_xx * robot.F_x().Transpose() + Q;

		std::cout << "Distance: " << distance2d(robot.Pose(), sim_robot.Pose()) << std::endl;

		// Landmark correction - known landmarks
		for (int i = 0; i < lids.size(); i++) {
			Vector x = robot.Pose();
			const std::pair<double, double>& landmark_p = landmarks[lids[i]];

			Vector landmark = Vector(landmark_p.first, landmark_p.second);

			Vector h_i = observation_model.H(x, landmark);

			Vector y_i = observations[i];

			Vector z_bar = y_i - h_i;

			if (z_bar[1] > pi) {
				z_bar[1] -= 2 * pi;
			}
			if (z_bar[1] < -pi) {
				z_bar[1] += 2 * pi;
			}

			Matrix H_x = observation_model.H_x(x, Vector(landmark_p.first, landmark_p.second));

			Matrix Z = H_x * P_xx * H_x.Transpose() + S;

			Matrix K = P_xx * H_x.Transpose() * Z.Inverse();

			robot.UpdateState(robot.State() + K * z_bar.Resize(4));
			robot.Normalize();

			P_xx = P_xx - K * Z * K.Transpose();

		}

		robot_positions.push_back(robot.Pose());
		sim_robot_positions.push_back(sim_robot.Pose());
		naive_robot_positions.push_back(naive_robot.Pose());

		DrawImage(it, robot_positions, sim_robot_positions, naive_robot_positions, landmarks, detection_radius);
	}

}

void EkfSlam::Solve2() {
	double r0 = 4.0;
	double u = 0.5;
	double v0 = r0 * sqrt(u / r0);
	//Robot robot = CreateRobot2D(Vector({ 0.0, r0, v0, 0.0 }));
	//Robot sim_robot = CreateRobot2D(Vector({ 0.0, r0, v0, 0.0 }));

	Robot robot = CreateRobotDirectional(Vector({ 0.0, r0, v0, 0.0 }));
	Robot sim_robot = CreateRobotDirectional(Vector({ 0.0, r0, v0, 0.0 }));
	Robot naive_robot = CreateRobotDirectional(Vector({ 0.0, r0, v0, 0.0 }));

	double detection_radius = 1.0;
	auto landmarks = GenerateLandmarks(-5.0, 5.0, -5.0, 5.0, 100);

	int n_dim = 4;
	int n_landmarks = landmarks.size();

	Matrix P_xx(n_dim, n_dim);
	Matrix P_xm(n_dim, 0);
	Matrix P_mx(0, n_dim);
	Matrix P_mm(0, 0);

	// System noise: Gaussian
	std::vector<double> q = { 0.1, 0.2 };
	Matrix Q(4, 4);
	Q[0][0] = q[0] * q[0];
	Q[1][1] = q[1] * q[1];
	Q[2][2] = q[0] * q[0];
	Q[3][3] = q[1] * q[1];

	// Measurement noise
	std::vector<double> s = { 0.01, 0.01 };
	Matrix S(2, 2);
	S[0][0] = s[0] * s[0];
	S[1][1] = s[1] * s[1];

	//Matrix H_x(2, 4);
	//H_x[0][0] = -1;
	//H_x[1][1] = -1;

	ObservationModel observation_model = CreateObservationModel();

	Matrix R(2, 2);

	std::vector<Vector> robot_positions({ robot.Pose() });
	std::vector<Vector> sim_robot_positions({ sim_robot.Pose() });
	std::vector<Vector> naive_robot_positions({ naive_robot.Pose() });

	std::map<int, int> landmark_ids;

	Vector xl;

	double delta_t = 0.02;
	int cnt = 0;
	for (int it = 0; it < 1000; it++) {
		std::cout << "Iteration: " << it << std::endl;
		Vector last_naive_pose = naive_robot.Pose();
		Vector last_pose = robot.Pose();

		// Simulator
		// Motion
		//Vector a = ToUnitVector(sim_robot.Pose()) * (-u);
		Vector a(std::vector<double>({ 0.0, -u }));
		naive_robot.Move(a, delta_t);

		a = Vector(std::vector<double>({ 0.0, -u }));
		ApplyRandomNoise(a, q);
		sim_robot.Move(a, delta_t);

		//image.DrawSegment(Point(robot.Pose().x_ + 5.0, robot.Pose().y_ + 5.0),
		//	Point(last_pose.x_ + 5.0, last_pose.y_ + 5.0), 5, 255, 0, 0);

		// P_xx = robot.F_x * P_xx * robot.F_x.Transpose() + Q;

		// Observations
		std::vector<int> lids = FindLandmarks(sim_robot.Pose()[0], sim_robot.Pose()[1], detection_radius, landmarks);
		std::vector<Vector> observations;
		for (int i = 0; i < lids.size(); i++) {
			Vector obs = observation_model.H(sim_robot.Pose(), Vector(landmarks[lids[i]].first, landmarks[lids[i]].second));
			ApplyRandomNoise(obs, s);
			observations.push_back(obs);
		}

		// Estimator
		// Predicion - robot motion
		//a = ToUnitVector(robot.Pose()) * (-u);
		//a = Vector(std::vector<double>({ 0.0, -u }));
		//ApplyRandomNoise(a, q);
		//sim_robot.Move(a, delta_t);

		a = Vector(std::vector<double>({ 0.0, -u }));
		//ApplyRandomNoise(a, q);
		robot.Move(a, delta_t);

		P_xm = robot.F_x() * P_xm;
		// P_mm = P_mm;
		P_mx = P_xm.Transpose();
		P_xx = robot.F_x() * P_xx * robot.F_x().Transpose() + Q;

		std::cout << "Distance: " << distance2d(robot.Pose(), sim_robot.Pose()) << std::endl;

		// Landmark correction - known landmarks
		for (int i = 0; i < lids.size(); i++) {
			Vector x = robot.Pose();
			int id = lids[i];
			if (landmark_ids.find(id) != landmark_ids.end()) {
				int p_matrix_id = landmark_ids[id];
				const std::pair<double, double>& landmark_p = landmarks[lids[i]];
				//Vector landmark = Vector(landmark_p.first, landmark_p.second);
				Vector landmark = Vector(xl[2 * p_matrix_id], xl[2 * p_matrix_id + 1]);

				Vector h_i = observation_model.H(x, landmark);

				Vector y_i = observations[i];

				Vector z_bar = y_i - h_i;

				if (z_bar[1] > pi) {
					z_bar[1] -= 2 * pi;
				}
				if (z_bar[1] < -pi) {
					z_bar[1] += 2 * pi;
				}

				Matrix H_x = observation_model.H_x(x, landmark);

				Matrix H_l = observation_model.H_l(x, landmark);

				Matrix H_x_t = H_x.Transpose();

				Matrix H_l_t = H_l.Transpose();

				Matrix P_xl = P_xm.Submatrix(0, 2* p_matrix_id, 4, 2);
				Matrix P_lx = P_xl.Transpose();
				Matrix P_ll = P_mm.Submatrix(2 * p_matrix_id, 2 * p_matrix_id, 2, 2);

				Matrix Z = H_x * P_xx * H_x_t +
					H_l * P_lx * H_x_t +
					H_x * P_xl * H_l_t +
					H_l * P_ll * H_l_t + S;

				Matrix K = MatrixUnion(P_xx * H_x_t + P_xl * H_l_t,
					P_lx * H_x_t + P_ll * H_l_t) * Z.Inverse();

				Vector x_update = K * z_bar;
				robot.UpdateState(robot.State() + x_update.Subarray(0, 4));
				xl.Set(2* p_matrix_id, xl.Subarray(2* p_matrix_id, 2) + x_update.Subarray(4, 2));

				P_xx = P_xx - K * Z * K.Transpose();
			}
			else {
				landmark_ids[id] = cnt++;

				Vector y_i = observations[i];

				Vector g_i = observation_model.G(x, y_i);

				Matrix G_x = observation_model.G_x(x, y_i);

				Matrix G_y = observation_model.G_y(x, y_i);

				Matrix G_x_t = G_x.Transpose();

				Matrix G_y_t = G_y.Transpose();

				Matrix P_ll = G_x * P_xx * G_x_t + G_y * S * G_y_t;
				Matrix P_lx_1 = G_x * P_xx;
				Matrix P_lx_2 = G_x * P_xm;

				xl.Append(g_i);
				P_xm.Expand(P_lx_1.Transpose());
				P_mm.Expand(P_lx_2, P_ll);
			}

		}

		if (distance2d(robot.Pose(), sim_robot.Pose()) > 0.1) {
			int hh = 0;
			hh++;
		}

		robot_positions.push_back(robot.Pose());
		sim_robot_positions.push_back(sim_robot.Pose());
		naive_robot_positions.push_back(naive_robot.Pose());

		DrawImage(it, robot_positions, sim_robot_positions, naive_robot_positions, landmarks, detection_radius);
	}

}