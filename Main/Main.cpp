#include <iostream>

#include "../EKF-SLAM/Landmarks.h"
#include "../EKF-SLAM/Robot.h"
#include "../EKF-SLAM/EkfSlam.h"
#include "../LibPngUtils/PngImage.h"

int main() {
	EkfSlam solver;
	solver.Solve2();

	system("C:\\Users\\Alex\\Downloads\\ffmpeg-4.3.2-2021-02-27-full_build\\ffmpeg-4.3.2-2021-02-27-full_build\\bin\\ffmpeg.exe -r 40 -i \"C:\\Users\\Alex\\source\\repos\\EKF-SLAM\\Main\\out\\test_%d.png\" -vcodec mpeg4 -y \"C:\\Users\\Alex\\source\\repos\\EKF-SLAM\\Main\\movie.mp4\"");

	//int width = 500;
	//int height = 500;
	//PngImage image = createWhite(width, height);
	//image.SetDimensions(0.0, 10.0, 0.0, 10.0);

	//auto landmarks = GenerateLandmarks(0.0, 10.0, 0.0, 10.0, 50);

	//for (auto landmark : landmarks) {
	//	image.DrawPoint(Point(landmark.first, landmark.second), 3, 0, 0, 0);
	//}

	//double r0 = 4.0;
	//double u = 0.5;
	//double v0 = r0 * sqrt(u / r0);
	//Robot robot(Vector(0.0, r0), Vector(v0, 0.0));

	//double delta_t = 0.02;
	//for (int i = 0; i < 1000; i++) {
	//	Vector last_pose = robot.Pose();
	//	Vector a = ToUnitVector(robot.Pose()) * (-u);
	//	robot.Move(a, delta_t);
	//	image.DrawSegment(Point(robot.Pose().x_ + 5.0, robot.Pose().y_ +5.0),
	//		Point(last_pose.x_ + 5.0, last_pose.y_ + 5.0), 5, 255, 0, 0);
	//}

	//int result = image.writeImage(_strdup("test.png"), _strdup("This is my test image"));
}
