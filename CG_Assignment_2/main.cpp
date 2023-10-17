#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		std::cout << "Left button of the mouse is clicked - position (" << x << ", "
		          << y << ")" << '\n';
		control_points.emplace_back(x, y);
	}
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)
{
	auto &p_0 = points[0];
	auto &p_1 = points[1];
	auto &p_2 = points[2];
	auto &p_3 = points[3];

	for (double t = 0.0; t <= 1.0; t += 0.001)
	{
		auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
		             3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

		window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
	}
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
	if (control_points.size() == 1)
	{
		return control_points[0];
	}
	else
	{
		std::vector<cv::Point2f> new_control_points;
		for (size_t i = 0; i < control_points.size() - 1; ++i)
		{
			cv::Point2f new_point = (1 - t) * control_points[i] + t * control_points[i + 1];
			new_control_points.push_back(new_point);
		}
		return recursive_bezier(new_control_points, t);
	}

	//return cv::Point2f();

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
	// TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
	// recursive Bezier algorithm.
	// Iterate through all t = 0 to t = 1 with small steps
	for (double t = 0.0; t <= 1.0; t += 0.001)
	{
		// Call recursive_bezier to get the point on the curve at t
		cv::Point2f point = recursive_bezier(control_points, t);

		// Calculate integer pixel coordinates
		int x = static_cast<int>(point.x);
		int y = static_cast<int>(point.y);

		// Calculate fractional parts of coordinates
		double xf = point.x - x;
		double yf = point.y - y;

		// Perform 2x2 anti-aliasing
		for (int dx = 0; dx <= 1; ++dx)
		{
			for (int dy = 0; dy <= 1; ++dy)
			{
				int px = x + dx;
				int py = y + dy;

				if (px >= 0 && px < window.cols && py >= 0 && py < window.rows)
				{
					double weight = (1 - std::abs(px - point.x)) * (1 - std::abs(py - point.y));
					window.at<cv::Vec3b>(py, px)[1] += static_cast<uchar>(weight * 255);
				}
			}
		}
	}
}

int main()
{
	cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
	cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
	cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

	cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

	int key = -1;
	while (key != 27)
	{
		window.setTo(0);
		for (auto &point : control_points)
		{
			cv::circle(window, point, 3, { 255, 255, 255 }, 3);
		}

		if (control_points.size() >= 4)
		{
			naive_bezier(control_points, window);
			bezier(control_points, window);

			cv::imshow("Bezier Curve", window);
			cv::imwrite("my_bezier_curve.png", window);
			key = cv::waitKey(1);

		}

		cv::imshow("Bezier Curve", window);
		key = cv::waitKey(20);
	}

	return 0;
}
