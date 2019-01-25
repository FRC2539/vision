#include <opencv2/opencv.hpp>
#include <networktables/NetworkTable.h>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>


struct Target {
	double error;
	int width;
	int position;
};

int main(int, char**)
{
	cv::VideoCapture stream(0); // open the default camera
	if(!stream.isOpened())  // check if we succeeded
	{
		std::cerr << "Could not open camera\n";
		return -1;
	}

//	NetworkTable::SetIPAddress("roborio-2539-frc.local");
	NetworkTable::SetTeam(2539);
	NetworkTable::SetClientMode();
	std::shared_ptr<NetworkTable> targetInfo =
		NetworkTable::GetTable("cameraTarget");

	std::vector<int> lowerHSVthreshold = {41, 0, 149};
	std::vector<int> upperHSVthreshold = {180, 203, 255};
	double targetAspectRatio = 12.0 / 20.0;
	cv::Mat image;
	cv::Mat newImage;

	//while ( ! targetInfo->GetBoolean("shutdownJetson", false))
	for (int j = 0; j < 100; j++)
	{
		/*do {
			image = newImage;
		} while (stream.read(newImage));*/
		stream.read(image);
		cv::cvtColor(image, image, CV_BGR2HSV);
		cv::inRange(image, lowerHSVthreshold, upperHSVthreshold, image);

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(
			image,
			contours,
			hierarchy,
			CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE
		);

		std::vector<Target> targets;
		for (auto contour : contours)
		{
			std::vector<cv::Point> hull;
			cv::convexHull(cv::Mat(contour), hull);
			if (cv::contourArea(hull) <= 200)
			{
				continue;
			}

			cv::RotatedRect box = cv::minAreaRect(contour);

			double aspectRatio = box.size.height / box.size.width;
			double error = std::abs(targetAspectRatio - aspectRatio);
			if (error > 0.4)
			{
				continue;
			}

			Target currentTarget = Target();
			currentTarget.width = box.size.width;
			currentTarget.position = box.center.x - image.cols / 2.0;
			currentTarget.error = error;

			targets.push_back(currentTarget);
		}

		if (targets.size() == 0)
		{
			targetInfo->PutBoolean("hasTarget", false);
			//std::cout << "notarget\n";
			continue;
		}
		std::cout << "target\n";

		int bestIndex = 0;
		int bestError = 1;
		int i = 0;
		for (auto target : targets)
		{
			if (target.error < bestError)
			{
				bestIndex = i;
			}
			i++;
		}
		std::cout << bestIndex << "\n";

		targetInfo->PutBoolean("hasTarget", true);
		targetInfo->PutNumber("centerX", targets[bestIndex].position);
		targetInfo->PutNumber(
			"distance",
			22.837262 - 0.210646 * targets[bestIndex].width
		);

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}
