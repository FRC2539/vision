/**
    An OpenCV plugin to find targets for Steamworks
*/
#include <networktables/NetworkTable.h>
#include <vector>
#include <opencv2/opencv.hpp>

// exports for the filter
extern "C" {
    bool filter_init(const char * args, void** filter_ctx);
    void filter_process(void* filter_ctx, cv::Mat &src, cv::Mat &dst);
    void filter_free(void* filter_ctx);
}

struct Target {
    double error;
    int width;
    int position;
};

// Forward-declare finder functions
void findLift(cv::Mat &output, std::vector<std::vector<cv::Point>> contours);

// Globals are better than passing by pointer-to-pointer
std::shared_ptr<NetworkTable> targetInfo;

std::vector<int> lowerHSVthreshold = {70, 6, 70};
std::vector<int> upperHSVthreshold = {108, 82, 76};
double targetAspect = 12.0/20.0;

/**
    Initializes the filter. If you return something, it will be passed to the
    filter_process function, and should be freed by the filter_free function
*/
bool filter_init(const char * args, void** filter_ctx) {
   
    NetworkTable::SetTeam(2539);
    NetworkTable::SetClientMode();
    NetworkTable::SetUpdateRate(0.01);
    targetInfo = NetworkTable::GetTable("cameraTarget");

    return true;
}

/**
    Called by the OpenCV plugin upon each frame
*/
void filter_process(void* filter_ctx, cv::Mat &src, cv::Mat &dst) {
    dst=src;

    cv::Mat image, image2;

    cv::normalize(src, image, 75.0, 0.0, cv::NORM_INF);

    cv::bilateralFilter(image, image2, -1, 1, 1);

    cv::cvtColor(image2, image, CV_BGR2HSV);
    cv::inRange(image, lowerHSVthreshold, upperHSVthreshold, image);

	cv::Mat cvErodeKernel;
	cv::Point cvErodeAnchor(-1, -1);
	cv::Scalar cvErodeBorderValue(-1);
    cv::erode(image, image2, cvErodeKernel, cvErodeAnchor, 2, cv::BORDER_CONSTANT, cvErodeBorderValue);

	cv::Mat cvDilateKernel;
	cv::Point cvDilateAnchor(-1, -1);
	cv::Scalar cvDilateBorderValue(-1);
    cv::dilate(image2, image, cvDilateKernel, cvDilateAnchor, 5, cv::BORDER_CONSTANT, cvDilateBorderValue);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(
        image,
        contours,
        hierarchy,
        CV_RETR_TREE,
        CV_CHAIN_APPROX_SIMPLE
    );

    cv::Scalar green(255, 0, 0);
    cv::drawContours(dst, contours, -1, green);

    findLift(dst, contours);
}

void findTower(cv::Mat &output, std::vector<std::vector<cv::Point>> contours)
{
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
        double error = std::abs(targetAspect - aspectRatio);
        if (error > 0.4)
        {
            continue;
        }

        Target currentTarget = Target();
        currentTarget.width = box.size.width;
        currentTarget.position = box.center.x - output.cols / 2.0;
        currentTarget.error = error;

        targets.push_back(currentTarget);
    }

    if (targets.size() == 0)
    {
        targetInfo->PutBoolean("hasTarget", false);
        return;
    }

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

    targetInfo->PutBoolean("hasTarget", true);
    targetInfo->PutNumber("centerX", targets[bestIndex].position);
    targetInfo->PutNumber(
        "distance",
        22.837262 - 0.210646 * targets[bestIndex].width
    );
}

void findLift(cv::Mat &output, std::vector<std::vector<cv::Point>> contours)
{
    std::vector<cv::Point> hull;
    std::vector<Target> targets;
    std::vector<std::vector<cv::Point>> filteredContours;
    for (std::vector<cv::Point> contour: contours)
    {
        cv::Rect bb = boundingRect(contour);
        if (bb.width < 2.0) continue;
        if (bb.height < 2.0) continue;
        double area = cv::contourArea(contour);
        if (area < 80.0) continue;
        cv::convexHull(cv::Mat(contour, true), hull);
        double solid = 100 * area / cv::contourArea(hull);
        if (solid < 75.0) continue;
        double ratio = bb.width / bb.height;
        if (ratio < 0.0 || ratio > 0.2) continue;

        cv::RotatedRect box = cv::minAreaRect(contour);
        Target currentTarget = Target();
        currentTarget.width = box.size.width;
        currentTarget.position = box.center.x - output.cols / 2.0;

        filteredContours.push_back(contour);
        targets.push_back(currentTarget);
    }

    if (targets.size() == 0)
    {
        targetInfo->PutBoolean("liftVisible", false);
        return;
    }

    targetInfo->PutBoolean("liftVisible", true);
    targetInfo->PutNumber("liftCenter", (targets[0].position + targets[1].position) / 2);

"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
    //GET DISTANCE WORKING! (currently wrong)
    targetInfo->PutNumber("distance", std::abs(targets[0].position - targets[1].position));
"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"

    targetInfo->PutNumber("Left", targets[0].position);
    targetInfo->PutNumber("Right", targets[1].position);

    cv::Scalar purple(255, 0, 255);
    cv::drawContours(output, filteredContours, -1, purple, 2);
}

void findBoiler(cv::Mat &output, std::vector<std::vector<cv::Point>> contours)
{

}

/**
    Called when the input plugin is cleaning up
*/
void filter_free(void* filter_ctx) {}
