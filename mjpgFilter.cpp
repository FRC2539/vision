/**
    Example C++ OpenCV filter plugin that doesn't do anything. Copy/paste this
    to create your own awesome filter plugins for mjpg-streamer.
    
    At the moment, only the input_opencv.so plugin supports filter plugins.
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

// Globals are better than passing by pointer-to-pointer
std::shared_ptr<NetworkTable> targetInfo;
    
std::vector<int> lowerHSVthreshold = {41, 0, 149};
std::vector<int> upperHSVthreshold = {180, 203, 255};
double targetAspect = 12.0/20.0;

/**
    Initializes the filter. If you return something, it will be passed to the
    filter_process function, and should be freed by the filter_free function
*/
bool filter_init(const char * args, void** filter_ctx) {
   
    NetworkTable::SetTeam(2539);
    NetworkTable::SetClientMode();
    targetInfo = NetworkTable::GetTable("cameraTarget");

    return true;
}

/**
    Called by the OpenCV plugin upon each frame
*/
void filter_process(void* filter_ctx, cv::Mat &src, cv::Mat &dst) {
    dst=src;

    cv::Mat image;
    cv::cvtColor(src, image, CV_BGR2HSV);
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

    cv::Scalar green(0, 255, 0);
    cv::drawContours(dst, contours, -1, green);

    findTower(dst, contours);
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
        currentTarget.position = box.center.x - image.cols / 2.0;
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
    )
}

void findLift(cv::Mat &output, std::vector<std::vector<cv::Point>> contours)
{

}

void findBoiler(cv::Mat &output, std::vector<std::vector<cv::Point>> contours)
{

}

/**
    Called when the input plugin is cleaning up
*/
void filter_free(void* filter_ctx) {
    // empty
}
