/**
    An OpenCV plugin to find targets for Steamworks
*/
#include <networktables/NetworkTable.h>
#include <vector>
#include <deque>
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

namespace color {
    cv::Scalar red(0, 0, 255);
    cv::Scalar green(0, 255, 0);
    cv::Scalar blue(255, 0, 0);
    cv::Scalar yellow(0, 255, 255);
    cv::Scalar purple(255, 0, 127);
    cv::Scalar orange(0, 127, 255);
    cv::Scalar black(0, 0, 0);
    cv::Scalar white(255, 255, 255);
    cv::Scalar gray(127, 127, 127);
}

/**
    Initializes the filter. If you return something, it will be passed to the
    filter_process function, and should be freed by the filter_free function
*/
bool filter_init(const char * args, void** filter_ctx) {
   
    NetworkTable::SetTeam(2539);
    NetworkTable::SetClientMode();
    //NetworkTable::SetUpdateRate(0.01);
    targetInfo = NetworkTable::GetTable("cameraTarget");

    return true;
}

/**
    Called by the OpenCV plugin upon each frame
*/
void filter_process(void* filter_ctx, cv::Mat &src, cv::Mat &dst) {
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

    cv::drawContours(src, contours, -1, color::blue);

    findLift(src, contours);

    cv::resize(src, dst, cv::Size(480, 360), 0, 0, cv::INTER_AREA);
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
        double error = std::abs(12.0/20.0 - aspectRatio);
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

bool leftToRight(cv::Rect i, cv::Rect j)
{
    return i.x < j.x;
}

bool bigToLittle(cv::Rect i, cv::Rect j)
{
    return i.height < j.height;
}

void findLift(cv::Mat &output, std::vector<std::vector<cv::Point>> contours)
{
    std::vector<cv::Point> hull;
    std::deque<cv::Rect> matchingBoxes;
    for (std::vector<cv::Point> contour: contours)
    {
        cv::RotatedRect box = cv::minAreaRect(contour);

        // Ignore if too skinny
        if (box.size.width < 2.0 || box.size.height < 2.0) continue;

        // Ignore if too small
        double area = cv::contourArea(contour);
        if (area < 30.0) continue;

        // Ignore if too rotated (currently commented out to keep it working properly)
        //if (box.angle < -10 || box.angle > 10) continue;

        // Ignore if too concave
        cv::convexHull(cv::Mat(contour, true), hull);
        double solid = 100 * area / cv::contourArea(hull);
        if (solid < 70.0) continue;

        // Ignore if wrong shape
        double ratio = box.size.width / box.size.height;
        if (ratio > 0.75) continue;

        matchingBoxes.push_back(cv::boundingRect(contour));
    }

    // We need at least 2 boxes to make a target
    if (matchingBoxes.size() < 2)
    {
        targetInfo->PutBoolean("liftVisible", false);
        return;
    }

    // Check if any two remaining rectangles create a lift target
    std::sort(matchingBoxes.begin(), matchingBoxes.end(), leftToRight);

    std::vector<cv::Rect> lifts;
    while (matchingBoxes.size() > 1)
    {
        auto box1 = matchingBoxes.front();
        matchingBoxes.pop_front();

        cv::rectangle(output, box1, color::green, 2);

        for (auto box2 : matchingBoxes)
        {
            // Are the boxes next to each other?
            if (std::abs(box1.y - box2.y) > .1 * box1.height) continue;

            // Are the boxes the same size?
            if (std::abs(box1.height - box2.height) > .1 * box1.height) continue;

            // Are the boxes the right distance apart?
            std::vector<cv::Point> combined;
            combined.push_back(cv::Point(box1.x, box1.y));
            combined.push_back(cv::Point(box1.x, box1.y + box1.height));
            combined.push_back(cv::Point(box2.x + box2.width, box2.y + box2.height));
            combined.push_back(cv::Point(box2.x + box2.width, box2.y));

            cv::Rect lift = cv::boundingRect(combined);
            double ratio = lift.width / lift.height;
            if (ratio > 3 || ratio < 1) continue;

            lifts.push_back(lift);
        }
    }
    // Draw last rectangle
    cv::rectangle(output, matchingBoxes.front(), color::green, 2);

    if (lifts.size() == 0)
    {
        targetInfo->PutBoolean("liftVisible", false);
        return;
    }

    std::sort(lifts.begin(), lifts.end(), bigToLittle);

    targetInfo->PutBoolean("liftVisible", true);
    targetInfo->PutNumber(
        "liftCenter",
        lifts[0].x + lifts[0].width / 2.0 - output.cols / 2.0
    );

/*XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX*/
    //GET DISTANCE WORKING! (currently uses made up slope and intercept)
    double slope = 0.5;
    double intercept = 1;
    targetInfo->PutNumber("liftDistance", slope * lifts[0].width + intercept);
/*XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX*/

    // Debugging
    targetInfo->PutNumber("Width", lifts[0].width);
    targetInfo->PutNumber("Left", lifts[0].x);
    targetInfo->PutNumber("Right", lifts[0].x + lifts[0].width);

    // Highlight targeted lift on screen
    cv::rectangle(output, lifts[0], color::yellow, 3);
}

void findBoiler(cv::Mat &output, std::vector<std::vector<cv::Point>> contours)
{

}

/**
    Called when the input plugin is cleaning up
*/
void filter_free(void* filter_ctx) {}
