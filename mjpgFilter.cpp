/**
    An OpenCV plugin to find targets for Steamworks
*/
#include <networktables/NetworkTable.h>
#include <vector>
#include <deque>
#include <opencv2/opencv.hpp>
#include <cmath>

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
void findBoiler(cv::Mat &output, std::vector<std::vector<cv::Point>> contours);

// Globals are better than passing by pointer-to-pointer
std::shared_ptr<NetworkTable> targetInfo;

std::vector<int> lowerHSVthreshold = {0, 0, 235};
std::vector<int> upperHSVthreshold = {255, 20, 255};
cv::Mat cameraMatrix, distCoeffs;

namespace color {
    cv::Scalar gold(50, 215, 255);
    cv::Scalar turquoise(208, 224, 64);
    cv::Scalar red(0, 0, 255);
    cv::Scalar green(0, 255, 0);
    cv::Scalar blue(255, 0, 0);
    cv::Scalar yellow(0, 255, 255);
    cv::Scalar pink(255, 0, 255);
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

    // Read camera settings
    cv::FileStorage fs("out_camera_data.xml", cv::FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        std::cout << "Could not open the configuration file: \"out_camera_data.xml\"\n";
        return -1;
    }

    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;
    fs.release();

    return true;
}

/**
    Called by the OpenCV plugin upon each frame
*/
void filter_process(void* filter_ctx, cv::Mat &src, cv::Mat &dst) {

    cv::Mat image, image2, corrected;

    cv::undistort(src, corrected, cameraMatrix, distCoeffs);

    cv::cvtColor(corrected, image, CV_BGR2HSV);
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

#if defined(DEBUG)
    cv::drawContours(corrected, contours, -1, color::blue);
#endif

#if DIRECTION_BOTH
    findBoiler(corrected, contours);
    findLift(corrected, contours);
#elif DIRECTION_FRONT
    findBoiler(corrected, contours);
#elif DIRECTION_BACK
    findLift(corrected, contours);
#else
    #error No camera direction defined
#endif

    cv::resize(corrected, dst, cv::Size(480, 360), 0, 0, cv::INTER_AREA);
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
        //if (box.angle < -15 || box.angle > 15) continue;

        // Ignore if too concave
        cv::convexHull(cv::Mat(contour, true), hull);
        double solid = 100 * area / cv::contourArea(hull);
        if (solid < 85.0) continue;

        // Ignore if wrong shape
        double ratio = box.size.width / box.size.height;
//        if (ratio > 0.7) continue;

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

#if defined(DEBUG)
        cv::rectangle(output, box1, color::green, 1);
#endif

        for (auto box2 : matchingBoxes)
        {
            // Are the boxes next to each other?
            if (std::abs(box1.y - box2.y) > .2 * box1.height) continue;

            // Are the boxes the same size?
            if (std::abs(box1.height - box2.height) > .2 * box1.height) continue;

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

#if defined(DEBUG)
    // Draw last rectangle
    cv::rectangle(output, matchingBoxes.front(), color::green, 1);
#endif

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

    //Calculate distance from lift. Not always accurate, but consistent.
    double A = 98.6132;
    double B = -0.00715589;
    targetInfo->PutNumber("liftDistance",  int(A * pow(2.71828182845904523536, B * lifts[0].width)));

    // Debugging
    targetInfo->PutNumber("Width", lifts[0].width);
    targetInfo->PutNumber("Left", lifts[0].x);
    targetInfo->PutNumber("Right", lifts[0].x + lifts[0].width);

    // Highlight targeted lift on screen
    cv::rectangle(output, lifts[0], color::pink, 10);
}

void findBoiler(cv::Mat &output, std::vector<std::vector<cv::Point>> contours)
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
        //if (box.angle < -15 || box.angle > 15) continue;

        // Ignore if too concave
        cv::convexHull(cv::Mat(contour, true), hull);
        double solid = 100 * area / cv::contourArea(hull);
        if (solid < 45.0) continue;

        // Ignore if wrong shape
        double ratio = box.size.width / box.size.height;
//        if (ratio > 0.7) continue;

        matchingBoxes.push_back(cv::boundingRect(contour));
    }

    // We need at least 2 boxes to make a target
    if (matchingBoxes.size() < 2)
    {
        targetInfo->PutBoolean("boilerVisible", false);
        return;
    }

    // Check if any two remaining rectangles create a lift target
    std::sort(matchingBoxes.begin(), matchingBoxes.end(), leftToRight);

    std::vector<cv::Rect> boilers;
    while (matchingBoxes.size() > 1)
    {
        auto box1 = matchingBoxes.front();
        matchingBoxes.pop_front();

#if defined(DEBUG)
        cv::rectangle(output, box1, color::orange, 1);
#endif

        for (auto box2 : matchingBoxes)
        {
            // Are the boxes on top of each other?
            if (std::abs(box1.x - box2.x) > .2 * box1.width) continue;

            // Are the boxes the same size?
            if (std::abs(box1.width - box2.width) > .4 * box1.width) continue;

            // Are the boxes the right distance apart?
            std::vector<cv::Point> combined;
            combined.push_back(cv::Point(box1.x, box1.y));
            combined.push_back(cv::Point(box1.x, box1.y + box1.height));
            combined.push_back(cv::Point(box2.x + box2.width, box2.y + box2.height));
            combined.push_back(cv::Point(box2.x + box2.width, box2.y));

            cv::Rect boiler = cv::boundingRect(combined);
            double ratio = boiler.width / boiler.height;
            if (ratio > 3 || ratio < 1) continue;

            boilers.push_back(boiler);
        }
    }

#if defined(DEBUG)
    // Draw last rectangle
    cv::rectangle(output, matchingBoxes.front(), color::orange, 1);
#endif

    if (boilers.size() == 0)
    {
        targetInfo->PutBoolean("boilerVisible", false);
        return;
    }

    std::sort(boilers.begin(), boilers.end(), bigToLittle);

    targetInfo->PutBoolean("boilerVisible", true);
    targetInfo->PutNumber(
        "boilerCenter",
        boilers[0].x + boilers[0].width / 2.0 - output.cols / 2.0
    );

/*XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX*/
    //GET DISTANCE WORKING! (currently uses made up slope and intercept)
    double slope = 0.5;
    double intercept = 1;
    targetInfo->PutNumber("boilerDistance", slope * boilers[0].width + intercept);
/*XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX*/

    // Debugging
    targetInfo->PutNumber("boilerWidth", boilers[0].width);
    targetInfo->PutNumber("top", boilers[0].y);
    targetInfo->PutNumber("bottom", boilers[0].y + boilers[0].height);

    // Highlight targeted lift on screen
    cv::rectangle(output, boilers[0], color::turquoise, 10);
}

/**
    Called when the input plugin is cleaning up
*/
void filter_free(void* filter_ctx) {}
