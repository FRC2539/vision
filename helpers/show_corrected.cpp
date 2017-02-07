#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
using namespace cv;
using namespace std;
int main(int, char**)
{
    Mat frame, corrected, cameraMatrix, distCoeffs;
    //--- INITIALIZE VIDEOCAPTURE
    VideoCapture cap;
    // Advance Usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID + apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    // Read camera settings
    FileStorage fs("out_camera_data.xml", FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"out_camera_data.xml\"\n";
        return -1;
    }

    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;
    fs.release();

    //--- GRAB AND WRITE LOOP
    cout << "Press Ctrl+C to terminate\n";
    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        undistort(frame, corrected, cameraMatrix, distCoeffs);
        // show live and wait for a key with timeout long enough to show images
        imshow("Live", corrected);
        waitKey(5);
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
