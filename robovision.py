#! /usr/bin/virtualenv python3
#A vision program for FRC, written in python.

import cscore as cs
import cv2
import numpy as np
from networktables import NetworkTables
from collections import namedtuple

HSV = namedtuple('HSV', ('H', 'S', 'V'))
Threshhold = namedtuple('Threshhold', ('min', 'max'))
Color = namedtuple('Color', ('blue', 'green', 'red'))

color = {
    'red': Color(0, 0, 255),
    'green': Color(0, 255, 0),
    'blue': Color(255, 0, 0),
    'yellow': Color(0, 255, 255),
    'black': Color(0, 0, 0),
    'white': Color(255, 255, 255),
    'gray': Color(127, 127, 127)
}

tapeHSV = Threshhold(
    HSV(100.35971223021582, 18.345323741007192, 73.38129496402877),
    HSV(112.76740237691003, 73.16638370118852, 109.96604414261459)
)
cubeHSV = Threshhold(
    HSV(17.805755395683452, 80.26079136690647, 20.638489208633093),
    HSV(68.45500848896434, 255.0, 125.11884550084889)
)

swapColor = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
swapBW = np.zeros(shape=(480, 640, 1), dtype=np.uint8)

contours = None
filteredContours = None

minArea = 50.0
minWidth = 5.0
minHeight = 5.0
minSolidity = 80.0
minRatio = 0.3
maxRatio = 0.7

red = (0, 0, 255)

x = None
y = None

NetworkTables.setServerTeam(2539)
targets = NetworkTables.getTable("cameraInfo")

def main():
    fs = cv2.FileStorage("back_camera_data.xml", cv2.FILE_STORAGE_READ)
    cameraMatrix = fs.getNode('Camera_Matrix').mat()
    distortionCoefficients = fs.getNode('Distortion_Coefficients').mat()
    fs.release()

    frontCamera = cs.UsbCamera("usbcam", "/dev/video-front")
    backCamera = cs.UsbCamera("usbcam", "/dev/video-back")

    frontCamera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)
    backCamera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)

    frontCvSink = cs.CvSink("cvsink")
    frontCvSink.setSource(frontCamera)

    frontCvSource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)
    frontServer = cs.MjpegServer("cvhttpserver", 8081)
    frontServer.setSource(frontCvSource)

    backCvSink = cs.CvSink("cvsink")
    backCvSink.setSource(backCamera)

    backCvSource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)
    backServer = cs.MjpegServer("cvhttpserver", 8082)
    backServer.setSource(backCvSource)

    front = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
    back = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
    fixed = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

    while True:

        frontTime, front = frontCvSink.grabFrame(front)
        if frontTime == 0:
            print("Front camera error:", frontCvSink.getError())
        else:
            cv2.undistort(front, cameraMatrix, distortionCoefficients, dst=fixed)
            frontCvSource.putFrame(process(fixed))

        backTime, back = backCvSink.grabFrame(back)
        if backTime == 0:
            print("Back camera error:", backCvSink.getError())
        else:
            cv2.undistort(back, cameraMatrix, distortionCoefficients, dst=fixed)
            backCvSource.putFrame(process(fixed))


def process(src):
    findSwitch(src)
    findCubes(src)

    return src


def findContours(img, threshhold):
    global swapColor, swapBW

    cv2.cvtColor(img, cv2.COLOR_BGR2HSV, dst=swapColor)
    cv2.inRange(swapColor, threshhold.min, threshhold.max, dst=swapBW)

    swapBW, contours, hierarchy = cv2.findContours(
        swapBW,
        cv2.RETR_LIST,
        cv2.CHAIN_APPROX_SIMPLE
    )

    return contours


def findSwitch(img):
    contours = findContours(img, tapeHSV)
    cv2.drawContours(img, contours, -1, color['gray'])


def findCubes(img):
    contours = findContours(img, cubeHSV)
    cv2.drawContours(img, contours, -1, color['yellow'])


if __name__  == '__main__':
    main()
