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

    camera = cs.UsbCamera("usbcam", 0)

    camera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)

    cvSink = cs.CvSink("cvsink")
    cvSink.setSource(camera)

    cvSource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)
    server = cs.MjpegServer("cvhttpserver", 8081)
    server.setSource(cvSource)

    img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
    fixed = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

    while True:

        time, img  = cvSink.grabFrame(img)
        if time == 0:
            print("Camera error:", cvSink.getError())
        else:
            cv2.undistort(img, cameraMatrix, distortionCoefficients, dst=fixed)
            cvSource.putFrame(process(fixed))


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
