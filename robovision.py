
#A vision program for FRC, written in python.

import cscore as cs
import cv2
import numpy as np
import math
from networktables import NetworkTables

hsvH = [0.0, 255]
hsvS = [0.0, 5]
hsvV = [235, 255]

hsvIn = None
hsvOut = None

contours = None
filteredContours = None

minArea = 40.0
minWidth = 2.0
minHeight = 2.0
minSolidity = 60.0
minRatio = 0.3
maxRatio = 1000.0

red = (0, 0, 255)

output = []
contoursOut = []

def main():

    NetworkTables.setTeam(2539);
    NetworkTables.setClientMode();
    targetInfo = NetworkTables.getTable("cameraTarget")

    frontCamera = cs.UsbCamera("usbcam", 2)
    backCamera = cs.UsbCamera("usbcam", 0)

    frontCamera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)
    backCamera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)

    frontCvSink = cs.CvSink("cvsink")
    frontCvSink.setSource(frontCamera)

    frontCvSource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)
    frontServer = cs.MjpegServer("cvhttpserver", 8081)
    frontServer.setSource(frontCvSource)

    print("Front mjpg server listening at port 8081")

    backCvSink = cs.CvSink("cvsink")
    backCvSink.setSource(backCamera)

    backCvSource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)
    backServer = cs.MjpegServer("cvhttpserver", 8082)
    backServer.setSource(backCvSource)

    print("Back mjpg server listening at port 8082")

    front = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
    back = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

    while True:

        frontTime, front = frontCvSink.grabFrame(front)
        backTime, back = backCvSink.grabFrame(back)

        if frontTime == 0:
            print("error:", frontCvSink.getError())
            continue
        if backTime == 0:
            print("error:", backCvSink.getError())
            continue

        frontCvSource.putFrame(filter_process(front))
        backCvSource.putFrame(filter_process(back))


def filter_process(src):

        output = []

        #HSV
        (hsvIn) = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        (hsvOut) = cv2.inRange(hsvIn, (hsvH[0], hsvS[0], hsvV[0]),  (hsvH[1], hsvS[1], hsvV[1]))


        #Find Contours
        img, contours, hierarchy = cv2.findContours(hsvOut, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        #Filter Contours
        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)
            if (w < minWidth):
                continue
            if (h < minHeight):
                continue
            area = cv2.contourArea(contour)
            if (area < minArea):
                continue
            hull = cv2.convexHull(contour)
            solidity = 100 * area / cv2.contourArea(hull)
            if (solidity < minSolidity):
                continue
            ratio = (float)(w) / h
            if (ratio < minRatio or ratio > maxRatio):
                continue
            output.append(contour)

        return cv2.drawContours(src, output, -1, red, 2, cv2.LINE_8)


if __name__  == '__main__':
    main()
