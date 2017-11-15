
#A vision program for FRC, written in python.

import cv2
import numpy
import math
from networktables import NetworkTables

def __init__(self):

        NetworkTables.setTeam(2539);
        NetworkTables.setClientMode();
        targetInfo = NetworkTables.getTable("cameraTarget")

        hsvIn = source ###########################
        hsvH = [0.0, 97]
        hsvS = [0.0, 9]
        hsvV = [74, 93]

        hsvIn = None
        hsvOut = None

        findIn = None

        externalOnly = False
        contours = None

        filteredContours = contours
        minArea = 80.0
        minWidth = 2.0
        maxWidth = 10000.0
        minHeight = 2.0
        maxHeight = 10000.0
        solidity = [60, 100.0]
        minRatio = 1.0
        maxRatio = 1000.0

        output = []


def main():
    frontCamera = cs.UsbCamera("usbcam", 0)
    frontCamera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)

    mjpegServer = cs.MjpegServer("httpserver", 8081)
    mjpegServer.setSource(camera)

    print("mjpg server listening at http://0.0.0.0:8081")

    cvsink = cs.CvSink("cvsink")
    cvsink.setSource(frontCamera)

    cvSource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)
    cvMjpegServer = cs.MjpegServer("cvhttpserver", 8082)
    cvMjpegServer.setSource(cvSource)

    print("OpenCV output mjpg server listening at http://0.0.0.0:8082")

    front = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
    back = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

    while True:

        front = cvsink.grabFrame(front)
        back = cvsink.grabFrame(back)

        if time == 0:
            print("error:", cvsink.getError())
            continue

        filter_process(self, ###################################X
        cvSource.putFrame(front)
        cvSource.putFrame(back)

def filter_process(self, src):

        hsvIn = src

        #HSV
        (hsvOut) = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        (findIn) = cv2.inRange(hsvOut, (hsvH[0], hsvH[0], hsvV[0]),  (hsvH[1], hsvS[1], hsvV[1])


        #Find Contours
        (contours) = cv2.findContours(findIn, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        #Filter Contours
        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)
            if (w < minWidth or w > maxWidth):
                continue
            if (h < minHeight or h > maxHeight):
                continue
            area = cv2.contourArea(contour)
            if (area < minArea):
                continue
            hull = cv2.convexHull(contour)
            solid = 100 * area / cv2.contourArea(hull)
            if (solid < solidity[0] or solid > solidity[1]):
                continue
            ratio = (float)(w) / h
            if (ratio < minRatio or ratio > maxRatio):
                continue
            output.append(contour)
        return output


if __name__  == '__main__':
    main()
