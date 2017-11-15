#! /usr/bin/virtualenv python3
#A vision program for FRC, written in python.

import cscore as cs
import cv2
import numpy as np
from networktables import NetworkTables

hsvH = [0.0, 255]
hsvS = [0.0, 5]
hsvV = [235, 255]

hsvIn = None
hsvOut = None

contours = None
filteredContours = None

minArea = 50.0
minWidth = 5.0
minHeight = 5.0
minSolidity = 80.0
minRatio = 0.3
maxRatio = 0.7

red = (0, 0, 255)

def main():

    NetworkTables.setTeam(2539)
    NetworkTables.setClientMode()
    targetInfo = NetworkTables.getTable("cameraTarget")

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
            frontCvSource.putFrame(filter_process(fixed))

        backTime, back = backCvSink.grabFrame(back)
        if backTime == 0:
            print("Back camera error:", backCvSink.getError())
        else:
            cv2.undistort(back, cameraMatrix, distortionCoefficients, dst=fixed)
            backCvSource.putFrame(filter_process(fixed))


def filter_process(src):

        matches = []
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

        for box1 in output:
            x1,y1,w1,h1 = cv2.boundingRect(box1)
            for box2 in output:
                x2,y2,w2,h2 = cv2.boundingRect(box2)
                if abs(y1 - y2) > (.25 * h1):
                    continue
                if abs(h1 - h2) > (.25 * h1):
                    continue
                cv2.rectangle(src, (x1, y1), (x2 + w2, y2 + h2), red, 2)

        return src


if __name__  == '__main__':
    main()
