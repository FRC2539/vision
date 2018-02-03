#! /usr/bin/virtualenv python3
#A vision program for FRC, written in python.

import cscore as cs
import cv2
import numpy as np
from networktables import NetworkTables
from collections import namedtuple
import math

HSV = namedtuple('HSV', ('H', 'S', 'V'))
Threshhold = namedtuple('Threshhold', ('min', 'max'))
Color = namedtuple('Color', ('blue', 'green', 'red'))

color = {
    'red': Color(0, 0, 255),
    'green': Color(0, 255, 0),
    'blue': Color(255, 0, 0),
    'purple': Color(255, 0, 255),
    'yellow': Color(0, 255, 255),
    'black': Color(0, 0, 0),
    'white': Color(255, 255, 255),
    'gray': Color(127, 127, 127)
}

tapeHSV = Threshhold(
    HSV(0, 0, 230),
    HSV(179, 25, 255)
)
cubeHSV = Threshhold(
    HSV(30, 50, 50),
    HSV(60, 255.0, 250)
)

swapColor = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
swapBW = np.zeros(shape=(480, 640, 1), dtype=np.uint8)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

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
    global swapColor, swapBW, kernel

    cv2.cvtColor(img, cv2.COLOR_BGR2HSV, dst=swapColor)
    cv2.inRange(swapColor, threshhold.min, threshhold.max, dst=swapBW)
    cv2.erode(swapBW, kernel, dst=swapBW)
    cv2.dilate(swapBW, kernel, dst=swapBW)

    swapBW, contours, hierarchy = cv2.findContours(
        swapBW,
        cv2.RETR_LIST,
        cv2.CHAIN_APPROX_SIMPLE
    )

    return contours


def findSwitch(img):
    contours = findContours(img, tapeHSV)
    #cv2.drawContours(img, contours, -1, color['gray'])
    relevant = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 200:
            continue

        box = cv2.minAreaRect(contour)

        # Checks if width is less than height.

        if box[1][0] < box[1][1]:
            # Ignore if too rotated
            if box[2] < -10:
                continue

            # Ignore if too skinny
            if box[1][0] < 5.0:
                continue

            ratio = box[1][0] / box[1][1]

        else:
            # Ignore if too rotated
            if box[2] > -80:
                continue

            # Ignore if too skinny
            if box[1][1] < 5.0:
                continue

            ratio = box[1][1] / box[1][0]

        # Ignore if wrong shape (2" x 15.3")
        if ratio < 0.08 or ratio > 0.2:
            continue

        # Ignore if too concave
        hull = cv2.convexHull(contour)
        solidity = area / cv2.contourArea(hull)
        if solidity < 0.8:
            continue



        relevant.append(cv2.boundingRect(contour))

    # We need at least 2 boxes to make a target
    if  len(relevant) < 2:
        targets.putBoolean("switchVisible", False)
        return

    relevant.sort(key=lambda x: x[0])
    switches = []
    while len(relevant) > 1:

        box1 = relevant.pop(0)

        for box2 in relevant:

            # Are the boxes next to each other?
            if abs(box1[1] - box2[1]) > .25 * box1[3]:
                continue

            # Are the boxes the same size?
            if abs(box1[3] - box2[3]) > .25 * box1[3]:
                continue

            width = box2[0] +  box2[2] - box1[0]
            yPoints = [box1[1], box2[1], box1[1] + box1[3], box2[1] + box2[3]]
            yPoints.sort()
            height = yPoints[3] - yPoints[0]
            ratio = width / height

            # Ignore if wrong shape (8" x 15.3")
            if ratio < 0.3 or ratio > 0.6:
                continue

            switches.append((box1[0], yPoints[0], width, height))

    targets.putBoolean('switchVisible', len(switches) > 0)

    for target in switches:
        cv2.rectangle(img, (target[0], target[1]), (target[0] + target[2], target[1] + target[3]), color['red'], 3)
        distance = 5543.635 * math.pow(target[2], -0.9634221)
        targets.putValue('switchDistance', distance)
        print(distance)


def findCubes(img):
    contours = findContours(img, cubeHSV)
    relevant = []
    target = None

    #cv2.drawContours(img, contours, -1, color['yellow'])

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 1000:
            continue

        solidity = 100 * area / cv2.contourArea(cv2.convexHull(contour))
        if solidity < 70.0:
            continue

        relevant.append(cv2.boundingRect(contour))

    if len(relevant) > 0:
        target = relevant[0]
        targetArea = target[2] * target[3]
        ratio = None

        for box in relevant:
            if box[2] * box[3] > targetArea:
                target = box

        ratio = (target[2] / target[3])

        if ratio > 1.1 and ratio < 1.7:
            cv2.rectangle(img, (target[0], target[1]), (target[0] + target[2], target[1] + target[3]), color['blue'], 3)
        else:
            cv2.rectangle(img, (target[0], target[1]), (target[0] + target[2], target[1] + target[3]), color['purple'], 3)

        height = target[3]
        distance = 8654.642 * math.pow(target[3], -1.037359)
        targets.putValue('cubeDistance', distance)
        print(distance)


if __name__  == '__main__':
    main()
