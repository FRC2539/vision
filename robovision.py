#! /usr/bin/env python3
#A vision program for FRC, written in python.

import cscore as cs
import cv2
import numpy as np
from networktables import NetworkTables
from collections import namedtuple
import math
from multiprocessing import Process, Queue

NetworkTables.initialize(server='roborio-2539-frc.local')

cameraTable = NetworkTables.getTable('cameraInfo')

HSV = namedtuple('HSV', ('H', 'S', 'V'))
Threshold = namedtuple('Threshold', ('min', 'max'))
Color = namedtuple('Color', ('blue', 'green', 'red'))

color = {
    'red': Color(0, 0, 255),
    'green': Color(0, 255, 0),
    'blue': Color(255, 0, 0),
    'purple': Color(255, 0, 255),
    'yellow': Color(0, 255, 255),
    'black': Color(0, 0, 0),
    'white': Color(255, 255, 255),
    'gray': Color(127, 127, 127),
    'neon': Color(244, 134, 66)
}

tapeHSV = Threshold(
    HSV(0, 0, 235),
    HSV(102, 21, 255)
)

cubeHSV = Threshold(
    HSV(30, 50, 50),
    HSV(60, 255.0, 250)
)
cargoHSV = Threshold(
    HSV(0, 115, 105),
    HSV(23, 255, 255)
)

swapColor = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
swapBW = np.zeros(shape=(480, 640, 1), dtype=np.uint8)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

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
    server = cs.MjpegServer("cvhttpserver", 5801)
    server.setSource(cvSource)

    img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
    fixed = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

    q = Queue()
    p = None

    while True:

        time, img  = cvSink.grabFrame(img)
        if time == 0:
            print("Camera error:", cvSink.getError())
        else:
            cv2.undistort(img, cameraMatrix, distortionCoefficients, dst=fixed)

            #cv2.putText(img,"Howdy", (0,0), cv2.Font_Hershey_Simplex, 2,255)

            #Replace process() call on fixed if vision processing is desired.
            cvSource.putFrame(process(fixed))

            p = Process(
                target=findCubes,
                args=(findContours(fixed, cubeHSV), q)
            )
            p.start()
            print('Process started')
            if not q.empty():
                targets = q.get()

            print(p)

            if p is None or not p.is_alive():
                p = Process(
                    target=findCubes,
                    args=(findContours(fixed, cubeHSV), q)
                )
                p.start()


def process(src):
    findTape(src)
    #findCubes(src)
    findCargo(src)

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


def findTape(img):

    contours = findContours(img, tapeHSV)
    cv2.drawContours(img, contours, -1, color['gray'])
    relevant = []
    temp_height = 350

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 25:
            continue

        box = cv2.minAreaRect(contour)
        # Checks if width is less than height.

        relevant.append(cv2.boundingRect(contour))

    """
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


    # We need at least 2 boxes to make a target

    if  len(relevant) < 2:
        q.put([False, 0])
        return

    """
    relevant.sort(key=lambda x: x[0])
    cv2.putText(img, 'relevant length ' + str(len(relevant)), (100, 450), cv2.FONT_HERSHEY_COMPLEX_SMALL, .5, (255,255,255))
    switches = []


    while len(relevant) > 1:

        box1 = relevant.pop(0)
        i_hate_this_code = 200
        for box2 in relevant:

            # Are the boxes next to each other?
            stringy = 'box[0]: ' + str(box1[0]) + '\nbox[1]: ' + str(box1[1]) + '\nbox[2]: ' + str(box1[2]) + '\nbox[3]: ' + str(box1[3])
            cv2.putText(img, str(stringy), (100, i_hate_this_code), cv2.FONT_HERSHEY_COMPLEX_SMALL, .5, (255,255,255))
            i_hate_this_code += 50
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
            #if ratio < 0.3 or ratio > 0.6:
             #   continue

            switches.append((box1[0], yPoints[0], width, height))


    cv2.putText(img, str(len(switches)), (300, temp_height), cv2.FONT_HERSHEY_COMPLEX_SMALL, .5, (255,255,255))
    if len(switches) > 0:
        cv2.rectangle(img, (switches[0][0], switches[0][1]), (switches[0][0] + switches[0][2], switches[0][1] + switches[0][3]), color['green'], 3)

    """
        distance = 5543.635 * math.pow(switches[0][2], -0.9634221)

        q.put([True, distance])
    else:
        q.put([False, 0])
    """

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
        cameraTable.putNumber('cubeDistance', int(distance))
        print(distance)

def findCargo(img):
    relevantXCenters = []
    relevantYCenters = []
    relevantRadius = []
    contours = findContours(img, cargoHSV)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 10000:
            continue

        cv2.putText(img, 'area: ' + str(area), (350, 400), cv2.FONT_HERSHEY_COMPLEX_SMALL, .5, (255,255,255))
        solidity = area / cv2.contourArea(cv2.convexHull(contour))
        if solidity < 0.6:
            continue

        center, radius = cv2.minEnclosingCircle(contour)

        relevantXCenters.append([center, radius])
        #relevantRadius.append(radius)

    cameraTable.putBoolean('cargoFound', False)

    if len(relevantXCenters) > 0:
        for circle in relevantXCenters:
            center = circle[0]
            center_x = int(center[0])
            center_y = int(center[1])
            radius = int(circle[1])
            diameter = radius * 2

            if radius < 80:
                continue

            string = 'Center: ' + str(center_x) + ', ' + str(center_y) + ':' + ' Diameter: ' + str(diameter)
            cv2.putText(img, string, (center_x, center_y), cv2.FONT_HERSHEY_COMPLEX_SMALL, .5, (255,255,255))

            cv2.circle(img, (center_x, center_y), radius, color['neon'], 3)

            cameraTable.putBoolean('cargoFound', True)

            cameraTable.putNumber('cargoX', center_x)
            cameraTable.putNumber('cargoY', center_y)

            distance = (-0.05990983 * int(diameter)) + 59.83871
            cv2.putText(img, 'distance: ' +  str(distance), (360, 360), cv2.FONT_HERSHEY_COMPLEX_SMALL, .5, (255,255,255))
            cameraTable.putNumber('distanceToCargo', distance)

if __name__  == '__main__':
    main()
