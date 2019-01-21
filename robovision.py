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
    HSV(102, 25, 255)
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
    cameraTable.putBoolean('tapeFound', False)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 25:
            continue

        box = cv2.minAreaRect(contour)
        # Checks if width is less than height.
        relevant.append(cv2.boundingRect(contour))

    """
    In theory:
    Box1[0] = x
    Box1[1] = y
    Box1[2] = width
    Box1[3] = height
    """

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
        cameraTable.putBoolean('tapeFound', False)
        cameraTable.putNumber('distanceToTape', -1)
        cameraTable.putNumber('tapeX', -1)

        box1 = relevant.pop(0)
        i_hate_this_code = 200
        for box2 in relevant:

            # Are the boxes next to each other?
            if abs(box1[1] - box2[1]) > .25 * box1[3]:
                continue

            # Are the boxes the same size?
            if abs(box1[3] - box2[3]) > .35 * box1[3]:
                continue

            if box1[0] > box2[0]:
                greaterXVal = box1[0]
                smallerXVal = box2[0]
            else:
                greaterXVal = box2[0]
                smallerXVal = box1[0]

            displacement = greaterXVal - smallerXVal
            centerDisplacement = displacement / 2
            finalCenter = smallerXVal + centerDisplacement

            distanceBetweenObject = abs(box1[0] - box2[0])

            distance = 18.55649 + (155.5885 * math.exp(-0.00888356 * int(distanceBetweenObject)))

            hasTape = True
            cameraTable.putBoolean('tapeFound', hasTape)

            if hasTape:
                cameraTable.putNumber('tapeX', finalCenter)
                cameraTable.putNumber('distanceToTape', int(distance))

            cv2.putText(img, 'distance between obj.: ' + str(distanceBetweenObject), (300, 350), cv2.FONT_HERSHEY_COMPLEX_SMALL, .5, (255, 0, 0))

            width = box2[0] +  box2[2] - box1[0]
            yPoints = [box1[1], box2[1], box1[1] + box1[3], box2[1] + box2[3]]
            yPoints.sort()
            height = yPoints[3] - yPoints[0]
            ratio = width / height

            # Ignore if wrong shape (8" x 15.3")
            #if ratio < 0.3 or ratio > 0.6:
             #   continue

            switches.append((box1[0], yPoints[0], width, height))



    if len(switches) > 0:
        cv2.rectangle(img, (switches[0][0], switches[0][1]), (switches[0][0] + switches[0][2], switches[0][1] + switches[0][3]), color['green'], 3)

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

    cameraTable.putNumber('distanceToCargo', -1)
    cameraTable.putNumber('cargoX', -1)
    cameraTable.putNumber('cargoY', -1)

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

            distanceToCargo = 9.349483 + (144.9359 * math.exp(-0.006633269 * int(diameter)))

            cameraTable.putBoolean('cargoFound', True)

            cameraTable.putNumber('distanceToCargo', int(distanceToCargo))
            cameraTable.putNumber('cargoX', center_x)
            cameraTable.putNumber('cargoY', center_y)

            #cameraTable.putNumber('distanceToCargo', distance)

if __name__  == '__main__':
    main()

