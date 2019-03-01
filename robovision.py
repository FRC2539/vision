#! /usr/bin/env python3
#A vision program for FRC, written in python.

import cscore as cs
import cv2
import numpy as np
from networktables import NetworkTables
from collections import namedtuple
import math
from multiprocessing import Process, Queue
from pipelines import tapetarget

import socket
#import argparse
#parser = argparse.ArgumentParser("simple_udp_example")
#parser.add_argument("message", help="Message to Send.", type=str)
#args = parser.parse_args()
#print(args.counter + 1)

#UDP_IP = "10.25.39.2"
#UDP_PORT = 5809

pipeline = tapetarget.TapeTarget()

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

'''
        self.__hsv_threshold_hue = [0.0, 180.0]
        self.__hsv_threshold_saturation = [0.0, 107.8013582342954]
        self.__hsv_threshold_value = [222.4370503597122, 255.0]
    tapeHSV = Threshold(
    HSV(0, 0, 235),
    HSV(102, 25, 255)
)
self.__hsv_threshold_hue = [85.79136690647482, 95.52901023890786]
        self.__hsv_threshold_saturation = [0.0, 152.73890784982936]
        self.__hsv_threshold_value = [176.30978570193577, 254.7360446947415]

        tapeHSV = Threshold(
    HSV(0, 0, 222),
    HSV(180, 107, 255)
)


tapeHSV = Threshold(
    HSV(85, 0, 176),
    HSV(95, 152, 254)
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
'''

def main():

    #raw 2nd camera test, no processing, put first so it executes before infinite loop for processing

    width=320
    height=240
    fps=30

    #cs.startAutomaticCapture(dev=None, name=None, path=None, camera=None, return_server=False)
    #was 2
    camera0 = cs.UsbCamera("usbcam", 0)
    camera0.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, width, height, fps)

    camera0.getProperty("exposure_auto").set(abs(cameraTable.getNumber('processCameraExposureAuto', 1)))
    camera0.getProperty("exposure_absolute").set(abs(cameraTable.getNumber('processCameraExposureAbsolute', 16)))
    #camera0.getProperty("gamma").set(abs(cameraTable.getNumber('processCameraGamma', 50))) #12 day or 52 night
    #camera0.getProperty("white_balance_temperature_auto").set(abs(cameraTable.getNumber('processCameraWBAuto', 0)))
    #camera.getProperty("brightness").set(abs(cameraTable.getNumber('processCameraBrightness', 15))) #20 day or 40 night

    mjpegServer0 = cs.MjpegServer("httpserver", 5801)
    mjpegServer0.setSource(camera0)

    cvsink0 = cs.CvSink("cvsink")
    cvsink0.setSource(camera0)



    #processed camera

    setCamera()

    #while True:
    #    print("Ready")




def setCamera():
    print("starting process camera")

    pcNumber = abs(cameraTable.getNumber('processCameraNumber', 2))
    pcPort = abs(cameraTable.getNumber('processCameraPort', 5802))
    pcWidth = abs(cameraTable.getNumber('processCameraWidth', 320))
    pcHeight = abs(cameraTable.getNumber('processCameraHeight', 240))
    pcFps = abs(cameraTable.getNumber('processCameraFps', 15 ))

    fs = cv2.FileStorage("back_camera_data.xml", cv2.FILE_STORAGE_READ)
    cameraMatrix = fs.getNode('Camera_Matrix').mat()
    distortionCoefficients = fs.getNode('Distortion_Coefficients').mat()
    fs.release()

    camera = cs.UsbCamera("usbcam", pcNumber)

    camera.getProperty("exposure_auto").set(abs(cameraTable.getNumber('processCameraExposureAuto', 0)))
    camera.getProperty("exposure_absolute").set(abs(cameraTable.getNumber('processCameraExposureAbsolute', 1)))
    camera.getProperty("gamma").set(abs(cameraTable.getNumber('processCameraGamma', 52))) #12 day or 52 night
    camera.getProperty("white_balance_temperature_auto").set(abs(cameraTable.getNumber('processCameraWBAuto', 0)))
    camera.getProperty("brightness").set(abs(cameraTable.getNumber('processCameraBrightness', 35))) #20 day or 40 night

    camera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, pcWidth, pcHeight, pcFps)

    cvSink = cs.CvSink("cvsink")
    cvSink.setSource(camera)

    cvSource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, pcWidth, pcHeight, pcFps)
    server = cs.MjpegServer("cvhttpserver", pcPort)
    server.setSource(cvSource)


    img = np.zeros(shape=(pcHeight, pcWidth, 3), dtype=np.uint8)
    fixed = np.zeros(shape=(pcHeight, pcWidth, 3), dtype=np.uint8)

    q = Queue()
    p = None

    print("cameras on")



    print("running pipeline")


    while True:


        time, img  = cvSink.grabFrame(img)
        if time == 0:
            print("Camera error:", cvSink.getError())
        else:
            cv2.undistort(img, cameraMatrix, distortionCoefficients, dst=fixed)


            #Replace process() call on fixed if vision processing is desired.
            cvSource.putFrame(process(fixed))

            '''
            p = Process(
                target=findCubes,
                args=(findContours(fixed, cubeHSV), q)
            )
            p.start()
            print('Process started')
            if not q.empty():
                targets = q.get()

            #print(p)

            if p is None or not p.is_alive():
                p = Process(
                    target=findCubes,
                    args=(findContours(fixed, cubeHSV), q)
                )
                p.start()
            '''




def process(src):
    #findTape(src)
    #findCubes(src)
    #findCargo(src)

    findTape2(src)

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



def findTape2(img):

    pipeline.process(img)
    #pl = pipeline.process(img)
    #print("got pl-" + str(pl))

    #if pipeline:
        #print("got pl")
        #contours = pl.filter_contours_output filter_contours_output
        #contours = pipeline.filter_contours_output()
    #else:
        #print("no pl")
    '''
    for contour in pipeline.find_contours_output:
        #print("got contour")
        x, y, w, h = cv2.boundingRect(contour)
        #print("x-"+str(x))
        cv2.drawContours(img, contour, -1, color['red'])
        cv2.circle(img, (x, y), 10, color['neon'], 3)
    '''

    contours = pipeline.find_contours_output #findContours(img, tapeHSV)
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
        topLeftX, topLeftY, width, height = cv2.boundingRect(contour)


    relevant.sort(key=lambda x: x[0])
    #cv2.putText(img, 'relevant length ' + str(len(relevant)), (100, 450), cv2.FONT_HERSHEY_COMPLEX_SMALL, .5, (255,255,255))
    switches = []

    finalCenter = 0
    distance = 0

    while len(relevant) > .5:
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
            if abs(box1[3] - box2[3]) > .25 * box1[3]:
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


            #tapeStrafe = (finalCenter - (640/2))/640 * 5
            #cv2.putText(img, 'tapeStrafe: ' + str(tapeStrafe), (100, 300), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5, (255, 255, 100))



            width = box2[0] +  box2[2] - box1[0]
            yPoints = [box1[1], box2[1], box1[1] + box1[3], box2[1] + box2[3]]
            yPoints.sort()

            height = yPoints[3] - yPoints[0]

            ratio = width / height

            # Ignore if wrong shape (8" x 15.3")
            #if ratio < 0.3 or ratio > 0.6:
             #   continue

            switches.append((box1[0], yPoints[0], width, height))


    tMessage = ""
    hasTape = False
    tapeStrafe = -1
    tapeDistance = 0

    if switches:
        #tswitches = np.asarray(switches[0])
        if switches:
            hasTape = True

            cameraTable.putBoolean('tapeFound', hasTape)

        
            cameraTable.putNumber('distanceToTape', int(distance))
            tapeX = (finalCenter - (320/2))/320 * 35
            cameraTable.putNumber('tapeX', tapeX)
            tapeDistance = int(distance)


            bottomRightX = int(topLeftX + width)
            bottomRightY = int(topLeftY + height)

            """
            centerX = box1[0]
            centerY = box1[1]
            width = box1[2]
            height = box1[3]
            """

            #cv2.putText(img, 'bottomRightY: ' + str(bottomRightY), (200, 26), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5, (255, 255, 100))

            #cv2.putText(img, 'box1: ' + str(box1), (200, 240), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5, (255, 255, 255))

            cv2.rectangle(img, (switches[0][0], switches[0][1]), (switches[0][0] + switches[0][2], switches[0][1] + switches[0][3]), color['green'], 3)

            #cv2.rectangle(img, (topLeftX, topLeftY), (topLeftX + width, topLeftY + height), (0, 0, 255), 3)
            #cv2.rectangle(img, (switches[0][0], switches[0][1]), (400, 240), color['red'], 3)



            cv2.putText(img, 'X: '+str(tapeX), (20,20), cv2.FONT_HERSHEY_COMPLEX_SMALL, .5, (255, 255, 255))
            cv2.putText(img, 'D: '+str(tapeDistance), (40,40), cv2.FONT_HERSHEY_COMPLEX_SMALL, .5, (255, 255, 255))



    tMessage = "tapeFound:"+str(hasTape)+",tapePos:"+str(tapeStrafe)+",tapeDistance:"+str(tapeDistance)


    #sendUdp(tMessage)

def sendUdp(message):

    print("sending udp message:" +  str(message))

    #for i in range(1000):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))

def findTape(img):


    contours = findContours(img, tapeHSV)
    cv2.drawContours(img, contours, -1, color['gray'])
    relevant = []
    temp_height = 350
    cameraTable.putBoolean('tapeFound', False)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 15:
            continue

        box = cv2.minAreaRect(contour)
        # Checks if width is less than height.
        relevant.append(cv2.boundingRect(contour))



    relevant.sort(key=lambda x: x[0])
    cv2.putText(img, 'relevant length ' + str(len(relevant)), (100, 450), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255,255,255))
    switches = []


    while len(relevant) > .30:
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
            if abs(box1[3] - box2[3]) > .25 * box1[3]:
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
        hasTape = True
        cameraTable.putBoolean('tapeFound', hasTape)

        if hasTape:
            cameraTable.putNumber('tapeX', finalCenter)
            cameraTable.putNumber('distanceToTape', int(distance))
        cv2.rectangle(img, (switches[0][0], switches[0][1]), (switches[0][0] + switches[0][2], switches[0][1] + switches[0][3]), color['green'], 3)


def findCubes(img,none):
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

            if radius < -10:
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
