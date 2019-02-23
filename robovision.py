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

driveCameraWidth = 320
driveCameraHeight = 240
driveCameraFps = 15

processCameraWidth = 640
processCameraHeight = 480
processCameraFps = 30


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
'''

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

def main():

    #raw 2nd camera test, no processing, put first so it executes before infinite loop for processing

    width=640
    height=480
    fps=30

    '''
    camera0 = cs.UsbCamera("usbcam", 0)
    camera0.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, width, height, fps)

    mjpegServer0 = cs.MjpegServer("httpserver", 5800)
    mjpegServer0.setSource(camera0)

    cvsink0 = cs.CvSink("cvsink")
    cvsink0.setSource(camera0)


    camera1 = cs.UsbCamera("usbcam", 1)
    camera1.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, width, height, fps)

    mjpegServer1 = cs.MjpegServer("httpserver", 5801)
    mjpegServer1.setSource(camera1)

    cvsink1 = cs.CvSink("cvsink")
    cvsink1.setSource(camera1)



    camera2 = cs.UsbCamera("usbcam", 1)
    camera2.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, width, height, fps)

    mjpegServer2 = cs.MjpegServer("httpserver", 5803)
    mjpegServer2.setSource(camera2)

    cvsink2 = cs.CvSink("cvsink")
    cvsink2.setSource(camera2)




    camera3 = cs.UsbCamera("usbcam", 3)
    camera3.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, width, height, fps)

    mjpegServer3 = cs.MjpegServer("httpserver", 5804)
    mjpegServer3.setSource(camera3)

    cvsink3 = cs.CvSink("cvsink")
    cvsink3.setSource(camera3)




    camera4 = cs.UsbCamera("usbcam", 4)
    camera4.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, width, height, fps)

    mjpegServer4 = cs.MjpegServer("httpserver", 5804)
    mjpegServer4.setSource(camera4)

    cvsink4 = cs.CvSink("cvsink")
    cvsink4.setSource(camera4)



    camera5 = cs.UsbCamera("usbcam", 5)
    camera5.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, width, height, fps)

    mjpegServer5 = cs.MjpegServer("httpserver", 5805)
    mjpegServer5.setSource(camera5)

    cvsink5 = cs.CvSink("cvsink")
    cvsink5.setSource(camera5)

    '''

    #processed camera

    setCamera()

    #while True:
    #    print("Ready")




def setCamera():
    print("starting process camera")

    pcNumber = abs(cameraTable.getNumber('processCameraNumber', 0))
    pcPort = abs(cameraTable.getNumber('processCameraPort', 5801))
    pcWidth = abs(cameraTable.getNumber('processCameraWidth', 640))
    pcHeight = abs(cameraTable.getNumber('processCameraHeight', 480))
    pcFps = abs(cameraTable.getNumber('processCameraFps', 30))

    fs = cv2.FileStorage("back_camera_data.xml", cv2.FILE_STORAGE_READ)
    cameraMatrix = fs.getNode('Camera_Matrix').mat()
    distortionCoefficients = fs.getNode('Distortion_Coefficients').mat()
    fs.release()

    camera = cs.UsbCamera("usbcam", pcNumber)


    camera.getProperty("exposure_auto").set(abs(cameraTable.getNumber('processCameraExposureAuto', 1)))
    camera.getProperty("exposure_absolute").set(abs(cameraTable.getNumber('processCameraExposureAbsolute', 1)))
    camera.getProperty("gamma").set(abs(cameraTable.getNumber('processCameraGamma', 39))) #12 day or 52 night
    camera.getProperty("white_balance_temperature_auto").set(abs(cameraTable.getNumber('processCameraWBAuto', 0)))
    camera.getProperty("brightness").set(abs(cameraTable.getNumber('processCameraBrightness', 45))) #20 day or 40 night

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
    #cameraTable.putBoolean('tapeFound', False)

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

        #cameraTable.putBoolean('tapeFound', False)
        #cameraTable.putNumber('distanceToTape', -1)
        #cameraTable.putNumber('tapeX', -1)

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

    if len(switches) >= 0:
        hasTape = True


        tapeX = (finalCenter - (640/2))/640 * 35
        tapeDistance = int(distance)

        cameraTable.putBoolean('tapeFound', hasTape)

        cameraTable.putNumber('tapeX', tapeX)
        cameraTable.putNumber('distanceToTape', int(distance))

        cv2.putText(img, 'tapex: ' + str(tapeX), (50, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5, (255, 255, 100))

        cv2.rectangle(img, (switches[0][0], switches[0][1]), (switches[0][0] + switches[0][2], switches[0][1] + switches[0][3]), color['green'], 3)
    else:
        cameraTable.putBoolean('tapeFound', False)

        cameraTable.putNumber('tapeStrafe', finalCenter)
        cameraTable.putNumber('distanceToTape', int(distance))
        tapeStrafe = (finalCenter - (640/2))/640 * 35
        tapeDistance = int(distance)


        bottomRightX = int(topLeftX + width)
        bottomRightY = int(topLeftY + height)

        """
        centerX = box1[0]
        centerY = box1[1]
        width = box1[2]
        height = box1[3]
        """

        cv2.putText(img, 'bottomRightY: ' + str(bottomRightY), (200, 26), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5, (255, 255, 100))

        cv2.putText(img, 'box1: ' + str(box1), (200, 240), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5, (255, 255, 255))

        cv2.rectangle(img, (switches[0][0], switches[0][1]), (switches[0][0] + switches[0][2], switches[0][1] + switches[0][3]), color['green'], 3)

        cv2.rectangle(img, (topLeftX, topLeftY), (topLeftX + width, topLeftY + height), (0, 0, 255), 3)
#        cv2.rectangle(img, (switches[0][0], switches[0][1]), (400, 240), color['red'], 3)



        cv2.putText(img, 'C', (bottomRightX, bottomRightY), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5, (255, 255, 255))


    tMessage = "tapeFound:"+str(hasTape)+",tapePos:"+str(tapeStrafe)+",tapeDistance:"+str(tapeDistance)


    sendUdp(tMessage)






















def findSingleTape(img, screenWidth):
    secondScreenWidth = screenWidth

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
    cv2.drawContours(img, contours, -1, color['yellow'])
    relevant = []
    temp_height = 350
    cameraTable.putBoolean('tapeFound', False)
    bensBoxes = []
    slantedBois = []
    boxes = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < 60:

            cv2.putText(img, ' AREA ERROR', (50, 350), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5, (255, 255, 100))
            continue

        solidity = 100 * area / cv2.contourArea(cv2.convexHull(contour))

        if solidity < 95.0:
            continue

        if cv2.boundingRect(contour)[2] >= cv2.boundingRect(contour)[3]:
            continue

        topLeftX, topLeftY, width, height = cv2.boundingRect(contour)
        slantedBois.append(cv2.minAreaRect(contour))


        #box = cv2.minAreaRect(contour)

        # Checks if width is less than height.
        relevant.append(cv2.boundingRect(contour))

        try:
            if relevant[0][2] > 50:
                continue
            else:
                #topLeftX, topLeftY, width, height = cv2.boundingRect(contour)
                #bensBoxes.append(cv2.boundingRect(contour))
                #slantedBois.append(cv2.minAreaRect(contour))
                pass
        except IndexError:
            continue

    relevant.sort(key=lambda x: x[0])
    switches = []
    distances = []
    chooseHeights = []

    finalCenter = 0
    distance = 0


    while len(relevant) > .5:
        cameraTable.putBoolean('tapeFound', False)
        cameraTable.putNumber('tapeDistance', -1)
        cameraTable.putNumber('tapeStrafe', -1)

        box1 = relevant.pop(0)
        print('relevant')
        for box2 in relevant:

            sizeLimit = box1[1] * 0.2
            heightLimit = box1[3] * 0.5

            #Are the boxes next two each other? 2.0!!!
            """
            if abs(box1[0] - box2[0]) > 10 * box1[2]:
                cv2.putText(img, ' ERROR is...' + str(topLeftX), (320, 300), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5, (255, 255, 100))
                continue
            """

            # Are the boxes next to each other?
            if abs(box1[1] - box2[1]) > .25 * box1[2]:
                continue

            # Are the boxes the same size?
            if abs(box1[3] - box2[3]) > .25 * box1[3]:
                continue

            # Funky, is there a box inside of a box?

            if abs((box1[0] + box1[2] + sizeLimit)) > box2[0] and abs(box1[0] - sizeLimit) < box2[0]:
                continue

            if abs(box1[2] - box2[2]) > heightLimit:
                continue

            if len(bensBoxes) >= 3:
                bensBoxes = sorted(bensBoxes, key=lambda boxx:abs(boxx[0]-secondScreenWidth))
                bensBoxes = bensBoxes[0:2]

            bensBoxes.append(box1)
            bensBoxes.append(box2)



            #TODO: Work out the kinks in the focusing on one object (below)


            """
            if abs(box1[1] - box2[1]) * 10 > 15:
                continue

            cv2.putText(img, 'difference: ' + str(abs(box1[2] - box2[2])), (box2[0], box2[1]), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5, (255, 255, 255))
            """


            """
           # individualDistance =
            chooseHeights.append(abs(box1[3] - box2[3]))
            if len(chooseHeights) > 1:
                if chooseHeights[0] < abs(box1[3] - box2[3]):
                    continue
            """
            if box1[0] > box2[0]:
                greaterXVal = box1[0]
                smallerXVal = box2[0]

            else:
                greaterXVal = box2[0]
                smallerXVal = box1[0]

            displacement = greaterXVal - smallerXVal
            centerDisplacement = displacement / 2
            finalCenter = smallerXVal + centerDisplacement

            #tapeStrafe = (finalCenter - (640/2))/640 * 5
            #cv2.putText(img, 'tapeStrafe: ' + str(tapeStrafe), (100, 300), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5, (255, 255, 100))

           # width = relevant[0][2]
            yPoints = [box1[1], box2[1], box1[1] + box1[3], box2[1] + box2[3]]
            yPoints.sort()
            height = yPoints[3] - yPoints[0]
            width = box2[2]

            switches.append((box1[0], yPoints[0], width, height))


    tMessage = ""
    hasTape = False
    tapeAngle = -1
    tapeStrafe = -1
    tapeDistance = 0

    if len(bensBoxes) > 1:
        hasTape = True



        """
        centerX = box1[0]
        centerY = box1[1]
        width = box1[2]
        height = box1[3]
        """
        firstHeight = int(bensBoxes[0][3])

        try:
            secondHeight = int(bensBoxes[1][3])
            chooseHeights.append((firstHeight, secondHeight))

        except IndexError:
            chooseHeights.append((firstHeight))


        #cv2.putText(img, ' LENGTH OF BB' + str(len(bensBoxes)), (200, 200), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5, (255, 255, 100))
>>>>>>> d9e491a526955b4bf68635d5acaecfcf5ae79edd

        topRightX2Displacement = 0
        topLeftY2 = 0

        moreThanTwo = True
        while moreThanTwo:
            bensBoxes = sorted(bensBoxes, key=lambda boxx:abs(boxx[0]-secondScreenWidth))

            #if len(bensBoxes) < 2 and len(bensBoxes) >= 1:

                #topLeftX = bensBoxes[0][0]
                #topLeftY = bensBoxes[0][1]
                #width = bensBoxes[0][2]
                #height = bensBoxes[0][3]

                #bottomRightX = int(topLeftX + width)
                #bottomRightY = int(topLeftY + height)

                #topLeftX2 = 0
                #topLeftY2 = 0

                #topRightX2Displacement = 0

                #try:
                    #rect = slantedBois[0]
                    #box = cv2.boxPoints(rect)
                    #box = np.int0(box)

                    #cv2.drawContours(img, [box], -1, (255, 0, 0), 3)


                #except IndexError:
                    #pass

            if len(bensBoxes) == 2:
                bensBoxes = sorted(bensBoxes, key=lambda box: box[0])
                finalBoxes = bensBoxes
                topLeftX = finalBoxes[0][0]
                topLeftY = finalBoxes[0][1]
                width = finalBoxes[0][2]
                height = finalBoxes[0][3]

                topLeftX2 = finalBoxes[1][0]
                topLeftY2 = finalBoxes[1][1]
                width2 = finalBoxes[1][2]
                height2 = finalBoxes[1][3]

                bottomRightX = int(topLeftX + width)
                bottomRightY = int(topLeftY + height)

                bottomRightX2 = int(topLeftX2 + width2)
                bottomRightY2 = int(topLeftY2 + height2)

                rect = slantedBois[0]
                rect2 = slantedBois[1]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                box2 = cv2.boxPoints(rect2)
                box2 = np.int0(box2)

                screenWidth *= 2
                topRightX2Displacement = abs(screenWidth - (bottomRightX2))

                cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
                cv2.drawContours(img, [box2], -1, (255, 0, 0), 3)

                distanceBetweenObject = abs(finalBoxes[0][0] - finalBoxes[1][0])

                distance = 16.55649 + (155.5885 * math.exp(-0.00888356 * int(distanceBetweenObject)))
                moreThanTwo = False

            else:
                bensBoxes = sorted(bensBoxes, key=lambda box: box[0])
                finalBoxes = bensBoxes
                if finalBoxes[0] == finalBoxes[1]:
                    del finalBoxes[1]
                finalBoxes = finalBoxes[0:2]
                continue

        pixelSpan = int(abs(distance * 0.02))

        heightDifferential = 0

        if topLeftX < topRightX2Displacement:
            val = -1 * abs(topRightX2Displacement - topLeftX)
            cameraTable.putString('tapeStrafe', val)
        else:
            val = abs(topLeftX - topRightX2Displacement)
            cameraTable.putString('tapeStrafe', val)

        cameraTable.putBoolean('tapeFound', hasTape)

        tapeAngle = (finalCenter - (640/2))/640 * 35
        cameraTable.putNumber('tapeStrafe', tapeAngle)

        cameraTable.putNumber('tapeStrafe', val)

        tapeDistance = int(distance)
        cameraTable.putNumber('tapeDistance', tapeDistance)

        cv2.putText(img, ' tape angle ' + str(tapeAngle), (50, 260), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5, (255, 255, 100))
        cv2.putText(img, ' tape distance ' + str(tapeDistance), (50, 285), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5, (255, 255, 100))

        tMessage = "tapeFound:"+str(hasTape)+",tapeAngle:"+str(tapeAngle)+",tapeDistance:"+str(tapeDistance)+",tapeStrafe:"+str(val)


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
    switches = []


    while len(relevant) > .30:
        cameraTable.putBoolean('tapeFound', False)
        cameraTable.putNumber('distanceToTape', -1)
        cameraTable.putNumber('tapeStrafe', -1)

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

            switches.append((box1[0], yPoints[0], width, height))



    if len(switches) > 0:
        hasTape = True
        cameraTable.putBoolean('tapeFound', hasTape)

        if hasTape:
            cameraTable.putNumber('tapeStrafe', finalCenter)
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
