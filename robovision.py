#! /usr/bin/env python3
# A vision program for FRC, written in python.

from cameratools import Threshhold, HSV, colors, Target, Camera

tapeHSV = Threshhold(
    HSV(0, 0, 230),
    HSV(179, 25, 255)
)
cubeHSV = Threshhold(
    HSV(30, 50, 50),
    HSV(60, 255.0, 250)
)

def main():
    camera = Camera(0)
    camera.addProcessor('switch', tapeHSV, findSwitch, color=colors['blue'])
    #camera.addProcessor('cube', cubeHSV, findCubes, color=colors['yellow'])
    Camera.startVision()


def findSwitch(contours, q):
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

            width = box2[0] + box2[2] - box1[0]
            yPoints = [box1[1], box2[1], box1[1] + box1[3], box2[1] + box2[3]]
            yPoints.sort()
            height = yPoints[3] - yPoints[0]
            ratio = width / height

            # Ignore if wrong shape (8" x 15.3")
            if ratio < 0.3 or ratio > 0.6:
                continue

            outline = [
                (box1[0], box1[1]),
                (box1[0], box1[1] + box1[3]),
                (box2[0] + box2[2], box2[1] + box2[3]),
                (box2[0] + box2[2], box2[1])
            ]
            switches.append((box1[0], yPoints[0], width, height, outline))

    if len(switches) > 0:
        distance = 5543.635 * math.pow(switches[0][2], -0.9634221)
        q.put(Target(
            distance,
            switches[0][0] + width/2,
            switches[0][1] + height/2,
            switches[0][4]
        ))


def findCubes(contours, q):
    relevant = []
    target = None

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


if __name__  == '__main__':
    main()
